"""Orchestrates refill / dispose / normal-transfer task sequences.

Hosts /robot/start_task and /robot/stop_task. Each start_task call handles
ONE priority tier per the spec doc's Scenario D rule
(State2 dispose > State0 refill > State1 normal-transfer) and then asks
vision for a recheck. This mirrors the section-14 flowchart exactly: a
recheck loops back to image Capture rather than being handled inline here,
so the *next* /vision/tube_state update (driven by main_decision_node) is
what decides the following action. Multi-tier requests therefore resolve
over a few reactive cycles instead of one long blocking call.
"""
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Trigger

from interfaces.msg import RobotStatus, TubeState
from interfaces.srv import (
    GripperControl,
    MoveToPose,
    RequestRecheck,
    StartTask,
    StopTask,
    CurrentTubeState,
    MarkTubeDisposed,
)
from robot_control_pkg.poses import (
    HOME_POSE,
    WASTE_APPROACH_POSE,
    WASTE_RELEASE_POSE,
    WASTE_ROTATE_POSE,
    WASTE_ROTATE_MIDDLE_POSE,
    WASTE_ROTATE_REAGENT_POSE,
    WASTE_ROTATE_TUBE_POSE,
    TRAY_TOOL_STAND_APPROACH_POSE,
    TRAY_TOOL_STAND_GRIP_POSE,
    REFILL_SOURCE_APPROACH_POSE,
    REFILL_SOURCE_GRIP_POSE,
    tray_transfer_tool_approach_pose,
    tray_transfer_tool_preinsert_pose,
    tray_transfer_tool_insert_pose,
    tray_transfer_lift_pose,
    tray_transfer_success_approach_pose,
    tray_transfer_success_place_pose,
    tray_transfer_tool_detach_pose,
    refill_target_approach_pose,
    refill_target_pour_pose,
    dispose_tube_approach_pose,
    dispose_tube_grip_pose,
)


class TaskAborted(Exception):
    """Raised internally when an emergency stop interrupts a task sequence."""


class TaskFailed(Exception):
    """Raised internally when a motion/gripper step reports failure."""


class RobotTaskManagerNode(Node):
    def __init__(self):
        super().__init__("robot_task_manager_node")

        self.declare_parameter("grip_retry_count", 1)
        self.declare_parameter("recheck_timeout_sec", 5.0)
        self.declare_parameter("service_call_timeout_sec", 100.0)
        # 260624 jiwan refill 반복 보충 루프의 안전 상한 (무한루프 방지)
        self.declare_parameter("max_pour_attempts", 5)
        # NORMAL 판정을 N번 연속으로 받아야 refill 완료로 인정.
        # YOLO 순간 오차로 인한 조기 종료 방지.
        self.declare_parameter("normal_confirm_count", 5)
        # end
        # 트레이는 깊이 방향으로 3줄(num_trays) 쌓여있고, 항상 맨 앞줄(tray_idx)만
        # 활성 상태. transfer_tray()가 끝날 때마다 tray_idx를 올려서 다음 줄로 넘어감.
        self.declare_parameter("num_trays", 3)
        self.tray_idx = 0
        #20260624 준형, grip_retry_count, recheck_timeout_sec, service_call_timeout_sec를 사용할 때 get_parameter()로 가져오도록 수정
        # self.grip_retry_count = int(self.get_parameter("grip_retry_count").value)
        # self.recheck_timeout_sec = float(self.get_parameter("recheck_timeout_sec").value)
        # self.service_call_timeout_sec = float(self.get_parameter("service_call_timeout_sec").value)
        #end

        #20260627 JH, HMI로 기능 이전
        # #20260626 JH, HMI에 publish할 때, 기존 값과 달라야 보내지게 하기 위한 임시 저장소 추가
        # self._last_status = None
        # self._last_current_task = None
        # self._last_detail = None
        # self._last_log = None

        self.stop_event = threading.Event()
        # 손 감지(is_emergency=False)와 구별되는 HMI emergency stop 전용 이벤트.
        # pour 루프에서 이 둘을 다르게 처리: 손 감지는 세우고 재개, emergency는 세우고 반납.
        self.emergency_event = threading.Event()
        cb_group = ReentrantCallbackGroup()

        self.status_pub = self.create_publisher(RobotStatus, "/robot/status", 10)

        self.move_client = self.create_client(
            MoveToPose, "/robot/move_to_pose", callback_group=cb_group
        )
        self.gripper_client = self.create_client(
            GripperControl, "/gripper/control", callback_group=cb_group
        )
        self.recheck_client = self.create_client(
            RequestRecheck, "/vision/request_recheck", callback_group=cb_group
        )
        #20260624 준형, CurrentTubeState 서비스 추가
        self.current_tube_state_client = self.create_client(
            CurrentTubeState, '/robot/current_tube_state', callback_group=cb_group
        )
        #end
        # 260624 jiwan vision 단독으로는 "안 보임"이 폐기인지 오검출인지 구분 못 해서,
        # 폐기 성공 시 로봇이 직접 알려주는 채널 추가
        self.mark_tube_disposed_client = self.create_client(
            MarkTubeDisposed, '/vision/mark_tube_disposed', callback_group=cb_group
        )
        # end
        # 트레이가 통째로 바뀔 때 vision이 새 트레이 기준으로 다시 부트스트랩하도록 함
        self.reset_slot_anchors_client = self.create_client(
            Trigger, '/vision/reset_slot_anchors', callback_group=cb_group
        )
        self.reset_handled_slots_client = self.create_client(
            Trigger, '/vision/reset_handled_slots', callback_group=cb_group
        )
        # 정지 요청이 들어오면 stop_event(체크포인트용)와 별개로, 지금 실제로
        # 움직이고 있는 모션을 doosan_robot_control_node 쪽에서 즉시 끊어달라고 요청한다.
        self.hard_stop_client = self.create_client(
            Trigger, '/robot/hard_stop', callback_group=cb_group
        )
        self.create_service(
            StartTask, "/robot/start_task", self.handle_start_task, callback_group=cb_group
        )
        self.create_service(
            StopTask, "/robot/stop_task", self.handle_stop_task, callback_group=cb_group
        )
        #20260626 JH, RobotStatus log 추가
        self.publish_status(RobotStatus.STATUS_IDLE, detail="robot_task_manager_node ready", log="robot_task_manager_node ready")
        self.get_logger().info("robot_task_manager_node ready")

    # ---- low-level helpers -------------------------------------------------

    #20260626 JH, RobotStatus publish에 log 추가(hmi 출력용)
    #20260626 JH, HMI에 publish할 때, 기존과 동일한 내용이면 전송하지 않는 로직 추가
    def publish_status(self, status, current_task="", detail="", log=""):
        #20260627 JH, HMI로 기능 이전
        # if (self._last_status == status and
        #     self._last_current_task == current_task and
        #     self._last_detail == detail and
        #     self._last_log == log):
        #     return  # 변경된 점이 없으면 여기서 함수를 종료(발행 안 함)

        # # 2. 변경점이 있다면 새로운 값으로 업데이트
        # self._last_status = status
        # self._last_current_task = current_task
        # self._last_detail = detail
        # self._last_log = log

        # 3. 메시지 생성 및 발행 (기존 로직)
        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = status
        msg.current_task = current_task
        msg.detail = detail
        msg.log = log
        
        self.status_pub.publish(msg)

    #20260624 준형, rclpy.spin_until_future_complete()형식으로 변환
    def _call_sync(self, client, request, timeout_sec=None, ignore_stop=False):
        if timeout_sec is None:
            timeout_sec = float(self.get_parameter("service_call_timeout_sec").value)

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service {client.srv_name} not available")
            return None

        future = client.call_async(request)
        done_event = threading.Event()
        future.add_done_callback(lambda _f: done_event.set())

        # 0.1초 단위로 polling해서 emergency stop이 걸리면 즉시 TaskAborted.
        # ignore_stop=True이면 stop_event를 무시하고 응답을 끝까지 기다림
        # (시약통 세우기 등 안전 복귀 동작에서 사용).
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if done_event.wait(timeout=0.1):
                break
            if not ignore_stop and self.stop_event.is_set():
                self.get_logger().warn(
                    f"Service {client.srv_name} aborted by emergency stop"
                )
                raise TaskAborted()
        else:
            self.get_logger().warn(f"Service {client.srv_name} timed out")
            return None

        try:
            return future.result()
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")
            return None
    #end

    # 정지 요청은 abort가 아니라 "그 자리에서 일시정지"임 - stop_event가 풀릴 때까지
    # 여기서 대기하다가, 풀리면 호출한 쪽의 바로 다음 줄부터 그대로 이어서 진행한다.
    # 단, 노드가 셧다운되는 중이면 영원히 블락되면 안 되니 그 경우만 진짜로 abort.
    def _check_stop(self):
        if not self.stop_event.is_set():
            return
        #20260626 JH, RobotStatus log 추가
        log_msg = ("Task paused - waiting for stop to clear")
        self.publish_status(RobotStatus.STATUS_EMERGENCY_STOP, detail="Paused - waiting to resume", log=log_msg)
        self.get_logger().warn(log_msg)
        while self.stop_event.is_set():
            if not rclpy.ok():
                raise TaskAborted()
            time.sleep(0.1)
        self.get_logger().warn("Task resumed")

    # 260625 준형, MoveToPose.srv에서 current_ratio 필드 제거에 맞춰 호출부도 정리
    def move(self, pose_name, move_type, status=None, ignore_stop=False, abort_on_stop=False):
        # 하드 스탑이 이 movel() 도중에 걸리면 로봇이 목표 지점에 도달하기 전에
        # 멈춰버릴 수 있음. stop_event가 다시 켜져 있으면(=이동 중 정지 요청이 왔던
        # 것) 재개를 기다렸다가 같은 목표로 다시 이동해서 실제로 도착했는지 보장한다.
        # ignore_stop=True  : stop_event를 무시하고 이동 완료 (안전 복귀 동작 전용).
        # abort_on_stop=True: stop 감지 시 TaskAborted를 호출부로 전달
        #                     (pour rotate처럼 호출부가 uprighting을 직접 처리할 때 사용).
        #                     False(기본값)이면 손 감지 정도의 stop은 여기서 흡수하여
        #                     손이 사라지면 자동으로 재시도 (grip/approach 등 일반 이동).
        #20260626 JH, RobotStatus log 추가
        if status is not None:
            self.publish_status(status[0], status[1], status[2], log=f"move to {pose_name}, move type : {move_type}")
        else:
            self.get_logger().info(f"move to {pose_name}, move type : {move_type}")
        while True:
            if not ignore_stop:
                self._check_stop()
            try:
                result = self._call_sync(
                    self.move_client,
                    MoveToPose.Request(pose_name=pose_name, move_type=move_type),
                    ignore_stop=ignore_stop,
                )
            except TaskAborted:
                # emergency stop이거나 호출부가 직접 처리하겠다고 한 경우만 전파.
                # 그 외(손 감지)는 여기서 흡수하여 손 사라지면 재시도.
                if self.emergency_event.is_set() or abort_on_stop:
                    raise
                self.get_logger().warn(f"Move to {pose_name} interrupted by hand; will retry once resumed")
                continue  # _check_stop()에서 대기 후 재시도
            if result is None or not result.success:
                raise TaskFailed(result.message if result else f"move to {pose_name} failed (timeout)")
            if ignore_stop or not self.stop_event.is_set():
                return
            self.get_logger().warn(f"Move to {pose_name} interrupted by stop; will retry once resumed")
    # end

    def grip(self, command, status=None, ignore_stop=False):
        if not ignore_stop:
            self._check_stop()
        if status is not None:
            if command == "OPEN":
                self.publish_status(RobotStatus.STATUS_GRIPPER_OPEN, status[1], status[2], 'release : gripper open')
            elif command == "CLOSE":
                self.publish_status(RobotStatus.STATUS_GRIPPER_CLOSE, status[1], status[2], 'grip : gripper close')
        else:
            if command == "OPEN":
                self.get_logger().info('release : gripper open')
            elif command == "CLOSE":
                self.get_logger().info('grip : gripper close')
        result = self._call_sync(self.gripper_client, GripperControl.Request(command=command), ignore_stop=ignore_stop)
        return result.success if result else False

    def grip_with_retry(self, command, status=None):
        self.grip_retry_count = int(self.get_parameter("grip_retry_count").value)
        for attempt in range(self.grip_retry_count + 1):
            if self.grip(command, status):
                return True
            self.get_logger().warn(f"Gripper {command} failed, attempt {attempt + 1}")
        return False

    def request_recheck(self):
        self.recheck_timeout_sec = float(self.get_parameter("recheck_timeout_sec").value)
        try:
            result = self._call_sync(
                self.recheck_client, RequestRecheck.Request(request=True), self.recheck_timeout_sec
            )
            self.get_logger().info(f"Recheck requested: {result.message}")
        except TaskFailed as e:
            self.get_logger().warn(f"Recheck request failed: {e}")

    # ---- task sequences -----------------------------------------------------

    def dispose_tube(self, idx):
        status = [RobotStatus.STATUS_MOVING, "dispose", f"approach tube {idx}"]
        self.move(dispose_tube_approach_pose(self.tray_idx, idx), 'move', status)
        self.move(dispose_tube_grip_pose(self.tray_idx, idx), 'down', status)
        
        if not self.grip_with_retry(GripperControl.Request.COMMAND_CLOSE, status):
            raise TaskFailed(f"Gripper failed to close on tube {idx}")

        status = [RobotStatus.STATUS_DISPOSING, "dispose", f"move tube {idx} to waste zone"]
        self.move(dispose_tube_approach_pose(self.tray_idx, idx), 'move', status)
        self.move(WASTE_APPROACH_POSE, 'move', status)
        self.move(WASTE_RELEASE_POSE, 'down', status)
        self.grip(GripperControl.Request.COMMAND_OPEN, status)

        status = [RobotStatus.STATUS_DISPOSING, "dispose", f"tube {idx} to pick rotate pose"]
        self.move(WASTE_APPROACH_POSE, 'move', status)
        self.move(WASTE_ROTATE_MIDDLE_POSE, 'move', status)
        self.move(WASTE_ROTATE_POSE, 'move', status)
        self.grip(GripperControl.Request.COMMAND_CLOSE, status)

        status = [RobotStatus.STATUS_DISPOSING, "dispose", f"tube {idx} to waste reagent"]
        self.move(WASTE_ROTATE_REAGENT_POSE, 'move', status)
        self.move(WASTE_ROTATE_REAGENT_POSE, 'rotate_reagent', status)
        
        status = [RobotStatus.STATUS_DISPOSING, "dispose", f"tube {idx} to waste tube"]
        self.move(WASTE_ROTATE_TUBE_POSE, 'move', status)
        self.grip(GripperControl.Request.COMMAND_OPEN, status)

        # 260624 jiwan 폐기 완료를 vision에 알림 (handled_slots override 트리거).
        # 여러 tube를 한 번에 폐기할 수 있어서 tube마다 즉시 알려줘야 함 - 끝나고
        # 한 번에 모아서 보내면 안 됨.
        result = self._call_sync(
            self.mark_tube_disposed_client, MarkTubeDisposed.Request(tube_index=idx)
        )
        if result is None or not result.success:
            self.get_logger().warn(f"Failed to mark tube {idx} as disposed in vision")
        # end

        #20260626 JH, 버리고 다시 위로 이동 추가
        status = [RobotStatus.STATUS_MOVING, "move to home", f"dispose tube {idx} complete"]
        self.move(WASTE_ROTATE_REAGENT_POSE, 'move', status)
        self.move(WASTE_ROTATE_POSE, 'move', status)
        self.move(WASTE_APPROACH_POSE, 'move', status)
        self.move(HOME_POSE, 'move', status)

    # 260624 jiwan 한 번에 계산해서 붓는 방식 -> 조금 붓고 vision 상태 확인해서
    # 모자르면 더 붓는 반복 루프로 변경. CurrentTubeState 서버가 이제 실제로
    # 구현되어 있어서(/robot/current_tube_state) 매 iteration 호출 가능.
    #20260626 JH, RobotStatus log 추가
    def refill_tube(self, idx):
        #refill 튜브 위치로 이동, 시약통 집기
        status = (RobotStatus.STATUS_MOVING, "refill", "approach refill zone")
        self.move(REFILL_SOURCE_APPROACH_POSE, 'move', status)
        self.move(REFILL_SOURCE_GRIP_POSE, 'down', status)
        if not self.grip_with_retry(GripperControl.Request.COMMAND_CLOSE, status):
            raise TaskFailed("Gripper failed to close on refill source")
        self.move(REFILL_SOURCE_APPROACH_POSE, 'move', status)

        #20260626 JH, TaskFailed 발생 시에도 시약통을 원래 위치로 가져다 놓도록 수정
        try:
            #채울 튜브 위치로 이동
            status = (RobotStatus.STATUS_MOVING, "refill", f"approach tube {idx}")
            self.move(refill_target_approach_pose(self.tray_idx, idx), 'move', status)
            #20260625 준형, refill_pose(45deg 기울인 위치)로 이동 추가
            self.move(refill_target_pour_pose(self.tray_idx, idx), 'down', status)

            max_pour_attempts = int(self.get_parameter("max_pour_attempts").value)
            normal_confirm_count = int(self.get_parameter("normal_confirm_count").value)
            self.publish_status(RobotStatus.STATUS_REFILLING, "refill", f"dispense reagent into tube {idx}")
            pour_status = (RobotStatus.STATUS_REFILLING, "refill", f"pour into tube {idx}")
            consecutive_normals = 0
            for attempt in range(max_pour_attempts):
                # stop이 걸려있으면 시약통을 먼저 세운 뒤 종류에 따라 처리.
                # _check_stop()을 먼저 호출하면 move 없이 바로 블락되므로,
                # ignore_stop=True로 down 자세 복귀를 먼저 실행.
                if self.stop_event.is_set():
                    self.get_logger().warn(f"Pour stopped (tube {idx}): uprighting bottle")
                    self.move(refill_target_pour_pose(self.tray_idx, idx), 'down', pour_status, ignore_stop=True)
                    if self.emergency_event.is_set():
                        # HMI emergency stop: 시약통 반납 후 완전 종료 (finally에서 처리)
                        raise TaskAborted()
                    # 손 감지: 손이 사라질 때까지 대기 후 pour 재개
                    self._check_stop()
                    consecutive_normals = 0  # 재개 후 카운터 초기화
                    continue  # state 재확인부터

                result = self._call_sync(self.current_tube_state_client, CurrentTubeState.Request(tube_index=idx))
                if result is None or not result.success:
                    raise TaskFailed(f"current_tube_state unavailable for tube {idx}")
                if result.state == TubeState.STATE_NORMAL:
                    consecutive_normals += 1
                    self.get_logger().info(
                        f"Tube {idx} NORMAL ({consecutive_normals}/{normal_confirm_count})"
                    )
                    if consecutive_normals >= normal_confirm_count:
                        break  # 연속 N회 NORMAL 확인 → refill 완료
                    # 아직 확인 중: 붓지 않고 다음 체크 대기
                    time.sleep(0.3)
                    continue
                consecutive_normals = 0  # NORMAL이 아니면 카운터 리셋
                if result.state == TubeState.STATE_DISPOSE_NEEDED:
                    raise TaskFailed(f"Tube {idx} overflowed during refill")

                try:
                    self.move(refill_target_pour_pose(self.tray_idx, idx), 'rotate', pour_status, abort_on_stop=True)
                except TaskAborted:
                    # rotate 진행 중 stop 감지 시 abort_on_stop=True로 TaskAborted가 여기까지 전달됨.
                    # 시약통이 기울어진 상태일 수 있으므로 즉시 세움.
                    self.get_logger().warn(f"Stop during rotate (tube {idx}): uprighting immediately")
                    self.move(refill_target_pour_pose(self.tray_idx, idx), 'down', pour_status, ignore_stop=True)
                    if self.emergency_event.is_set():
                        raise TaskAborted()  # emergency → finally에서 반납
                    self._check_stop()       # 손 감지 → 손 사라질 때까지 대기
                    continue                 # 재개 후 state 재확인부터

                # rotate 완료 직후 stop 감지 시 즉시 세우기 (기울어진 채 대기 방지)
                if self.stop_event.is_set():
                    self.get_logger().warn(f"Stop after rotate (tube {idx}): uprighting immediately")
                    self.move(refill_target_pour_pose(self.tray_idx, idx), 'down', pour_status, ignore_stop=True)
                    if self.emergency_event.is_set():
                        raise TaskAborted()
                    self._check_stop()
                    continue

                time.sleep(0.3)
            else:
                raise TaskFailed(f"Tube {idx} still not normal after {max_pour_attempts} pour attempts")

            #시약통 반납
            #20260625 준형, 회전 후 회전 전 최초 위치로 복귀 후 approach_pose로 이동
            # status = [RobotStatus.STATUS_MOVING, "refill", "move to refill zone"]
            # self.move(refill_target_pour_pose(self.tray_idx, idx), 'down', status)
            # self.move(refill_target_approach_pose(self.tray_idx, idx), 'move', status)

        finally:
            # TaskFailed / TaskAborted 어느 경우든 시약통을 반납하고 홈으로 복귀.
            # ignore_stop=True는 hard_stop에 의해 목표 미도달 상태에서도 다음 단계로
            # 진행해버리는 버그가 있어서 사용 금지.
            # 대신 _do_cleanup으로 TaskAborted를 잡아 stop 해제 후 재시도하여
            # 각 단계마다 실제로 목표 위치에 도달한 뒤에만 다음 단계를 실행한다.
            # 260626 jiwan 
            status = [RobotStatus.STATUS_MOVING, "refill", "move to refill zone"]
            self.move(refill_target_pour_pose(self.tray_idx, idx), 'down', status)
            self.move(refill_target_approach_pose(self.tray_idx, idx), 'move', status)
            #  end
            def _do_cleanup(fn):
                while True:
                    try:
                        fn()
                        return
                    except TaskAborted:
                        # stop이 해제될 때까지 대기 후 재시도
                        while self.stop_event.is_set():
                            if not rclpy.ok():
                                return
                            time.sleep(0.2)
            try:
                status_cleanup = [RobotStatus.STATUS_MOVING, "refill", "return refill source"]
                _do_cleanup(lambda: self.move(REFILL_SOURCE_APPROACH_POSE, 'move', status_cleanup))
                _do_cleanup(lambda: self.move(REFILL_SOURCE_GRIP_POSE, 'down', status_cleanup))
                _do_cleanup(lambda: self.grip(GripperControl.Request.COMMAND_OPEN))
                status_cleanup = [RobotStatus.STATUS_MOVING, "move to home", f"refill tube {idx} complete"]
                _do_cleanup(lambda: self.move(REFILL_SOURCE_APPROACH_POSE, 'move', status_cleanup))
                _do_cleanup(lambda: self.move(HOME_POSE, 'move', status_cleanup))
            except Exception as e:
                self.get_logger().warn(f"Failed to cleanup refill source after failure: {e}")
    # end

    def transfer_tray(self):
        status = [RobotStatus.STATUS_MOVING, "transfer_normal", "pick up tray tool"]
        self.move(TRAY_TOOL_STAND_APPROACH_POSE, 'move', status)
        self.move(TRAY_TOOL_STAND_GRIP_POSE, 'down', status)

        if not self.grip_with_retry(GripperControl.Request.COMMAND_CLOSE, status):
            raise TaskFailed("Gripper failed to grip tray tool")
        self.move(TRAY_TOOL_STAND_APPROACH_POSE, 'move', status)

        status = [RobotStatus.STATUS_TRANSFER_TRAY, "transfer_tray", f"transfer tray {self.tray_idx}"]
        self.move(tray_transfer_tool_approach_pose(self.tray_idx), 'move', status)
        self.move(tray_transfer_tool_preinsert_pose(self.tray_idx), 'down_tray', status)
        self.move(tray_transfer_tool_insert_pose(self.tray_idx), 'down_tray', status)
        self.move(tray_transfer_lift_pose(self.tray_idx), 'move', status)

        self.move(tray_transfer_success_approach_pose(self.tray_idx), 'move', status)
        self.move(tray_transfer_success_place_pose(self.tray_idx), 'down_tray', status)
        self.move(tray_transfer_tool_detach_pose(self.tray_idx), 'move', status)

        self.move(TRAY_TOOL_STAND_APPROACH_POSE, 'move', status)
        self.move(TRAY_TOOL_STAND_GRIP_POSE, 'down', status)
        self.grip(GripperControl.Request.COMMAND_OPEN, status)
        status = [RobotStatus.STATUS_MOVING, "move to home", f"transfer tray {self.tray_idx} complete"]
        self.move(TRAY_TOOL_STAND_APPROACH_POSE, 'move', status)
        self.move(HOME_POSE, 'move', status)

        self._advance_tray()

    # 트레이 1개 처리 완료 후 다음 줄로 넘어가면서, vision이 새 트레이 기준으로
    # 다시 부트스트랩하도록 anchor/handled_slots를 리셋한다.
    def _advance_tray(self):
        num_trays = int(self.get_parameter("num_trays").value)
        if self.tray_idx >= num_trays - 1:
            self.get_logger().warn("All trays already transferred; staying on the last tray_idx")
            return

        self.tray_idx += 1
        for client, request in (
            (self.reset_slot_anchors_client, Trigger.Request()),
            (self.reset_handled_slots_client, Trigger.Request()),
        ):
            result = self._call_sync(client, request)
            if result is None or not result.success:
                self.get_logger().warn(f"{client.srv_name} failed while advancing to tray {self.tray_idx}")
        self.get_logger().info(f"Advanced to tray_idx={self.tray_idx}")

    # ---- service handlers ---------------------------------------------------

    def handle_start_task(self, request, response):
        # stop_event는 그리퍼 open보다 먼저 clear해야 함 - 이전 emergency stop이
        # 남겨놓은 stop_event가 그대로면 grip()의 _check_stop()이 TaskAborted를
        # 던지는데, try 블록 밖이면 아무도 못 잡고 노드가 죽어버림.
        self.stop_event.clear()
        state_map = dict(zip(request.tube_index, request.state))

        dispose_idxs = sorted(i for i, s in state_map.items() if s == TubeState.STATE_DISPOSE_NEEDED)
        refill_idxs = sorted(i for i, s in state_map.items() if s == TubeState.STATE_REFILL_NEEDED)
        all_normal = bool(state_map) and all(s == TubeState.STATE_NORMAL for s in state_map.values())

        try:
            #20260625 준형, start_task(시스템 시작)시 그리퍼 open 추가
            self.grip(GripperControl.Request.COMMAND_OPEN)
            if dispose_idxs:
                for idx in dispose_idxs:
                    self.dispose_tube(idx)
                self.request_recheck()
                response.success = True
                response.message = f"Disposed tube(s) {dispose_idxs}; recheck requested"
            elif refill_idxs:
                for idx in refill_idxs:
                    self.refill_tube(idx)
                self.request_recheck()
                response.success = True
                response.message = f"Refilled tube(s) {refill_idxs}; recheck requested"
            elif all_normal:
                self.transfer_tray()
                response.success = True
                response.message = "Tray transferred to normal zone"
            else:
                response.success = True
                response.message = "No action required"

            self.publish_status(RobotStatus.STATUS_IDLE, detail=response.message)

        except TaskAborted:
            response.success = False
            response.message = "Task aborted by emergency stop"
            self.publish_status(RobotStatus.STATUS_EMERGENCY_STOP, detail=response.message)
        except TaskFailed as e:
            response.success = False
            response.message = str(e)
            self.publish_status(RobotStatus.STATUS_ERROR, detail=response.message)

        return response

    def handle_stop_task(self, request, response):
        if request.stop:
            self.stop_event.set()
            if request.is_emergency:
                self.emergency_event.set()
            # 체크포인트(다음 move/grip 호출 전)를 기다리지 않고, 지금 실제로 진행 중인
            # movel()이 있으면 doosan_robot_control_node가 하드웨어 레벨로 즉시 멈추게 함.
            # 응답을 기다릴 필요는 없어서 fire-and-forget.
            if self.hard_stop_client.service_is_ready():
                self.hard_stop_client.call_async(Trigger.Request())
            else:
                self.get_logger().warn("/robot/hard_stop not available; falling back to checkpoint-only stop")
            response.success = True
            kind = "Emergency stop" if request.is_emergency else "Hand-detected stop"
            response.message = f"{kind} requested; current motion will halt immediately"
            self.get_logger().warn(response.message)
        else:
            self.stop_event.clear()
            self.emergency_event.clear()
            response.success = True
            response.message = "Resume requested; any paused task will continue"
            self.get_logger().warn("Resume requested")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = RobotTaskManagerNode()
    executor = MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
