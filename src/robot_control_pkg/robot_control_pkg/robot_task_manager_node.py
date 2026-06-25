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
    NORMAL_TRAY_APPROACH_POSE,
    NORMAL_TRAY_POSE,
    TRAY_PICK_POSE,
    WASTE_POSE,
    WASTE_DOWN_POSE,
    REFILL_POSE,
    tube_approach_pose,
    tube_pick_pose,
    tube_refill_pose,
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
        # end
        #20260624 준형, grip_retry_count, recheck_timeout_sec, service_call_timeout_sec를 사용할 때 get_parameter()로 가져오도록 수정
        # self.grip_retry_count = int(self.get_parameter("grip_retry_count").value)
        # self.recheck_timeout_sec = float(self.get_parameter("recheck_timeout_sec").value)
        # self.service_call_timeout_sec = float(self.get_parameter("service_call_timeout_sec").value)
        #end

        self.stop_event = threading.Event()
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
        self.create_service(
            StartTask, "/robot/start_task", self.handle_start_task, callback_group=cb_group
        )
        self.create_service(
            StopTask, "/robot/stop_task", self.handle_stop_task, callback_group=cb_group
        )

        self.publish_status(RobotStatus.STATUS_IDLE, detail="robot_task_manager_node ready")
        self.get_logger().info("robot_task_manager_node ready")

    # ---- low-level helpers -------------------------------------------------

    def publish_status(self, status, current_task="", detail=""):
        msg = RobotStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = status
        msg.current_task = current_task
        msg.detail = detail
        self.status_pub.publish(msg)

    #20260624 준형, rclpy.spin_until_future_complete()형식으로 변환
    def _call_sync(self, client, request, timeout_sec=None):
        if timeout_sec is None:
            timeout_sec = float(self.get_parameter("service_call_timeout_sec").value)

        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Service {client.srv_name} not available")
            return None

        future = client.call_async(request)

        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if future.done():
            try:
                return future.result()
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return None
        else:
            self.get_logger().warn(f"Service {client.srv_name} timed out")
            return None
    #end

    def _check_stop(self):
        if self.stop_event.is_set():
            raise TaskAborted()

    # 260624 jiwan current_ratio 파라미터 추가 - 'rotate' move_type에서 MoveToPose로
    # 안 넘어가고 있던 버그(refill_tube가 3개 인자로 호출해서 TypeError 났었음) 수정
    def move(self, pose_name, move_type, current_ratio=0.0):
        self._check_stop()
        result = self._call_sync(
            self.move_client,
            MoveToPose.Request(pose_name=pose_name, move_type=move_type, current_ratio=current_ratio),
        )
        if not result.success:
            raise TaskFailed(result.message)
    # end

    def grip(self, command):
        result = self._call_sync(self.gripper_client, GripperControl.Request(command=command))
        return result.success

    def grip_with_retry(self, command):
        self.grip_retry_count = int(self.get_parameter("grip_retry_count").value)
        for attempt in range(self.grip_retry_count + 1):
            self._check_stop()
            if self.grip(command):
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
        self.publish_status(RobotStatus.STATUS_MOVING, "dispose", f"approach tube {idx}")
        self.move(tube_approach_pose(idx), 'move')
        self.move(tube_pick_pose(idx), 'down')

        self.publish_status(RobotStatus.STATUS_PICKING, "dispose", f"grip tube {idx}")
        if not self.grip_with_retry(GripperControl.Request.COMMAND_CLOSE):
            raise TaskFailed(f"Gripper failed to close on tube {idx}")

        self.move(tube_approach_pose(idx), 'move')
        self.publish_status(RobotStatus.STATUS_DISPOSING, "dispose", f"move tube {idx} to waste zone")
        self.move(WASTE_POSE, 'move')
        self.move(WASTE_DOWN_POSE, 'down')
        self.grip(GripperControl.Request.COMMAND_OPEN)

        # 260624 jiwan 폐기 완료를 vision에 알림 (handled_slots override 트리거).
        # 여러 tube를 한 번에 폐기할 수 있어서 tube마다 즉시 알려줘야 함 - 끝나고
        # 한 번에 모아서 보내면 안 됨.
        result = self._call_sync(
            self.mark_tube_disposed_client, MarkTubeDisposed.Request(tube_index=idx)
        )
        if result is None or not result.success:
            self.get_logger().warn(f"Failed to mark tube {idx} as disposed in vision")
        # end

        self.move(HOME_POSE, 'move')

    # 260624 jiwan 한 번에 계산해서 붓는 방식 -> 조금 붓고 vision 상태 확인해서
    # 모자르면 더 붓는 반복 루프로 변경. CurrentTubeState 서버가 이제 실제로
    # 구현되어 있어서(/robot/current_tube_state) 매 iteration 호출 가능.
    def refill_tube(self, idx):
        #refill 튜브 위치로 이동, 시약통 집기
        self.publish_status(RobotStatus.STATUS_MOVING, "refill", "approach refill zone")
        self.move(REFILL_POSE, 'move')
        self.move(REFILL_POSE, 'down')
        self.grip(GripperControl.Request.COMMAND_CLOSE)
        self.move(REFILL_POSE, 'move')

        #채울 튜브 위치로 이동
        self.publish_status(RobotStatus.STATUS_MOVING, "refill", f"approach tube {idx}")
        self.move(tube_approach_pose(idx), 'move')
        #20260625 준형, refill_pose(45deg 기울인 위치)로 이동 추가
        self.move(tube_refill_pose(idx), 'down')

        max_pour_attempts = int(self.get_parameter("max_pour_attempts").value)
        self.publish_status(RobotStatus.STATUS_REFILLING, "refill", f"dispense reagent into tube {idx}")
        for attempt in range(max_pour_attempts):
            self._check_stop()
            result = self._call_sync(self.current_tube_state_client, CurrentTubeState.Request(tube_index=idx))
            if result is None or not result.success:
                raise TaskFailed(f"current_tube_state unavailable for tube {idx}")
            if result.state == TubeState.STATE_NORMAL:
                break
            if result.state == TubeState.STATE_DISPOSE_NEEDED:
                raise TaskFailed(f"Tube {idx} overflowed during refill")

            self.move(tube_refill_pose(idx), 'rotate')
            self._check_stop()
            time.sleep(0.3)  # simulated dispense duration for one pour increment
        else:
            raise TaskFailed(f"Tube {idx} still not normal after {max_pour_attempts} pour attempts")

        #시약통 반납
        #20260625 준형, 회전 후 회전 전 최초 위치로 복귀 후 approach_pose로 이동
        self.move(tube_refill_pose(idx), 'down')
        self.move(tube_approach_pose(idx), 'move')
        self.publish_status(RobotStatus.STATUS_MOVING, "refill", "approach refill zone")
        self.move(REFILL_POSE, 'move')
        self.move(REFILL_POSE, 'down')
        self.grip(GripperControl.Request.COMMAND_OPEN)
        self.move(REFILL_POSE, 'move')

        self.move(HOME_POSE, 'move')
    # end

    def transfer_tray(self):
        self.publish_status(RobotStatus.STATUS_MOVING, "transfer_normal", "approach tray")
        self.move(TRAY_PICK_POSE, 'move')

        self.publish_status(RobotStatus.STATUS_PICKING, "transfer_normal", "grip tray")
        if not self.grip_with_retry(GripperControl.Request.COMMAND_CLOSE):
            raise TaskFailed("Gripper failed to grip tray")

        self.publish_status(
            RobotStatus.STATUS_TRANSFER_NORMAL, "transfer_normal", "move tray to normal zone"
        )
        self.move(NORMAL_TRAY_APPROACH_POSE, 'down_tray')
        self.move(NORMAL_TRAY_POSE, 'move')
        self.grip(GripperControl.Request.COMMAND_OPEN)
        self.move(HOME_POSE, 'move')

    # ---- service handlers ---------------------------------------------------

    def handle_start_task(self, request, response):
        #20260625 준형, start_task(시스템 시작)시 그리퍼 open 추가
        self.grip(GripperControl.Request.COMMAND_OPEN)
        self.stop_event.clear()
        state_map = dict(zip(request.tube_index, request.state))

        dispose_idxs = sorted(i for i, s in state_map.items() if s == TubeState.STATE_DISPOSE_NEEDED)
        refill_idxs = sorted(i for i, s in state_map.items() if s == TubeState.STATE_REFILL_NEEDED)
        all_normal = bool(state_map) and all(s == TubeState.STATE_NORMAL for s in state_map.values())

        try:
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
            response.success = True
            response.message = "Stop requested; current task will abort at the next safe checkpoint"
            self.get_logger().warn("Emergency stop requested")
        else:
            response.success = True
            response.message = "No-op (stop=false)"
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
