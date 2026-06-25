"""Mock Doosan M0609 arm motion node.

Hosts /robot/move_to_pose. Each named pose is an [x, y, z, rx, ry, rz]
posx-style coordinate loaded from config/robot_params.yaml (spec doc
section 13's coordinate map). The actual motion is simulated with a sleep
proportional to move_duration_sec, standing in for movej/movel + mwait().

To connect to the real robot, replace _simulate_move() with calls into
doosan_robot2 (DSR_ROBOT2.movel / mwait), keeping pose lookup and the
service interface unchanged.
"""
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from std_srvs.srv import Trigger
from dsr_msgs2.srv import MoveStop
from interfaces.srv import MoveToPose
from robot_control_pkg.poses import all_pose_names

if not rclpy.ok():
    rclpy.init(args=sys.argv)

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

dsr_node = rclpy.create_node('dsr_lib_node_move', namespace=ROBOT_ID)
# movel()이 내부적으로 dsr_node를 spin_until_future_complete로 블락하는 동안에도
# 정지 요청이 끼어들 수 있어야 해서, MoveStop 클라이언트는 별도 노드에 둔다 - 같은
# 노드를 같이 쓰면 movel()이 막혀있는 동안 이 요청도 같이 막혀버림.
dsr_stop_node = rclpy.create_node('dsr_lib_node_stop', namespace=ROBOT_ID)

import DR_init
DR_init.__dsr__node = dsr_node
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
# 260625 준형님 코드로 부분 수정
# from DSR_ROBOT2 import movel, move_periodic
from DSR_ROBOT2 import movel, move_periodic, get_current_posx, DR_BASE, DR_HOLD
# END
class DoosanRobotControlNode(Node):
    def __init__(self):
        super().__init__("doosan_robot_control_node")
        
        #나중에 HMI에서 받아오게 바꿔야 함
        self.declare_parameter("m_velocity", 60.0)
        self.declare_parameter("m_acceleration", 60.0)
        self.declare_parameter("d_velocity", 30.0)
        self.declare_parameter("d_acceleration", 30.0)
        self.declare_parameter("move_duration_sec", 0.4)

        self.move_duration_sec = float(self.get_parameter("move_duration_sec").value)

        self.poses = {}
        for name in all_pose_names(num_tubes=3):
            param_name = f"poses.{name}"
            self.declare_parameter(param_name, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.poses[name] = list(self.get_parameter(param_name).value)

        self.current_pose_name = "home_pose"

        self.move_stop_client = dsr_stop_node.create_client(MoveStop, "motion/move_stop")

        # HMI의 "Robot Control Log" 탭용 - 터미널에만 찍히던 동작을 사람이 읽을
        # 문장으로 같이 발행한다. get_logger() 호출은 그대로 두고 추가만 함.
        self.robot_log_pub = self.create_publisher(String, "/robot/log", 10)

        self.create_service(MoveToPose, "/robot/move_to_pose", self.handle_move_to_pose)
        # move_to_pose와 다른 callback group을 써야 함 - 기본(상호배제) 그룹을 같이 쓰면
        # movel()이 진행 중인 동안 이 서비스 콜백도 같은 그룹에서 큐에 걸려 대기하느라
        # 정지 요청이 모션이 끝날 때까지 전달이 안 됨.
        self.create_service(
            Trigger, "/robot/hard_stop", self.handle_hard_stop, callback_group=ReentrantCallbackGroup()
        )
        self.get_logger().info(f"doosan_robot_control_node ready ({len(self.poses)} poses loaded)")

    def _log_event(self, message, level="info"):
        getattr(self.get_logger(), level)(message)
        self.robot_log_pub.publish(String(data=message))

    #20260624 준형, move_type에 따라 이동방식 다르게 적용
    #20260625 준형, rotate 로직 변경
    def move(self, pose_name, target, move_type):
        self._log_event(f"MOVE -> {pose_name} {target} {move_type}")
        if move_type == 'move':
            movel(target, vel=self.get_parameter("m_velocity").value, acc=self.get_parameter("m_acceleration").value)
        elif move_type == 'down':
            movel(target, vel=self.get_parameter("d_velocity").value, acc=self.get_parameter("d_acceleration").value)
        elif move_type == 'down_tray':
            movel(target, vel=self.get_parameter("d_velocity").value, acc=self.get_parameter("d_acceleration").value)
        elif move_type == 'rotate':
            rotate_pos = get_current_posx(DR_BASE)[0]
            rotate_pos[2] += -1.7
            rotate_pos[4] += -2
            rotate_pos[3] = 90
            rotate_pos[5] = -90
            movel(rotate_pos, vel=[self.get_parameter("d_velocity").value, 5], acc=[self.get_parameter("d_acceleration").value, 5])
            move_periodic([0, 0, 0, 0, 0, 3], period=0.5, repeat=3)
        time.sleep(self.move_duration_sec)
    #end
    
    #20260624 준형, pose 실시간 업데이트 적용
    #20260625 준형, ratio 더 이상 사용하지 않으므로 해당 내용 삭제
    def handle_move_to_pose(self, request, response):
        pose_name = request.pose_name
        param_name = f"poses.{pose_name}"
        if not self.has_parameter(param_name):
            response.success = False
            response.message = f"Unknown pose_name: {pose_name}"
            self._log_event(response.message, level="error")
            return response

        target = self.get_parameter(param_name).value

        self.get_logger().info(f'{target} to move')
        self.move(pose_name, target, request.move_type)
        self.current_pose_name = pose_name

        response.success = True
        response.message = f"Moved to {pose_name}"
        return response

    # 지금 진행 중인 movel()을 하드웨어 레벨에서 즉시 정지시킨다 (DR_HOLD: 다시
    # movel()을 보내면 그대로 이어갈 수 있는 정지 모드 - STO/QSTOP처럼 안전 정지
    # 상태로 빠지지 않음). 응답을 기다릴 필요 없는 fire-and-forget이라 비동기로만 호출.
    def handle_hard_stop(self, request, response):
        if not self.move_stop_client.service_is_ready():
            response.success = False
            response.message = "motion/move_stop service not available"
            self._log_event(response.message, level="error")
            return response

        future = self.move_stop_client.call_async(MoveStop.Request(stop_mode=DR_HOLD))
        future.add_done_callback(self._log_move_stop_result)
        response.success = True
        response.message = "Hard stop requested"
        self._log_event(response.message, level="warn")
        return response

    def _log_move_stop_result(self, future):
        try:
            result = future.result()
            self._log_event(f"motion/move_stop -> success={result.success}", level="warn")
        except Exception as e:
            self._log_event(f"motion/move_stop call failed: {e}", level="error")
#end

def main():
    control_node = DoosanRobotControlNode()

    executor = MultiThreadedExecutor()
    executor.add_node(control_node)
    executor.add_node(dsr_node)
    executor.add_node(dsr_stop_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        control_node.destroy_node()
        dsr_node.destroy_node()
        dsr_stop_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
