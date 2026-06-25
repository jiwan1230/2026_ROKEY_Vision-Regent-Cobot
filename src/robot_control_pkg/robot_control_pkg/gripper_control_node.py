"""Mock gripper node, mirroring the open_gripper()/close_gripper()/get_status()
interface of the OnRobot RG gripper wrapper used elsewhere in this team's
Doosan bootcamp code (pick_and_place_text/onrobot.py). Swap _simulate_grip()
for real RG calls when wiring up hardware.
"""
import time
import sys
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from interfaces.srv import GripperControl

if not rclpy.ok():
    rclpy.init(args=sys.argv)

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

dsr_node = rclpy.create_node('dsr_lib_node', namespace=ROBOT_ID)

import DR_init
DR_init.__dsr__node = dsr_node
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from DSR_ROBOT2 import set_digital_output, ON, OFF, get_digital_input

class GripperControlNode(Node):
    def __init__(self):
        super().__init__("gripper_control_node")

        #20260624 준형, move_duration_sec 사용하지 않음으로 주석처리
        # self.declare_parameter("move_duration_sec", 0.3)
        # self.move_duration_sec = float(self.get_parameter("move_duration_sec").value)
        #end
        self.gripper_state = "OPEN"

        # HMI의 "Robot Control Log" 탭용 - 터미널에만 찍히던 동작을 사람이 읽을
        # 문장으로 같이 발행한다. get_logger() 호출은 그대로 두고 추가만 함.
        self.robot_log_pub = self.create_publisher(String, "/robot/log", 10)

        self.create_service(GripperControl, "/gripper/control", self.handle_gripper_control)
        self.get_logger().info("gripper_control_node ready")

    def _log_event(self, message, level="info"):
        getattr(self.get_logger(), level)(message)
        self.robot_log_pub.publish(String(data=message))

    def handle_gripper_control(self, request, response):
        command = request.command

        if command == GripperControl.Request.COMMAND_OPEN:
            self.open_gripper()
        elif command == GripperControl.Request.COMMAND_CLOSE:
            self.close_gripper()
        else:
            response.success = False
            response.message = f"Unsupported gripper command: {command}"
            self._log_event(response.message, level="error")
            return response
        self.gripper_state = command

        response.success = True
        response.message = f"Gripper {command}"
        return response

    def wait_digital_input(self, port, state, timeout_sec):
        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            if get_digital_input(port) == state:
                return True
            time.sleep(0.01)
        return False

    def open_gripper(self):
        self._log_event("그리퍼 열기")
        set_digital_output(1, OFF)
        set_digital_output(2, ON)
        self.wait_digital_input(2, ON, 5.0)
        return

    def close_gripper(self):
        self._log_event("그리퍼 닫기")
        set_digital_output(1, ON)
        set_digital_output(2, OFF)
        self.wait_digital_input(1, ON, 5.0)
        return

def main(args=None):
    gripper_node = GripperControlNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(gripper_node)
    executor.add_node(dsr_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        gripper_node.destroy_node()
        dsr_node.destroy_node() 
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
