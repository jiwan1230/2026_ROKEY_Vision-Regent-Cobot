"""Mock gripper node, mirroring the open_gripper()/close_gripper()/get_status()
interface of the OnRobot RG gripper wrapper used elsewhere in this team's
Doosan bootcamp code (pick_and_place_text/onrobot.py). Swap _simulate_grip()
for real RG calls when wiring up hardware.
"""
import time

import rclpy
from rclpy.node import Node

from interfaces.srv import GripperControl


class GripperControlNode(Node):
    def __init__(self):
        super().__init__("gripper_control_node")

        self.declare_parameter("move_duration_sec", 0.3)
        self.move_duration_sec = float(self.get_parameter("move_duration_sec").value)
        self.gripper_state = "OPEN"

        self.create_service(GripperControl, "/gripper/control", self.handle_gripper_control)
        self.get_logger().info("gripper_control_node ready")

    def _simulate_grip(self, command):
        self.get_logger().info(f"GRIPPER -> {command}")
        time.sleep(self.move_duration_sec)

    def handle_gripper_control(self, request, response):
        command = request.command
        if command not in (GripperControl.Request.COMMAND_OPEN, GripperControl.Request.COMMAND_CLOSE):
            response.success = False
            response.message = f"Unknown gripper command: {command}"
            self.get_logger().error(response.message)
            return response

        self._simulate_grip(command)
        self.gripper_state = command

        response.success = True
        response.message = f"Gripper {command}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripperControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
