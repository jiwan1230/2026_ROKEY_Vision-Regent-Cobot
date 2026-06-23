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

import rclpy
from rclpy.node import Node

from interfaces.srv import MoveToPose
from robot_control_pkg.poses import all_pose_names


class DoosanRobotControlNode(Node):
    def __init__(self):
        super().__init__("doosan_robot_control_node")

        self.declare_parameter("velocity", 60.0)
        self.declare_parameter("acceleration", 60.0)
        self.declare_parameter("move_duration_sec", 0.4)

        self.move_duration_sec = float(self.get_parameter("move_duration_sec").value)

        self.poses = {}
        for name in all_pose_names(num_tubes=3):
            param_name = f"poses.{name}"
            self.declare_parameter(param_name, [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
            self.poses[name] = list(self.get_parameter(param_name).value)

        self.current_pose_name = "home_pose"

        self.create_service(MoveToPose, "/robot/move_to_pose", self.handle_move_to_pose)
        self.get_logger().info(f"doosan_robot_control_node ready ({len(self.poses)} poses loaded)")

    def _simulate_move(self, pose_name, target):
        self.get_logger().info(f"MOVE -> {pose_name} {target}")
        time.sleep(self.move_duration_sec)

    def handle_move_to_pose(self, request, response):
        pose_name = request.pose_name
        target = self.poses.get(pose_name)
        if target is None:
            response.success = False
            response.message = f"Unknown pose_name: {pose_name}"
            self.get_logger().error(response.message)
            return response

        self._simulate_move(pose_name, target)
        self.current_pose_name = pose_name

        response.success = True
        response.message = f"Moved to {pose_name}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DoosanRobotControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
