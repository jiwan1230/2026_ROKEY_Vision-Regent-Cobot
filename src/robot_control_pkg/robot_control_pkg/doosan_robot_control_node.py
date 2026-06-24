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
from interfaces.srv import MoveToPose
from robot_control_pkg.poses import all_pose_names

if not rclpy.ok():
    rclpy.init(args=sys.argv)

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"

dsr_node = rclpy.create_node('dsr_lib_node_move', namespace=ROBOT_ID)

import DR_init
DR_init.__dsr__node = dsr_node
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

from DSR_ROBOT2 import movel, move_periodic

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

        self.create_service(MoveToPose, "/robot/move_to_pose", self.handle_move_to_pose)
        self.get_logger().info(f"doosan_robot_control_node ready ({len(self.poses)} poses loaded)")

    #20260624 준형, move_type에 따라 이동방식 다르게 적용
    def move(self, pose_name, target, move_type, current_ratio):
        self.get_logger().info(f"MOVE -> {pose_name} {target} {move_type}")
        if move_type == 'move':
            movel(target, vel=self.get_parameter("m_velocity").value, acc=self.get_parameter("m_acceleration").value)
        elif move_type == 'down':
            movel(target, vel=self.get_parameter("d_velocity").value, acc=self.get_parameter("d_acceleration").value)
        elif move_type == 'down_tray':
            movel(target, vel=self.get_parameter("d_velocity").value, acc=self.get_parameter("d_acceleration").value)
        elif move_type == 'rotate':            
            rotate_deg = (1 - current_ratio) / 0.01 * 1
            target_copy = target.copy()
            target_copy[3] = 90
            target_copy[4] += rotate_deg
            target_copy[5] = -90
            movel(target_copy, vel=self.get_parameter("d_velocity").value, acc=self.get_parameter("d_acceleration").value)
        time.sleep(self.move_duration_sec)
    #end
    
    #20260624 준형, pose 실시간 업데이트 적용
    def handle_move_to_pose(self, request, response):
        pose_name = request.pose_name
        param_name = f"poses.{pose_name}"
        if not self.has_parameter(param_name):
            response.success = False
            response.message = f"Unknown pose_name: {pose_name}"
            self.get_logger().error(response.message)
            return response

        target = self.get_parameter(param_name).value

        self.get_logger().info(f'{target} to move')
        self.move(pose_name, target, request.move_type, request.current_ratio)
        self.current_pose_name = pose_name

        response.success = True
        response.message = f"Moved to {pose_name}"
        return response
#end

def main(args=None):
    control_node = DoosanRobotControlNode()

    executor = MultiThreadedExecutor()
    executor.add_node(control_node)
    executor.add_node(dsr_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        control_node.destroy_node()
        dsr_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
