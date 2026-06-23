import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("robot_control_pkg"), "config", "robot_params.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="robot_control_pkg",
                executable="doosan_robot_control_node",
                name="doosan_robot_control_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="robot_control_pkg",
                executable="gripper_control_node",
                name="gripper_control_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="robot_control_pkg",
                executable="robot_task_manager_node",
                name="robot_task_manager_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="robot_control_pkg",
                executable="main_decision_node",
                name="main_decision_node",
                parameters=[config],
                output="screen",
            ),
        ]
    )
