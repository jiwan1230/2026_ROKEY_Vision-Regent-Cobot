import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("vision_pkg"), "config", "vision_params.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="vision_pkg",
                executable="side_camera_node",
                name="side_camera_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="vision_pkg",
                executable="liquid_height_detector_node",
                name="liquid_height_detector_node",
                parameters=[config],
                output="screen",
            ),
            Node(
                package="vision_pkg",
                executable="tube_state_publisher_node",
                name="tube_state_publisher_node",
                parameters=[config],
                output="screen",
            ),
        ]
    )
