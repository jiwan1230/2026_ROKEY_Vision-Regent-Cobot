from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # vision_pkg's own nodes (side_camera_node, liquid_height_detector_node,
    # tube_state_publisher_node) are launched by vision_launch.py - this file
    # only owns the HMI dashboard, so including both this and vision_launch.py
    # from system_launch.py doesn't double-start the same node names.
    return LaunchDescription(
        [
            Node(
                package="hmi_pkg",
                executable="hmi_node",
                name="hmi_node",
                output="screen",
            ),
        ]
    )
