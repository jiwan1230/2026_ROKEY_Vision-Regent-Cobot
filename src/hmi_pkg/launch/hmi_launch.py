import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vision_share = get_package_share_directory("vision_pkg")
    vision_config = os.path.join(vision_share, "config", "vision_params.yaml")
    default_model_path = os.path.join(vision_share, "weights", "reagent_yolov8n.pt")
    default_image_dir = os.path.join(vision_share, "sample_images")

    return LaunchDescription(
        [
            DeclareLaunchArgument("source_mode", default_value="image_dir",
                                  description="device | video_file | image_dir"),
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("image_dir", default_value=default_image_dir),
            DeclareLaunchArgument("model_path", default_value=default_model_path),
            Node(
                package="vision_pkg",
                executable="side_camera_node",
                name="side_camera_node",
                parameters=[
                    vision_config,
                    {
                        "source_mode": LaunchConfiguration("source_mode"),
                        "camera_index": LaunchConfiguration("camera_index"),
                        "image_dir": LaunchConfiguration("image_dir"),
                    },
                ],
                output="screen",
            ),
            Node(
                package="vision_pkg",
                executable="liquid_height_detector_node",
                name="liquid_height_detector_node",
                parameters=[vision_config, {"model_path": LaunchConfiguration("model_path")}],
                output="screen",
            ),
            Node(
                package="vision_pkg",
                executable="tube_state_publisher_node",
                name="tube_state_publisher_node",
                parameters=[vision_config],
                output="screen",
            ),
            Node(
                package="hmi_pkg",
                executable="hmi_node",
                name="hmi_node",
                output="screen",
            ),
        ]
    )
