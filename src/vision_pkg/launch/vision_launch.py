"""Vision pipeline launch.

Ships with a bundled YOLOv8n weight (weights/reagent_yolov8n.pt) and a
handful of sample frames (sample_images/) so a fresh clone works with no
configuration. Point model_path/image_dir/source_mode/camera_index at your
own model and camera to go beyond the bundled demo, e.g.:

    ros2 launch vision_pkg vision_launch.py source_mode:=device camera_index:=0
    ros2 launch vision_pkg vision_launch.py model_path:=/path/to/your.pt
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory("vision_pkg")
    config = os.path.join(share_dir, "config", "vision_params.yaml")
    default_model_path = os.path.join(share_dir, "weights", "reagent_yolov8n.pt")
    default_image_dir = os.path.join(share_dir, "sample_images")

    model_path_arg = DeclareLaunchArgument("model_path", default_value=default_model_path)
    image_dir_arg = DeclareLaunchArgument("image_dir", default_value=default_image_dir)
    source_mode_arg = DeclareLaunchArgument("source_mode", default_value="image_dir")
    camera_index_arg = DeclareLaunchArgument("camera_index", default_value="0")

    return LaunchDescription(
        [
            model_path_arg,
            image_dir_arg,
            source_mode_arg,
            camera_index_arg,
            Node(
                package="vision_pkg",
                executable="side_camera_node",
                name="side_camera_node",
                parameters=[
                    config,
                    {
                        "source_mode": LaunchConfiguration("source_mode"),
                        "image_dir": LaunchConfiguration("image_dir"),
                        "camera_index": LaunchConfiguration("camera_index"),
                    },
                ],
                output="screen",
            ),
            Node(
                package="vision_pkg",
                executable="liquid_height_detector_node",
                name="liquid_height_detector_node",
                parameters=[config, {"model_path": LaunchConfiguration("model_path")}],
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
