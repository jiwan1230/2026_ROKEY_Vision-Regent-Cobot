"""Top-level system launch (spec doc section 20 recommended layout).

Brings up the full pipeline: vision (camera + YOLOv8n detector + state
publisher) -> robot_control (mock Doosan M0609 control + decision logic)
-> hmi (operator dashboard).

Usage:
    ros2 launch launch/system_launch.py
    ros2 launch launch/system_launch.py launch_hmi:=false
    ros2 launch launch/system_launch.py source_mode:=device camera_index:=0
    ros2 launch launch/system_launch.py model_path:=/path/to/your.pt
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    # Mirrors vision_launch.py's own bundled-weight/sample-image defaults so
    # this top-level launch file works out of the box too, while still
    # allowing overrides (ros2 launch ... model_path:=/path/to/your.pt).
    vision_share_dir = get_package_share_directory("vision_pkg")
    default_model_path = os.path.join(vision_share_dir, "weights", "reagent_yolov8n.pt")
    default_image_dir = os.path.join(vision_share_dir, "sample_images")

    launch_hmi_arg = DeclareLaunchArgument(
        "launch_hmi", default_value="true", description="Whether to start the HMI dashboard"
    )
    source_mode_arg = DeclareLaunchArgument("source_mode", default_value="image_dir")
    camera_index_arg = DeclareLaunchArgument("camera_index", default_value="0")
    model_path_arg = DeclareLaunchArgument("model_path", default_value=default_model_path)
    image_dir_arg = DeclareLaunchArgument("image_dir", default_value=default_image_dir)

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("vision_pkg"), "launch", "vision_launch.py"
            )
        ),
        launch_arguments={
            "source_mode": LaunchConfiguration("source_mode"),
            "camera_index": LaunchConfiguration("camera_index"),
            "model_path": LaunchConfiguration("model_path"),
            "image_dir": LaunchConfiguration("image_dir"),
        }.items(),
    )

    robot_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("robot_control_pkg"),
                "launch",
                "robot_control_launch.py",
            )
        )
    )

    hmi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("hmi_pkg"), "launch", "hmi_launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("launch_hmi")),
    )

    return LaunchDescription(
        [
            launch_hmi_arg,
            source_mode_arg,
            camera_index_arg,
            model_path_arg,
            image_dir_arg,
            vision_launch,
            robot_control_launch,
            hmi_launch,
        ]
    )
