"""Top-level system launch (spec doc section 20 recommended layout).

Brings up the full pipeline: vision (camera + YOLOv8n detector + state
publisher) -> robot_control (mock Doosan M0609 control + decision logic)
-> hmi (operator dashboard).

Usage:
    ros2 launch launch/system_launch.py
    ros2 launch launch/system_launch.py launch_hmi:=false
"""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    launch_hmi_arg = DeclareLaunchArgument(
        "launch_hmi", default_value="true", description="Whether to start the HMI dashboard"
    )

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("vision_pkg"), "launch", "vision_launch.py"
            )
        )
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
        [launch_hmi_arg, vision_launch, robot_control_launch, hmi_launch]
    )
