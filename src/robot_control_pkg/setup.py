import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robot_control_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*_launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jiwan",
    maintainer_email="gwanshin12301230@gmail.com",
    description="Main PC decision logic and mock Doosan M0609 robot control",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "main_decision_node = robot_control_pkg.main_decision_node:main",
            "robot_task_manager_node = robot_control_pkg.robot_task_manager_node:main",
            "doosan_robot_control_node = robot_control_pkg.doosan_robot_control_node:main",
            "gripper_control_node = robot_control_pkg.gripper_control_node:main",
        ],
    },
)
