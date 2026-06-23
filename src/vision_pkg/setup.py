import os
from glob import glob

from setuptools import find_packages, setup

package_name = "vision_pkg"

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
    description="Side-view camera capture and YOLOv8n-based liquid height / tube state inference",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "side_camera_node = vision_pkg.side_camera_node:main",
            "liquid_height_detector_node = vision_pkg.liquid_height_detector_node:main",
            "tube_state_publisher_node = vision_pkg.tube_state_publisher_node:main",
        ],
    },
)
