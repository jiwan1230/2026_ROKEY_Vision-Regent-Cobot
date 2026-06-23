import os
from glob import glob

from setuptools import find_packages, setup

package_name = "hmi_pkg"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*_launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jiwan",
    maintainer_email="gwanshin12301230@gmail.com",
    description="Operator HMI dashboard",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "hmi_node = hmi_pkg.hmi_node:main",
        ],
    },
)
