# ! DO NOT MANUALLY INVOKE THIS setup.py, USE COLCON INSTEAD
import os
from glob import glob
from setuptools import setup

package_name = "pose_cmds_logger"

# fetch values from package.xml
setup(
    name="pose_cmds_logger",
    version="0.0.2",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # Include all launch files.
        (os.path.join("share", package_name), glob("launch/*launch.[pxy][yma]*")),
        # Include all config files
        (os.path.join("share", package_name), glob("config/*")),
    ],
    install_requires=["setuptools", 'norlab_icp_mapper_ros'],
    zip_safe=True,
    author="Dominic Baril",
    author_email="dominic.baril@norlab.ulaval.ca",
    maintainer="Dominic Baril",
    maintainer_email="dominic.baril@norlab.ulaval.ca",
    description="Pose Commands Logger Node",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pose_cmds_logger = pose_cmds_logger.pose_cmds_logger_node:main",
        ],
    },
)
