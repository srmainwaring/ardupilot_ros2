import os
from glob import glob
from setuptools import setup

package_name = "ap_ci_tests"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.parm")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rhys Mainwaring",
    maintainer_email="rhys.mainwaring@me.com",
    description="Continuous integration tests for ArduPilot / ROS2",
    license="GPL-3.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "time_listener = ap_ci_tests.time_listener:main",
        ],
    },
)
