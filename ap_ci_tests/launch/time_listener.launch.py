import os
import yaml

from launch import LaunchDescription
from launch_ros.actions import Node


"""
Usage
-----

1. Run

ros2 launch ap_ci_tests time_listener.launch.py


Details
-------

The launch file starts the following processes / nodes and uses the launch
event handler to ensure the nodes are started in the correct sequence.
The launch file is equivalent to running the following on the command line
from the workspace directory.

1. Run test node that listens to /ROS_Time

ros2 run ap_ci_tests time_listener

"""


def generate_launch_description():

    # listen to the time topic
    time_listener = Node(
        package="ap_ci_tests",
        namespace="",
        executable="time_listener",
        name="time_listener",
        output="both",
    )

    return LaunchDescription([time_listener])
