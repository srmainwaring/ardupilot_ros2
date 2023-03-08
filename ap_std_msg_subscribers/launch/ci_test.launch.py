from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    time_listener = Node(
        package="ap_std_msg_subscribers",
        namespace="",
        executable="time_listener",
        name="time_listener",
        output="both",
    )

    return LaunchDescription([time_listener])
