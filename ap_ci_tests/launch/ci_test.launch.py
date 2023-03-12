from launch import LaunchDescription
from launch_ros.actions import Node

"""
A. Setup 

Create a directory for the virtual ttys.

mkdir $HOME/dev

https://stackoverflow.com/questions/49847650/how-to-determine-which-pair-of-pseudo-tty-ports-are-connected-to-each-other
https://nlamprian.me/blog/software/ros/2019/04/14/ros-node-configuration/

B. Run

1. Run socat

socat -d -d -v pty,raw,echo=0,link=$HOME/dev/ttyROS pty,raw,echo=0,link=$HOME/dev/ttyROS0

2. Run micro_ros_agent

ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D $HOME/dev/ttyROS -r ./src/ardupilot/libraries/AP_XRCE_Client/dds_xrce_profile.xml

3. Run simulation

 ../Tools/autotest/sim_vehicle.py -D --enable-xrce-dds -A "--uartC=uart:$HOME/dev/ttyROS0" --console

4. Run test node

ros2 launch ap_std_msg_subscribers ci_test.launch.py

"""


def generate_launch_description():

    # micro_ros_agent = Node(
    #     package="micro_ros_agent",
    #     namespace="",
    #     executable="micro_ros_agent",
    #     name="micro_ros_agent",
    #     output="both",
    # )

    time_listener = Node(
        package="ap_ci_tests",
        namespace="",
        executable="time_listener",
        name="time_listener",
        output="both",
        arguments=[],
        parameters=[],
        # remappings=[
        #     ("/topic", "/ROS2_Topic"),
        # ],
    )

    return LaunchDescription([time_listener])
