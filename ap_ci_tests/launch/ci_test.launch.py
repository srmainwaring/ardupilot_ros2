from launch import LaunchDescription

from launch_ros.actions import Node

from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import (
    OnExecutionComplete,
    OnProcessExit,
    OnProcessIO,
    OnProcessStart,
    OnShutdown,
)
from launch.events import Shutdown
from launch.substitutions import (
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    LocalSubstitution,
    PythonExpression,
)

# from ament_index_python.packages import get_package_share_directory

"""
Launch files
https://roboticscasual.com/tutorial-ros2-launch-files-all-you-need-to-know/
https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Event-Handlers.html

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

    home = "/Users/rhys"
    device = f"{home}/dev/ttyROS"
    baudrate = 115200
    # pkg_ardupilot = get_package_directory("ardupilot")
    pkg_ardupilot = f"{home}/Code/ros2/xrce-dds/ardupilot_ros2_ws/src/ardupilot"
    print(pkg_ardupilot)
  
    dds_profile = f"{pkg_ardupilot}/libraries/AP_DDS/dds_xrce_profile.xml"
    print(dds_profile)

    # create virtual ports
    create_ports = ExecuteProcess(
        cmd=[
            [
                "socat ",
                "-d -d ",
                f"pty,raw,echo=0,link={device} ",
                f"pty,raw,echo=0,link={device}0 ",
            ]
        ],
        shell=True,
        output="both",
        respawn=False,
    )

    micro_ros_agent = Node(
        package="micro_ros_agent",
        namespace="",
        executable="micro_ros_agent",
        name="micro_ros_agent",
        output="both",
        arguments=[
            "serial",
            "-b",
            f"{baudrate}",
            "-D",
            f"{device}",
            "-r",
            f"{dds_profile}",
        ],
    )

    # SITL and MAVProxy
    sim_vehicle_cmd = f"{pkg_ardupilot}/Tools/autotest/sim_vehicle.py"
    sim_vehicle = ExecuteProcess(
        cmd=[
            [
                f"{sim_vehicle_cmd} ",
                "-D ",
                "-v ",
                "ArduCopter ",
                "-f ",
                "quad ",
                "--enable-xrce-dds ",
                "-A ",
                f'"--uartC=uart:{device}0" ',
                "--console"
            ]
        ],
        shell=True,
        output="both",
        respawn=False,
    )

    # listen to the time topic
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

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=create_ports,
                    on_start=[
                      LogInfo(msg='create_ports started'),

                      RegisterEventHandler(
                          event_handler=OnProcessStart(
                              target_action=micro_ros_agent,
                              on_start=[
                                LogInfo(msg='micro_ros_agent started'),
                                sim_vehicle,
                                time_listener,
                              ]
                          )
                      ),

                      micro_ros_agent,
                    ],
                )
            ),

            create_ports,
        ]
    )
