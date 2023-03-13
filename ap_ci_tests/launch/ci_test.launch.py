import os

from launch import LaunchDescription

from ament_index_python.packages import get_package_share_directory

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
A. Setup 

Create a directory for the virtual ttys.

mkdir $HOME/dev

B. Run

1. Run socat

socat -d -d -v pty,raw,echo=0,link=$HOME/dev/ttyROS pty,raw,echo=0,link=$HOME/dev/ttyROS0

2. Run micro_ros_agent

ros2 run micro_ros_agent micro_ros_agent \
  serial -b 115200 -D $HOME/dev/ttyROS \
  -r ./src/ardupilot/libraries/AP_XRCE_Client/dds_xrce_profile.xml

3. Run simulation

../Tools/autotest/sim_vehicle.py \
  -D --enable-xrce-dds \
  -A "--uartC=uart:$HOME/dev/ttyROS0" \
  --add-param-file=./config/dds.parm \
  --console

4. Run test node

ros2 launch ap_std_msg_subscribers ci_test.launch.py

"""


def generate_launch_description():
    # TODO(srmainwaring) remove
    home = "/Users/rhys"

    # TODO(srmainwaring) resolve params from config/config.yaml
    # params
    device = f"{home}/dev/ttyROS"
    baudrate = 115200
    vehicle = "ArduCopter"
    frame = "quad"

    # TODO(srmainwaring) use ament for get package directory
    pkg_ardupilot = f"{home}/Code/ros2/xrce-dds/ardupilot_ros2_ws/src/ardupilot"
    print(pkg_ardupilot)

    # TODO(srmainwaring) install to config?
    dds_profile_file = f"{pkg_ardupilot}/libraries/AP_DDS/dds_xrce_profile.xml"
    print(dds_profile_file)

    # TODO(srmainwaring) set env hook to place sim_vehicle.py in path
    sim_vehicle_cmd = f"{pkg_ardupilot}/Tools/autotest/sim_vehicle.py"

    pkg_ap_ci_tests = get_package_share_directory("ap_ci_tests")
    print(pkg_ap_ci_tests)

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
            f"{dds_profile_file}",
        ],
    )

    # SITL and MAVProxy
    dds_param_file = os.path.join(pkg_ap_ci_tests, "config", "dds.parm")
    sim_vehicle = ExecuteProcess(
        cmd=[
            [
                f"{sim_vehicle_cmd} ",
                "-D ",
                "-v ",
                f"{vehicle} ",
                "-f ",
                f"{frame} ",
                "--enable-dds ",
                "-A ",
                f'"--uartC=uart:{device}0" ',
                f"--add-param-file={dds_param_file} ",
                "--console",
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
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=create_ports,
                    on_start=[
                        LogInfo(msg="create_ports started"),
                        RegisterEventHandler(
                            event_handler=OnProcessStart(
                                target_action=micro_ros_agent,
                                on_start=[
                                    LogInfo(msg="micro_ros_agent started"),
                                    sim_vehicle,
                                    time_listener,
                                ],
                            )
                        ),
                        micro_ros_agent,
                    ],
                )
            ),
            create_ports,
        ]
    )
