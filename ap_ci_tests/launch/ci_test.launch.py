import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import (
    OnProcessStart,
)
from launch.substitutions import (
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    LocalSubstitution,
    PythonExpression,
    TextSubstitution,
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

    # default params
    mra_serial_device = f"{home}/dev/ttyROS0"
    mra_serial_baud = 115200
    mra_refs_file = ""

    ap_serial_device = f"{home}/dev/ttyROS1"
    ap_serial_baud = 115200
    ap_vehicle = "ArduCopter"
    ap_frame = "quad"

    # TODO(srmainwaring) use ament for get package directory
    pkg_ardupilot = f"{home}/Code/ros2/xrce-dds/ardupilot_ros2_ws/src/ardupilot"
    print(pkg_ardupilot)

    # TODO(srmainwaring) install to config?
    mra_refs_file = f"{pkg_ardupilot}/libraries/AP_DDS/dds_xrce_profile.xml"
    print(mra_refs_file)

    # TODO(srmainwaring) set env hook to place sim_vehicle.py in path
    sim_vehicle_cmd = f"{pkg_ardupilot}/Tools/autotest/sim_vehicle.py"

    pkg_ap_ci_tests = get_package_share_directory("ap_ci_tests")

    # The micro_ros_agent and ardupilot nodes do not expose params
    # as ROS params, parse the config file and send them in as node args.
    params = os.path.join(pkg_ap_ci_tests, "config", "ardupilot-dds.yaml")

    with open(params, "r") as f:
        params_str = f.read()
        params = yaml.safe_load(params_str)
        print(params)

        mra_params = params["/micro_ros_agent"]
        if mra_params["transport"] == "serial":
            mra_serial_baud = f"{mra_params['serial_baud']}"
            # mra_serial_device = f"{mra_params['serial_device']}"
            # mra_refs_file = f"{mra_params['refs_file']}"

        ap_params = params["/ardupilot"]
        if ap_params:
            ap_vehicle = ap_params["vehicle"]
            ap_frame = ap_params["frame"]
            # ap_serial_device = ap_params["serial_device"]
            ap_serial_baud = ap_params["serial_baud"]


    # create virtual ports
    create_ports = ExecuteProcess(
        cmd=[
            [
                "socat ",
                "-d -d ",
                f"pty,raw,echo=0,link={mra_serial_device} ",
                f"pty,raw,echo=0,link={ap_serial_device} ",
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
            f"{mra_serial_baud}",
            "-D",
            f"{mra_serial_device}",
            "-r",
            f"{mra_refs_file}",
        ],
    )

    # SITL and MAVProxy
    dds_param_file = os.path.join(pkg_ap_ci_tests, "config", "dds.parm")
    ardupilot = ExecuteProcess(
        cmd=[
            [
                f"{sim_vehicle_cmd} ",
                "-D ",
                "-v ",
                f"{ap_vehicle} ",
                "-f ",
                f"{ap_frame} ",
                "--enable-dds ",
                "-A ",
                f'"--uartC=uart:{ap_serial_device}" ',
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
                                    ardupilot,
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
