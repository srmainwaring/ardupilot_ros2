import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.events import Shutdown
from launch.event_handlers import (
    OnProcessStart,
    OnProcessExit,
)

"""
Usage
-----

1. Setup 

Create a `dev` directory in the workspace for the virtual ttys.

mkdir -p ./dev

2. Run

ros2 launch ap_ci_tests ci_test.launch.py


Details
-------

The launch file starts the following processes / nodes and uses the launch
event handler to ensure the nodes are started in the correct sequence.
The launch file is equivalent to running the following on the command line
from the workspace directory.

1. Run socat

socat -d -d -v pty,raw,echo=0,link=./dev/ttyROS0 pty,raw,echo=0,link=./dev/ttyROS1

We use the symlinks to the ttys created in ./dev to pass the serial ports to
the micro_ros_agent and ardupilot SITL.

2. Run micro_ros_agent

ros2 run micro_ros_agent micro_ros_agent \
  serial -b 115200 -D ./dev/ttyROS0 \
  -r ./src/ardupilot/libraries/AP_DDS/dds_xrce_profile.xml

3. Run simulation

./src/ardupilot/Tools/autotest/sim_vehicle.py \
  -D -v ArduCopter \
  -f quad \
  --enable-dds \
  -A "--uartC=uart:./dev/ttyROS1" \
  --add-param-file=./config/dds.parm \
  --console

4. Run test node that listens to /ROS_Time

ros2 run ap_ci_tests time_listener

"""


def generate_launch_description():

    # TODO(srmainwaring) set env hook in ardupilot cmake
    #     to place sim_vehicle.py in path?
    # TODO(srmainwaring) add install target in ardupilot cmake
    #     to copy mra_refs_file to config?

    # default params
    mra_serial_device = f"./dev/ttyROS0"
    mra_serial_baud = 115200
    mra_refs_file = "dds_xrce_profile.xml"

    ap_sim_vehicle_cmd = "sim_vehicle.py"
    ap_serial_device = f"./dev/ttyROS1"
    ap_serial_baud = 115200
    ap_vehicle = "ArduCopter"
    ap_frame = "quad"

    pkg_ardupilot = get_package_share_directory("ardupilot")
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
            mra_serial_device = f"{mra_params['serial_device']}"
            mra_refs_file = f"{mra_params['refs_file']}"

        ap_params = params["/ardupilot"]
        if ap_params:
            ap_sim_vehicle_cmd = ap_params["sim_vehicle_cmd"]
            ap_vehicle = ap_params["vehicle"]
            ap_frame = ap_params["frame"]
            ap_serial_device = ap_params["serial_device"]
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
                f"{ap_sim_vehicle_cmd} ",
                "-D ",
                "-v ",
                f"{ap_vehicle} ",
                "-f ",
                f"{ap_frame} ",
                "--enable-dds ",
                "-A ",
                f'"--uartC=uart:{ap_serial_device}" ',
                f"--add-param-file={dds_param_file} ",
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

    # event handlers
    on_exit_time_listener = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=time_listener,
            on_exit=[
                LogInfo(msg="time_listener exited"),
                EmitEvent(event=Shutdown(reason="Test completed")),
            ],
        )
    )

    on_start_micro_ros_agent = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=micro_ros_agent,
            on_start=[
                LogInfo(msg="micro_ros_agent started"),
                ardupilot,
                time_listener,
                on_exit_time_listener,
            ],
        )
    )

    on_start_create_ports = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=create_ports,
            on_start=[
                LogInfo(msg="create_ports started"),
                micro_ros_agent,
                on_start_micro_ros_agent,
                TimerAction(
                    period=60.0,
                    actions=[EmitEvent(event=Shutdown(reason="Test timed out"))],
                ),
            ],
        )
    )

    return LaunchDescription(
        [
            create_ports,
            on_start_create_ports,
        ]
    )
