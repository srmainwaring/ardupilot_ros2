import os
import pytest
import yaml

from pathlib import Path
from threading import Event, Thread

import launch
import launch_pytest
import launch_ros
import rclpy
import rclpy.node

from ament_index_python.packages import get_package_share_directory

from launch.actions import (
    EmitEvent,
    ExecuteProcess,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)

from launch.event_handlers import (
    OnProcessStart,
    OnProcessExit,
    OnShutdown,
)

from launch.events import Shutdown

from launch.substitutions import (
    LocalSubstitution,
)


from builtin_interfaces.msg import Time


# tempdir for tests
# import tempfile
# tmpdir = tempfile.TemporaryDirectory()
# tmpdirname = "" #tmpdir.name

# time listener node - for checking messages received.
class TimeListener(rclpy.node.Node):
    def __init__(self):
        super().__init__("time_listener")
        self.msg_event_object = Event()

        # Declare and acquire `topic` parameter
        self.declare_parameter("topic", "ROS2_Time")
        self.topic = self.get_parameter("topic").get_parameter_value().string_value

    def start_subscriber(self):
        # Create a subscriber
        self.subscription = self.create_subscription(
            Time, self.topic, self.subscriber_callback, 1
        )

        # Add a spin thread
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node), args=(self,)
        )
        self.ros_spin_thread.start()

    def subscriber_callback(self, msg):
        self.msg_event_object.set()

        if msg.sec:
            self.get_logger().info(
                "From AP : True [sec:{}, nsec: {}]".format(msg.sec, msg.nanosec)
            )
        else:
            self.get_logger().info("From AP : False")


@launch_pytest.fixture
def generate_test_description():

    # TODO(srmainwaring) path manipulation needs tidying up.

    # current working dir
    cwd = Path.cwd()
    print(f"\ncwd: {cwd}\n")

    # workspace dir
    ws_dir = os.path.join(cwd, "..", "..", "..")
    print(f"ws_dir: {ws_dir}\n")

    #
    # START_FROM ap_ci_test.launch.py
    #

    # ensure ws_dir/dev is available
    # home = os.path.expanduser("~")
    if not ("dev" in os.listdir(ws_dir)):
        os.mkdir(os.path.join(ws_dir, "dev"))

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
                f"pty,raw,echo=0,link={os.path.join(ws_dir, mra_serial_device)} ",
                f"pty,raw,echo=0,link={os.path.join(ws_dir, ap_serial_device)} ",
            ]
        ],
        shell=True,
        output="both",
        respawn=False,
    )

    micro_ros_agent = launch_ros.actions.Node(
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
            f"{os.path.join(ws_dir, mra_serial_device)}",
            "-r",
            f"{os.path.join(ws_dir, mra_refs_file)}",
        ],
        additional_env={"PYTHONUNBUFFERED": "1"},
    )

    # SITL and MAVProxy
    dds_param_file = os.path.join(pkg_ap_ci_tests, "config", "dds.parm")
    ardupilot = ExecuteProcess(
        cmd=[
            [
                f"{os.path.join(ws_dir, ap_sim_vehicle_cmd)} ",
                "-D ",
                "-v ",
                f"{ap_vehicle} ",
                "-f ",
                f"{ap_frame} ",
                "--enable-dds ",
                "-A ",
                f'"--uartC=uart:{os.path.join(ws_dir, ap_serial_device)}" ',
                f"--add-param-file={dds_param_file} ",
            ]
        ],
        shell=True,
        output="both",
        respawn=False,
    )

    # listen to the time topic
    time_listener = launch_ros.actions.Node(
        package="ap_ci_tests",
        namespace="",
        executable="time_listener",
        name="time_listener",
        output="both",
        additional_env={"PYTHONUNBUFFERED": "1"},
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

    on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                LogInfo(
                    msg=[
                        "Launch was asked to shutdown: ",
                        LocalSubstitution("event.reason"),
                    ]
                )
            ]
        )
    )

    #
    # END_FROM ap_ci_test.launch.py
    #

    path_to_test = Path(__file__).parent

    return launch.LaunchDescription(
        [
            create_ports,
            on_start_create_ports,
            on_shutdown,
        ]
    )


@pytest.mark.launch(fixture=generate_test_description)
def test_check_ros_time_msgs_published():
    rclpy.init()
    try:
        node = TimeListener()
        node.start_subscriber()
        msgs_received_flag = node.msg_event_object.wait(timeout=40.0)
        assert msgs_received_flag, "Did not receive 'ROS_Time' msgs!"
    finally:
        rclpy.shutdown()
