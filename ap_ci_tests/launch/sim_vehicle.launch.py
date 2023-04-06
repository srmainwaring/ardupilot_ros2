import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import ExecuteProcess


"""
Usage
-----

ros2 launch ap_ci_tests sim_vehicle.launch.py


Details
-------

The launch file starts ardupilot SITL loading parameters from the yaml
file ./config/ardupilot_dds.yaml. It is equivalent to the following command.

./src/ardupilot/Tools/autotest/sim_vehicle.py \
  -D -v ArduCopter \
  -f quad \
  --enable-dds \
  -A "--uartC=uart:./dev/ttyROS1" \
  --add-param-file=./config/dds.parm \
  --console
"""


def generate_launch_description():

    # default params
    ap_sim_vehicle_cmd = "sim_vehicle.py"
    ap_serial_device = f"./dev/ttyROS1"
    ap_serial_baud = 115200
    ap_vehicle = "ArduCopter"
    ap_frame = "quad"

    pkg_ardupilot = get_package_share_directory("ardupilot")
    pkg_ap_ci_tests = get_package_share_directory("ap_ci_tests")

    # The micro_ros_agent and ardupilot nodes do not expose params
    # as ROS params, parse the config file and send them in as node args.
    params = os.path.join(pkg_ap_ci_tests, "config", "ardupilot_dds.yaml")

    with open(params, "r") as f:
        params_str = f.read()
        params = yaml.safe_load(params_str)
        print(params)

        ap_params = params["/ardupilot"]
        if ap_params:
            ap_sim_vehicle_cmd = ap_params["sim_vehicle_cmd"]
            ap_vehicle = ap_params["vehicle"]
            ap_frame = ap_params["frame"]
            ap_serial_device = ap_params["serial_device"]
            ap_serial_baud = ap_params["serial_baud"]

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
                "--console",
            ]
        ],
        shell=True,
    )

    return LaunchDescription(
        [
            ardupilot,
        ]
    )
