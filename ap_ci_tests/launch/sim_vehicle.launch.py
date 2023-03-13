import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch.actions import ExecuteProcess


"""
Run simulation

sim_vehicle.py -D -v ArduCopter -f quad \
  --enable-xrce-dds \
  -A "--uartC=uart:$HOME/dev/ttyROS0" \
  --add-param-file=./config/dds.parm \
  --console
"""


def generate_launch_description():
    # TODO(srmainwaring) remove
    home = "/Users/rhys"

    # TODO(srmainwaring) resolve params from config/config.yaml
    # params
    device = f"{home}/dev/ttyROS"
    vehicle = "ArduCopter"
    frame = "quad"

    # TODO(srmainwaring) use ament for get package directory
    pkg_ardupilot = f"{home}/Code/ros2/xrce-dds/ardupilot_ros2_ws/src/ardupilot"
    print(pkg_ardupilot)

    # TODO(srmainwaring) set env hook to place sim_vehicle.py in path
    sim_vehicle_cmd = f"{pkg_ardupilot}/Tools/autotest/sim_vehicle.py"

    pkg_ap_ci_tests = get_package_share_directory("ap_ci_tests")

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
    )

    return LaunchDescription(
        [
            sim_vehicle,
        ]
    )
