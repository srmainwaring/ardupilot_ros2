from launch import LaunchDescription

from launch.actions import ExecuteProcess


"""
Run simulation

sim_vehicle.py -D -v ArduCopter -f quad \
  --enable-xrce-dds \
  -A "--uartC=uart:/Users/rhys/dev/ttyROS0" \
  --console
"""


def generate_launch_description():

    home = "/Users/rhys"
    device = f"{home}/dev/ttyROS"
    # pkg_ardupilot = get_package_directory("ardupilot")
    pkg_ardupilot = f"{home}/Code/ros2/xrce-dds/ardupilot_ros2_ws/src/ardupilot"
    print(pkg_ardupilot)

    pkg_ap_ci_tests = f"{home}/Code/ros2/xrce-dds/ardupilot_ros2_ws/src/ardupilot_ros2/ap_ci_tests"
    print(pkg_ap_ci_tests)

    dds_profile = f"{pkg_ardupilot}/libraries/AP_DDS/dds_xrce_profile.xml"
    print(dds_profile)

    # SITL and MAVProxy
    dds_param = f"{pkg_ap_ci_tests}/config/dds.parm"
    print(dds_param)

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
                f"--add-param-file={dds_param} "
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
