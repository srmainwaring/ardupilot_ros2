### The previous version of the repo has been moved to the [legacy](https://github.com/arshPratap/ardupilot_ros2/tree/legacy/) branch

# ardupilot-ros2

This repository contains ROS-2 packages and yaml files for eProsima's Integration services.
Exact content of this repository are as follows:
- [ap_custom_services](https://github.com/arshPratap/ardupilot_ros2/tree/main/ap_custom_services) : ROS2 services for the following services :
  - [Arming Service](https://github.com/arshPratap/ardupilot_ros2/blob/main/ap_custom_services/srv/ArmMotors.srv) : ROS-2 .srv file for arming the motors
  - [Mode Select](https://github.com/arshPratap/ardupilot_ros2/blob/main/ap_custom_services/srv/ModeSelect.srv) : ROS-2 .srv file for switching between drive modes

## Prerequisites
The following things are required before cloning this repository :

- ROS2 Humble : https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- Install the DDS Agent you want to use
    - XRCE-DDS Agent : https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installing-the-agent-standalone
    - MicroROS Agent : https://micro.ros.org/docs/tutorials/core/first_application_linux/

## Workspace Setup
Create your own ROS-2 workspace as 
``` 
source /opt/ros/humble/setup.bash
mkdir -p ~/ros_ws/src
cd ~/ros_ws/src
```
For more details regarding a ROS-2 workspace,you can refer this [link](https://docs.ros.org/en/humble/Tutorials/Workspace/Creating-A-Workspace.html) 
## Installation for testing with XRCE-DDS Agent
### Build and Install the Custom Interfaces
```
git clone https://github.com/arshPratap/ardupilot_ros2.git
cd ~/ros_ws
colcon build --packages-select ap_custom_services
```
In a new terminal
```
source /opt/ros/humble/setup.bash
cd ~/ros_ws
. install/setup.bash
ros2 pkg list
```
If everything is done right,you should see ap_custom_services listed in the ROS-2 package list
