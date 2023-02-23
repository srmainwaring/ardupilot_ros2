# ardupilot-ros2

This repository contains Ros2 packages and yaml files for eProsima's Integration services.
Exact content of this repository are as follows:
- [ap_custom_interfaces](https://github.com/arshPratap/ardupilot_ros2/tree/main/ap_custom_interfaces) : Message definitions for ROS 2 to interact with Ardupilot's custom sensor topics
- [ap_custom_msg_subscribers](https://github.com/arshPratap/ardupilot_ros2/tree/main/ap_custom_msg_subscribers) : ROS2 subscribers for the above mentioned topics 
- [ap_std_msg_subscribers](https://github.com/arshPratap/ardupilot_ros2/tree/main/ap_std_msg_subscribers) : ROS2 subscribers for std_msg topics
- [ap_ros2_dds](https://github.com/arshPratap/ardupilot_ros2/tree/main/ap_ros2_dds) : yaml files for eProsima's Integration Services
    
    - [is_custom_msg](https://github.com/arshPratap/ardupilot_ros2/tree/main/ap_ros2_dds/is_custom_msg) : yaml files for the Ardupilot's custom sensor topics
    - [is_std_msg](https://github.com/arshPratap/ardupilot_ros2/tree/main/ap_ros2_dds/is_std_msg) : yaml files for the Ardupilot's std_msg topics

## Prerequisites
The following things are required before cloning this repository :

- ROS2 Humble : https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
- Clone this particular fork of Ardupilot : 
    - https://github.com/arshPratap/ardupilot
   
  and switch to **ddsPrototype** branch
- Install the DDS Agent you want to use
    - XRCE-DDS Agent : https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installing-the-agent-standalone
    - MicroROS Agent : https://micro.ros.org/docs/tutorials/core/first_application_linux/

***For Instructions regarding the commands to run on the ArduPilot end, check the README file [here](https://github.com/arshPratap/ardupilot/tree/ddsPrototype/libraries/AP_XRCE_Client#testing-with-ddsmicro-ros)*** 
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
colcon build --packages-select ap_custom_interfaces
```
In a new terminal
```
source /opt/ros/humble/setup.bash
cd ~/ros_ws
. install/setup.bash
ros2 pkg list
```
If everything is done right,you should see ap_custom_interfaces listed in the ROS-2 package list
### Build and Install the Integration Services
```
cd src/
git clone https://github.com/eProsima/Integration-Service.git --recursive
git clone https://github.com/eProsima/ROS2-SH.git
git clone https://github.com/eProsima/FastDDS-SH.git
cd ~/ros_ws
colcon build --packages-select ap_custom_msg_subscribers ap_std_msg_subscribers --cmake-args -DMIX_ROS2_PACKAGES="ap_custom_interfaces std_msgs"
```
*This build might take some time*

In a new terminal
```
source /opt/ros/humble/setup.bash
cd ~/ros_ws
. install/setup.bash
```
 - To run the node,simply ```ros2 run <package-name> <node-name>``` (From the root of the workspace directory)
 - To the run the Integration Service , simply ```integration-service src/ardupilot_ros2/ap_ros2_dds/is_custom_msg/ardu_ros2_dds_baro.yaml``` (From the root of the workspace directory) 
## Installation for testing with Micro-ROS Agent
After creating your workspace and make sure you are in the src folder of the workspace
```
git clone https://github.com/arshPratap/ardupilot_ros2.git
colcon build
```
In a new terminal
```
source /opt/ros/humble/setup.bash
cd ~/ros_ws
. install/setup.bash
ros2 pkg list
```
If everything is done right,you should see ap_custom_interfaces, ap_custom_msg_subscribers and ap_std_msg_subscribers listed in the ROS-2 package list
