# ardupilot-ros2

This repository contains ROS 2 packages and configuration files for
running ROS 2 nodes that communicate with the ArduPilot DDS client library
using the microROS agent.

This repository contains the following packages:

- ap_ci_tests: A package for testing communication between `micro_ros_agent` and the ArduPilot `AP_DDS` client.

## Prerequisites

This project has the following dependencies:

- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
   

## Install and Run

#### 1. Create a workspace folder and source ROS 2: 

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
source /opt/ros/humble/setup.bash
```

The ROS 2 tutorials contain more details regarding [ROS 2 workspaces](https://docs.ros.org/en/humble/Tutorials/Workspace/Creating-A-Workspace.html).

#### 2. Get `ros2.repos` file:

```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/srmainwaring/ardupilot/srmainwaring/ddsPrototype/Tools/scripts/ros2/ros2.repos
vcs import --recursive < ros2.repos
```

The following steps are not yet automated:

#### 2.1. Build microxrcedds_gen:

```bash
cd ~/ros2_ws/src/microxrcedds_gen
./gradlew assemble
export PATH=$PATH:$(pwd)/scripts
```

#### 2.3. Create a `dev` directory for virtual ports

```bash
cd ~/ros2_ws
mkdir ./dev
```


#### 3. Build:

```bash
cd ~/ros2_ws
colcon build
```

#### 4. Run:

```bash
cd ~/ros2_ws
source ./install/setup.zsh
ros2 launch ardupilot_ros2 ap_ci_test.launch.py
```
