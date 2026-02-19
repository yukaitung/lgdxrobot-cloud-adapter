# LGDXRobot Cloud Adapter

[![pipeline status](https://gitlab.com/lgdxrobotics/lgdxrobot-cloud-adapter/badges/main/pipeline.svg)](https://gitlab.com/lgdxrobotics/lgdxrobot-cloud-adapter/-/commits/main)  [![Latest Release](https://gitlab.com/lgdxrobotics/lgdxrobot-cloud-adapter/-/badges/release.svg)](https://gitlab.com/lgdxrobotics/lgdxrobot-cloud-adapter/-/releases) 

## Overview

The LGDXRobot Cloud Adapter is a ROS 2 node that integrates any robot with the LGDXRobot Cloud using ROS 2 topics and services. Integration is straightforward: simply map the topics to your robot's configuration and it's ready to go.

- LGDXRobot Cloud: ([GitLab](https://gitlab.com/lgdxrobotics/lgdxrobot-cloud) | [GitHub](https://github.com/yukaitung/lgdxrobot-cloud))
- LGDXRobot Cloud Adapter: ([GitLab](https://gitlab.com/lgdxrobotics/lgdxrobot-cloud-adapter) | [GitHub](https://github.com/yukaitung/lgdxrobot-cloud-adapter))

### Getting Help

- [Homepage](https://lgdxrobot.bristolgram.uk/cloud/)
- [Documentation](https://docs.lgdxrobot.bristolgram.uk/cloud/)
- Issue boards on both GitLab and GitHub

## Installation

### APT

1. Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
2. Install [Nav2](https://docs.nav2.org/getting_started/index.html)
3. The packages are hosted in a self-hosted repository, install this package to add the repository and the public key.

```bash
wget -q http://packages.bristolgram.uk/lgdxrobotics-apt-source.deb
sudo dpkg -i lgdxrobotics-apt-source.deb
sudo apt update
```

4. Install LGDXRobot Cloud Adapter.

```bash
sudo apt install ros-${ROS_DISTRO}-lgdxrobot-cloud-*
```

## Build from Source

### 1. Prerequisites

1. Install [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
2. Install [Nav2](https://docs.nav2.org/getting_started/index.html)

### 2. Build

Clone the project and run the following commands:

```bash
mkdir -p ~/lgdx_ws/src
cd ~/lgdx_ws/src
git clone --recurse-submodules https://gitlab.com/lgdxrobotics/lgdxrobot-cloud-adapter.git
cd ..

# Install dependencies
rosdep install --from-paths /src --ignore-src -y

# Ensure that lgdxrobot_cloud_msgs is built
colcon build --packages-select lgdxrobot_cloud_msgs  --symlink-install
source install/setup.bash

colcon build --symlink-install
```

## Robot Integration

The LGDXRobot Cloud Adapter is designed for any robot running ROS 2 and Nav2.  
The robot’s ROS node must map the topics listed below.

### Published Topics

| Topic Name | Type | Description |
| --- | --- | --- |
| /cloud/robot_data | [lgdxrobot_cloud_msgs/RobotData](https://gitlab.com/lgdxrobotics/lgdxrobot-cloud-adapter/-/blob/main/lgdxrobot_cloud_msgs/msg/RobotData.msg) | Robot data sent to the cloud |

`RobotData` includes the following fields:

| Field Name | Type | Description |
| --- | --- | --- |
| hardware_emergency_stop_enabled | bool | Indicates whether the robot is in a hardware emergency stop state. |
| batteries_voltage | float32[2] | Battery voltages: [0] for the first battery, [1] for the second battery. |

### Subscribed Topics

| Topic Name | Type | Description |
| --- | --- | --- |
| /cloud/software_emergency_stop | std_msgs/Bool | Enables or disables the software emergency stop to halt robot movement. |

### Parameters

When starting the LGDXRobot Cloud Adapter, the following parameters must be configured:

| Parameter Name | Type | Description |
| --- | --- | --- |
| slam_enable | bool | Enables or disables SLAM mode. |
| address | string | Address of the LGDXRobot Cloud. |
| root_cert | string | Path to the server root certificate. |
| client_key | string | Path to the client's private key. |
| client_cert | string | Path to the client's certificate chain. |

