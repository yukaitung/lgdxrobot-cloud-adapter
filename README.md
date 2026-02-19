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

4. Install tLGDXRobot Cloud Adapter.

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
