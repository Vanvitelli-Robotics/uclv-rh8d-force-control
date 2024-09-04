# Controller PI for grasp

This repository contains two ROS 2 node, written in C++, which represents a PI controller thanks to which it is possible to perform a grasp action with the RH8D from Seed Robotics.


## Table of Contents

- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)

## Prerequisites

Before using this library, make sure you have the following installed:

- C++ compiler
- ROS 2
- UCLV Seed Robotics ROS

## Installation

First, install the Dynamixel SDK via ROS 2:

```bash
sudo apt-get install ros-[ROS Distribution]-dynamixel-sdk
```
Replace [ROS Distribution] with your ROS 2 distribution (e.g., foxy, galactic, humble).<br /> 
Next, clone the repository:
```bash
git clone https://github.com/Vanvitelli-Robotics/uclv-dynamixel-utils.git
```
## Usage
Read [euler_integrator_controller_node](./README/README_euler_integrator_controller_node)
Read [proportional_controller_node](./README/README_proportional_controller_node)



