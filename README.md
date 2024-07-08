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
Here's a basic example of how to use the library to move Dynamixel motors:
```cpp
```



* avvio il launch della mano
* avvio il launch dell'integratore
* ros2 topic echo /motor_state per vedere lo stato delle posizioni (per ora) dei motori
* mi servono le velocità ros2 topic pub /desired_velocity     ros2 topic pub /cmd/desired_velocity uclv_seed_robotics_ros_interfaces/msg/             MotorVelocities "{ids: [31, 32, 33, 34, 35, 36, 37, 38] ,velocities: [100, 100, 100, 100, 100, 1000, 100, 100]}"
* se starto il servizio, si prende le posizioni e integra     ros2 service call /startstop std_srvs/srv/SetBool "{data: True}"

