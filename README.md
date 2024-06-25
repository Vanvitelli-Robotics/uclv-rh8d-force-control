# Controller PI for grasp

This repository contains a ROS 2 node, written in C++, which represents a PI controller thanks to which it is possible to perform a grasp action with the RH8D from Seed Robotics.


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
Replace [ROS Distribution] with your ROS 2 distribution (e.g., foxy, galactic, humble).
Next, clone the repository:
```bash
git clone https://github.com/yourusername/uclv-dynamixel-utils.git
cd uclv-dynamixel-utils
```
## Usage
Here's a basic example of how to use the library to move Dynamixel motors:
```cpp
#include "uclv_dynamixel_utils/hand.hpp"

class TestMoveMotors : public rclcpp::Node
{
public:
    TestMoveMotors()
    : Node("test_move_motors_node")
    {
        hand_ = std::make_shared<Hand>(serial_port_, baudrate_, protocol_version_);
        hand_->setSerialPortLowLatency(serial_port_);
        if(!hand_->initialize()) {
            throw std::runtime_error("Error: Hand not initialized");
        }

        std::vector<int> motor_ids = {31, 32, 33, 34, 35, 36, 37, 38};
        std::vector<float> motor_positions = {1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000};
        
        hand_->moveMotors(motor_ids, motor_positions);
    }

private:
    std::shared_ptr<Hand> hand_;
    std::string serial_port_ = "/dev/ttyUSB0";
    int baudrate_ = 1000000;
    float protocol_version_ = 2.0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto hand_driver_node = std::make_shared<TestMoveMotors>();
    rclcpp::spin(hand_driver_node);

    rclcpp::shutdown();
    return 0;
}
```
