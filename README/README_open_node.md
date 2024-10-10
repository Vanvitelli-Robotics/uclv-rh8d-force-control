# Open Node

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Installation](#installation)
  - [Dependencies](#dependencies)
  - [Build Instructions](#build-instructions)
- [Usage](#usage)
  - [Running the Node](#running-the-node)
  - [Subscribing Topics](#subscribing-topics)
  - [Publishing Topics](#publishing-topics)
  - [Services](#services)
  - [Parameters](#parameters)
- [Examples](#examples)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

The `Open` node controls motor positions by stopping integrator and proportional controllers and setting the desired positions for motors. It interacts with external services for calibration and controller management.

---

## Features

- Subscribes to services to manage the integrator and proportional controllers.
- Publishes motor positions to achieve a desired configuration.
- Includes a calibration feature triggered by a service call.

---

## Installation

### Dependencies

- ROS 2 Humble
- `uclv_seed_robotics_ros_interfaces` package for custom messages
- `std_srvs` for ROS2 service definitions

### Build Instructions

```bash
# Clone this repository into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/yourusername/your-ros2-package.git

# Build the workspace
cd ~/ros2_ws
colcon build

# Source the workspace
source install/setup.bash
```
## Usage

### Running the Node

To run the `open` node with default parameters, use:

```bash
ros2 run your_package_name open
```
To specify custom parameters, provide a YAML configuration file:
```bash
ros2 run your_package_name open --ros-args --params-file path/to/your_config.yaml
```
### Subscribing Topics

None.

### Publishing Topics

- **`/desired_position_topic`** (`uclv_seed_robotics_ros_interfaces/msg/MotorPositions`): Publishes the desired motor positions with corresponding motor IDs.

### Services

- **`/open_service_name`** (`std_srvs/srv/SetBool`): Service to start or stop the motor positioning process.
- **`/integrator_service_name`** (`std_srvs/srv/SetBool`): Service to start or stop the integrator controller.
- **`/proportional_service_name`** (`std_srvs/srv/SetBool`): Service to start or stop the proportional controller.
- **`/calibrate_service_name`** (`std_srvs/srv/Trigger`): Service to trigger the calibration process.

### Parameters

- **`integrator_service_name`** (`string`): Name of the integrator service.
- **`proportional_service_name`** (`string`): Name of the proportional controller service.
- **`calibrate_service_name`** (`string`): Name of the calibration service.
- **`desired_position_topic`** (`string`): Name of the topic to publish motor positions.
- **`open_service_name`** (`string`): Name of the service to start the node.
- **`motor_ids`** (`std::vector<int64_t>`): List of motor IDs to control.
- **`motor_positions`** (`std::vector<double>`): Desired positions for the motors.

---

## Examples

Here is an example of how to run the node and interact with it:

1. Run the node:
   ```bash
   ros2 run your_package_name open
    ```
2. Call the start service:
```bash
ros2 service call /open_service_name std_srvs/srv/SetBool "{data: true}"
```
---

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

---

## License

This project is licensed under the MIT License.
