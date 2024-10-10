# Proportional Controller Node

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

The `ProportionalController` node controls the motors by calculating the error between desired and measured normalized forces and applying a proportional gain to adjust the motor velocities. The node subscribes to both desired and measured force data, computes the weighted errors, and publishes the results to adjust motor velocities.

---

## Features

- Subscribes to both measured and desired normalized force data.
- Computes the weighted error for each motor based on the proportional control law.
- Publishes motor velocity corrections to minimize the error.
- Allows dynamic setting of proportional gain through a service call.

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

To run the `proportional_controller` node with default parameters, use:

```bash
ros2 run your_package_name proportional_controller
```
To specify custom parameters, provide a YAML configuration file:
```bash
ros2 run your_package_name proportional_controller --ros-args --params-file path/to/your_config.yaml
```
### Subscribing Topics

- **`/measured_norm_forces_topic`** (`uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped`): Subscribes to measured normalized forces for the motors.
- **`/desired_norm_forces_topic`** (`uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped`): Subscribes to desired normalized forces for the motors.

### Publishing Topics

- **`/measured_velocity_topic`** (`uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped`): Publishes the calculated velocity corrections for each motor based on the error between measured and desired forces.

### Services

- **`/gain_service_name`** (`uclv_seed_robotics_ros_interfaces/srv/SetGain`): Service to set the proportional gain value.
- **`/proportional_service_name`** (`std_srvs/srv/SetBool`): Service to activate or deactivate the controller.

### Parameters

- **`gain`** (`double`): Proportional gain value for the controller.
- **`motor_ids`** (`std::vector<int64_t>`): List of motor IDs managed by the controller.
- **`motor_sensor_mappings`** (`std::vector<std::string>`): Mapping between motor IDs and sensor IDs.
- **`sensor_weight_mappings`** (`std::vector<std::string>`): Mapping between sensor IDs and weight values.
- **`measured_norm_forces_topic`** (`string`): Topic name for receiving measured normalized forces.
- **`desired_norm_forces_topic`** (`string`): Topic name for receiving desired normalized forces.
- **`measured_velocity_topic`** (`string`): Topic name for publishing calculated velocity corrections.
- **`gain_service_name`** (`string`): Name of the service to set the proportional gain.
- **`proportional_service_name`** (`string`): Name of the service to activate or deactivate the controller.

---

## Examples

Here is an example of how to run the node and interact with it:

1. Run the node:
   ```bash
   ros2 run your_package_name proportional_controller
   ```
2. Call the service to set the gain:
   ```bash
   ros2 service call /gain_service_name uclv_seed_robotics_ros_interfaces/srv/SetGain "{gain: 1.5}"
   ```
3. Activate the controller:
    ```bash
    ros2 service call /proportional_service_name std_srvs/srv/SetBool "{data: true}"
    ```
---

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

---

## License

This project is licensed under the MIT License.
