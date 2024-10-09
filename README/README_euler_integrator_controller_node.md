
# Euler Integrator Controller

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

The `EulerIntegratorController` is a ROS2 node that performs Euler integration of motor velocities to compute motor positions. It subscribes to motor velocity data, integrates the velocities over time, and publishes the desired motor positions. The controller uses a configurable threshold to avoid exceeding limits during integration.

---

## Features

- Subscribes to measured motor velocities and integrates them into motor positions.
- Publishes desired motor positions for the motors.
- Offers a service to start and stop the integration process.
- Configurable parameters for motor thresholds, integration step size (`dt`), and motor IDs.

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

---

## Usage

### Running the Node

To run the `euler_integrator_controller` with default parameters, use:

```bash
ros2 run your_package_name euler_integrator_controller
```

To specify custom parameters, provide a YAML configuration file:

```bash
ros2 run your_package_name euler_integrator_controller --ros-args --params-file path/to/your_config.yaml
```

### Subscribing Topics

- **`/measured_velocity_topic`** (`uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped`): This topic receives the measured motor velocities.

### Publishing Topics

- **`/desired_position_topic`** (`uclv_seed_robotics_ros_interfaces/msg/MotorPositions`): Publishes the integrated motor positions after applying velocity integration.

### Services

- **`/integrator_service_name`** (`std_srvs/srv/SetBool`): Service to start or stop the integration process.

### Parameters

- **`dt`** (`double`): Time step for the Euler integration.
- **`motor_ids`** (`std::vector<int64_t>`): List of motor IDs that the controller is managing.
- **`motor_thresholds`** (`std::vector<int64_t>`): Thresholds to prevent motors from exceeding their limits during integration.
- **`measured_velocity_topic`** (`string`): Topic name for receiving motor velocities.
- **`desired_position_topic`** (`string`): Topic name for publishing desired motor positions.
- **`integrator_service_name`** (`string`): Name of the service to start or stop the integrator.

---

## Examples

Here is an example of how to run the node and interact with it:

1. Run the node:
   ```bash
   ros2 run your_package_name euler_integrator_controller
   ```

2. Subscribe to the desired motor positions:
   ```bash
   ros2 topic echo /desired_position_topic
   ```

3. Call the start service to begin integration:
   ```bash
   ros2 service call /integrator_service_name std_srvs/srv/SetBool "{data: true}"
   ```

---

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

---

## License

This project is licensed under the MIT License.
