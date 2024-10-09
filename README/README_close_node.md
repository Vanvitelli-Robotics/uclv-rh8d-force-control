
# Close Node

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

The `close_node` is a ROS2 node that manages the control of motor velocities based on force sensor data. It subscribes to sensor norms, and adjusts the velocity of actuators based on predefined thresholds. Additionally, it interacts with external services to activate and deactivate proportional and integrator controllers.

---

## Features

- Subscribes to sensor norm data and monitors if the force exceeds a threshold.
- Publishes motor velocity commands for motor control.
- Offers services to start or stop the node and activate proportional or integrator controllers.
- Configurable thresholds and velocity values.

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

To run the `close_node` with the default parameters, use:

```bash
ros2 run your_package_name close_node
```

To specify custom parameters, provide a YAML file:

```bash
ros2 run your_package_name close_node --ros-args --params-file path/to/your_config.yaml
```

### Subscribing Topics

- **`/measured_norm_topic`** (`uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped`): This topic receives sensor norm values and corresponding IDs.

### Publishing Topics

- **`/measured_velocity_topic`** (`uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped`): Publishes calculated motor velocity commands with respective motor IDs.

### Services

- **`/close_service_name`** (`std_srvs/srv/SetBool`): Service to start or stop the node.
- **`/proportional_service_name`** (`std_srvs/srv/SetBool`): Service to activate or deactivate the proportional controller.
- **`/integrator_service_name`** (`std_srvs/srv/SetBool`): Service to activate or deactivate the integrator controller.

### Parameters

- **`measured_norm_topic`** (`string`): Name of the topic that provides normalized forces.
- **`measured_velocity_topic`** (`string`): Name of the topic where motor velocities are published.
- **`motor_ids`** (`std::vector<int64_t>`): IDs of the motors being controlled.
- **`motor_sensor_mappings`** (`std::vector<std::string>`): Mapping between motors and the sensors monitoring them.
- **`threshold`** (`double`): Threshold value for the norm of the forces. If exceeded, action is taken.
- **`initial_velocity`** (`double`): Initial velocity to apply when the node is activated.
- **`close_service_name`** (`string`): Name of the service to start/stop the node.
- **`proportional_service_name`** (`string`): Name of the proportional service to activate.
- **`integrator_service_name`** (`string`): Name of the integrator service to activate.

---

## Examples

Here is an example of how to run the node and interact with it:

1. Run the node:
   ```bash
   ros2 run your_package_name close_node
   ```

2. Subscribe to the motor velocity topic:
   ```bash
   ros2 topic echo /measured_velocity_topic
   ```

3. Call the start service:
   ```bash
   ros2 service call /close_service_name std_srvs/srv/SetBool "{data: true}"
   ```

---

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

---

## License

This project is licensed under the MIT License.
