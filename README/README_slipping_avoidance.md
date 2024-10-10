
# Slipping Avoidance Node

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

This ROS2 node provides a slipping avoidance mechanism for robotic manipulators, adjusting forces based on real-time sensor data. The node listens to force-torque sensor readings and calculates the delta forces to ensure proper grip during manipulation tasks.

---

## Features

- Subscribes to force sensor data and calculates the delta between current and initial forces.
- Provides a service to activate or deactivate the force adjustment mechanism.
- Publishes the adjusted force values and deviations from the initial state.
- Dynamic coefficients for fine-tuning force response.

---

## Installation

### Dependencies

- ROS 2 Foxy
- `geometry_msgs` for vector messages
- `uclv_seed_robotics_ros_interfaces` for custom interfaces

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

```bash
# Run the slipping avoidance node
ros2 run your_package_name slipping_avoidance_node
```

### Subscribing Topics

- `/sensor_state_topic` (`uclv_seed_robotics_ros_interfaces/msg/FTS3Sensors`): Provides the current forces from sensors.

### Publishing Topics

- `/desired_norm_topic` (`uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped`): Publishes the adjusted force values with respective IDs.
- `/delta_forces_topic` (`geometry_msgs/msg/Vector3Stamped`): Publishes the calculated delta between current and initial forces.

### Services

- `/slipping_service` (`uclv_seed_robotics_ros_interfaces/srv/SlippingAvoidance`): Activates or deactivates the node to start/stop the force adjustment.

**Example Service Call**:
```bash
ros2 service call /slipping_service uclv_seed_robotics_ros_interfaces/srv/SlippingAvoidance
```

### Parameters

- `coefficients` (`std::vector<double>`): A vector of coefficients used in the force adjustment calculations.
- `sensor_state_topic` (`string`): The topic name for the force-torque sensor data.
- `desired_norm_forces_topic` (`string`): The topic for publishing adjusted force values.
- `delta_forces_topic` (`string`): The topic for publishing the delta between current and initial forces.
- `slipping_service_name` (`string`): The name of the service that starts/stops force adjustment.

---

## Examples

- To adjust forces for different grippers using custom coefficients:
```bash
ros2 run your_package_name slipping_avoidance_node --ros-args -p coefficients:="[0.1, 0.2, 0.3]"
```

---

## Contributing

Please follow the standard guidelines for contributing to ROS2 packages. Feel free to open issues or submit pull requests for bug fixes or feature additions.

---

## License

This project is licensed under the MIT License.
