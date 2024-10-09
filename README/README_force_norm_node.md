# Force Norm Node

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
  - [Parameters](#parameters)
- [Examples](#examples)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

The `ForceNorm` node calculates the norm of 3D forces received from force-torque sensors and publishes the result. It subscribes to the sensor data, computes the magnitude of the forces, and publishes the normalized values for further use.

---

## Features

- Subscribes to force-torque sensor data and calculates the norms of the forces.
- Publishes the calculated force norms with corresponding sensor IDs.

---

## Installation

### Dependencies

- ROS 2 Humble
- `uclv_seed_robotics_ros_interfaces` package for custom messages

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

To run the `force_norm` node with default parameters, use:

```bash
ros2 run your_package_name force_norm
```
To specify custom parameters, provide a YAML configuration file:
```bash
ros2 run your_package_name force_norm --ros-args --params-file path/to/your_config.yaml
```
### Subscribing Topics

- **`/sensor_state_topic`** (`uclv_seed_robotics_ros_interfaces/msg/FTS3Sensors`): This topic receives the force-torque sensor data, which includes the sensor IDs and 3D force vectors.

### Publishing Topics

- **`/measured_norm_topic`** (`uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped`): Publishes the calculated norms of the 3D forces, including sensor IDs and norm values.

### Parameters

- **`sensor_state_topic`** (`string`): The topic to subscribe for sensor data.
- **`measured_norm_topic`** (`string`): The topic to publish the calculated force norms.

---

## Examples

Here is an example of how to run the node and interact with it:

1. Run the node:
   ```bash
   ros2 run your_package_name force_norm
   ```
2. Subscribe to the calculated norms:
   ```bash
   ros2 topic echo /measured_norm_topic
   ```
---

## Contributing

Contributions are welcome! Please fork the repository and submit a pull request.

---

## License

This project is licensed under the MIT License.

