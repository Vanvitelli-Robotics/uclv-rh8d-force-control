# Proportional Controller Node - ROS 2

## Overview

This ROS 2 node implements a **proportional controller** that manages the relationship between measured and desired forces for various motors and sensors. The node subscribes to topics for normalized measured and desired forces, computes the weighted error for each motor based on sensor data, and applies a proportional gain to generate the motor errors. These errors are then published to a topic.

## Code Breakdown

### Dependencies

The code depends on the following ROS 2 packages and messages:

- `rclcpp`: The core C++ client library for ROS 2.
- `uclv_seed_robotics_ros_interfaces/msg/float64_with_ids_stamped.hpp`: Custom message format that contains IDs and corresponding float64 data.
- `uclv_seed_robotics_ros_interfaces/srv/set_gain.hpp`: Service for dynamically setting the proportional gain of the controller.

### Main Components

#### 1. Class: `ProportionalController`

The main class of this node, `ProportionalController`, inherits from `rclcpp::Node`. It manages parameters, subscriptions, publications, and services to control the flow of data and computation within the node.

#### 2. Parameters

The node declares the following parameters, which can be set through the launch file or command line:

- `gain`: The proportional gain used in the controller (must be non-negative).
- `motor_ids`: A list of motor IDs managed by the controller.
- `motor_sensor_mappings`: A mapping between motor IDs and sensor IDs. The format is `motor_id:sensor_id1,sensor_id2`, etc.
- `sensor_weight_mappings`: A mapping between sensor IDs and weights. The format is `sensor_id:weight`.
- `measured_norm_topic`: Topic name for measured normalized forces (default: `"norm_forces"`).
- `desired_norm_topic`: Topic name for desired normalized forces (default: `"/cmd/desired_norm_forces"`).
- `measured_velocity_topic`: Topic name where the computed errors (velocities) are published (default: `"measured_velocity"`).
- `set_gain_service_name`: Service name for dynamically updating the gain (default: `"set_gain"`).

#### 3. Data Structures

- **Motor to Sensor Map (`motor_to_sensor_map_`)**: This unordered map links motor IDs to a list of sensor IDs.
- **Sensor to Weight Map (`sensor_to_weight_map_`)**: This unordered map links sensor IDs to corresponding weights.

#### 4. Subscriptions

The node subscribes to two topics:
- `measured_norm_forces_sub_`: Subscribes to the measured forces topic.
- `desired_norm_forces_sub_`: Subscribes to the desired forces topic.

Both subscriptions trigger callbacks to handle received data and compute motor errors.

#### 5. Publication

The node publishes errors for each motor based on the computed weighted average of errors:
- `measured_velocity_pub_`: Publishes the motor errors, which are adjusted by the proportional gain.

#### 6. Service

A service is available for dynamically setting the gain value during runtime:
- `set_gain_service_`: Listens for requests to update the proportional gain.

### Functions

#### `ProportionalController()`

Constructor for the node. It declares parameters, sets up subscriptions and publications, and initializes internal mappings between motors, sensors, and weights.

#### `initialize_motor_to_sensor_map()`

This function parses the `motor_sensor_mappings` parameter to create the `motor_to_sensor_map_`. The format of the mapping allows for multiple sensors to be associated with a single motor, separated by commas (e.g., `"38:3,4"`).

#### `initialize_sensor_to_weight_map()`

Similar to the previous function, this parses the `sensor_weight_mappings` parameter to map each sensor to a set of weights. The weights are used to compute the weighted error in force.

#### `measured_norm_forces_callback()` and `desired_norm_forces_callback()`

These functions are triggered when the node receives new messages on the measured or desired forces topics. They store the received data and call `compute_and_publish_error()` to compute motor errors if both sets of forces have been received.

#### `set_gain_callback()`

This function handles incoming service requests to update the proportional gain. It checks that the new gain value is non-negative and updates the internal `gain_` variable.

#### `compute_and_publish_error()`

The core function of the controller. It:
- Iterates over each motor ID.
- Retrieves the associated sensor IDs from `motor_to_sensor_map_`.
- For each sensor ID, it calculates the error between the desired and measured normalized forces.
- Applies the associated weights from `sensor_to_weight_map_` to compute a weighted error.
- Averages the weighted errors and applies the proportional gain.
- Publishes the result as a motor error.

### Special Cases

#### Handling Multiple Sensors for One Motor

When multiple sensors are mapped to a single motor (e.g., `"38:3,4"`), the code calculates the weighted error for each sensor individually and combines them based on the specified weights. This ensures the controller accurately reflects the contribution of each sensor to the motor's total error.

#### Error Handling

The code checks for the following conditions:
- Missing or invalid motor/sensor mappings.
- Sensors present in the motor mapping but not in the incoming measured/desired forces data.
- Missing weight mappings for any sensor.

Each error condition is logged appropriately using ROS 2 logging mechanisms (`RCLCPP_ERROR`, `RCLCPP_FATAL`, etc.).

### Example Launch

```bash
ros2 launch <your_package> controller.launch.py
```
In the launch file, you can set the required parameters like:
```python
gain: 1.5
motor_ids: [35, 36, 37, 38]
motor_sensor_mappings: ["35:0", "36:1", "37:2", "38:3,4"]
sensor_weight_mappings: ["0:1", "1:1", "2:1", "3:0.6", "4:0.4"]
measured_norm_topic: "measured_forces"
desired_norm_topic: "/cmd/desired_norm_forces"
measured_velocity_topic: "measured_velocity"
set_gain_service_name: "set_gain"
```

## Running the node

To install this package, follow these steps:

1. Build the package using `colcon`:
```sh
colcon build --packages-select repo_controller
```
2. Source the workspace:
```sh
source install/setup.bash
```
3. Launch the node:
```sh
ros2 run repo_controller proportional_controller_node
```
