# Proportional Controller Node

## Overview

`proportional_controller_node` is a proportional controller implemented in C++ for the ROS 2 framework. Its primary purpose is to compute the error between desired and measured normalized forces for a group of motors and apply a proportional gain to achieve precise motor control.

## Main Functionality

- **Receives Measured Normalized Forces**: Subscribes to sensor data that provides the measured normalized forces associated with each motor.
- **Receives Desired Normalized Forces**: Subscribes to a topic to receive desired normalized forces, which act as target values for control.
- **Calculates Control Errors**: Computes the control error for each motor based on the difference between the desired and measured forces.
- **Publishes Control Errors**: Applies the proportional gain to the computed errors and publishes the results for each motor.
- **Allows Dynamic Gain Adjustment**: Provides a service that enables the gain value to be dynamically updated during runtime.

## Node Details

### Parameters

1. **`gain`** (double, default: 1.0)  
   The proportional gain value used in the control algorithm. Must be non-negative.

2. **`motor_ids`** (list of int64, default: empty)  
   The list of motor IDs managed by this controller. This parameter must be set; otherwise, the node will terminate with an error.

### Subscriptions

1. **`norm_forces`** (`uclv_seed_robotics_ros_interfaces/msg/SensorsNorm`)  
   Topic where the node subscribes to receive the measured normalized forces from the sensors.  
   **Message Fields**:
   - `header` (std_msgs/Header): The standard ROS 2 message header.
   - `ids` (int64[]): List of sensor IDs.
   - `norms` (float64[]): List of corresponding normalized force values for each sensor.

2. **`/cmd/desired_norm_forces`** (`uclv_seed_robotics_ros_interfaces/msg/SensorsNorm`)  
   Topic where the node subscribes to receive the desired normalized forces to be achieved by the motors.  
   **Message Fields**:  
   - Same as above.

### Publisher

1. **`/result_proportional_controller`** (`uclv_seed_robotics_ros_interfaces/msg/MotorError`)  
   Topic where the node publishes the computed motor errors.  
   **Message Fields**:
   - `header` (std_msgs/Header): The standard ROS 2 message header.
   - `motor_ids` (uint16[]): List of motor IDs for which the errors are computed.
   - `errors` (float64[]): Corresponding list of computed errors for each motor.

### Service

1. **`set_gain`** (`uclv_seed_robotics_ros_interfaces/srv/SetGain`)  
   Service that allows dynamic updating of the proportional gain.  
   **Request Fields**:
   - `gain` (float64): The new gain value to be set.  
   **Response Fields**:
   - `success` (bool): Indicates if the gain update was successful.
   - `message` (string): Provides feedback or an error message about the gain update request.

### Callbacks

1. **`measured_norm_forces_callback`**  
   Triggered when new measured normalized forces are received.  
   - Updates the internal state with the latest measured forces.
   - Calls `compute_and_publish_error` if both measured and desired forces have been received.

2. **`desired_norm_forces_callback`**  
   Triggered when new desired normalized forces are received.  
   - Updates the internal state with the latest desired forces.
   - Calls `compute_and_publish_error` if both measured and desired forces have been received.

3. **`set_gain_callback`**  
   Triggered when a request is made to update the proportional gain.  
   - Validates the new gain value (must be non-negative).
   - Updates the internal gain state if valid and returns a success response.

### Error Handling

- If the `gain` parameter is negative, the node will log a fatal error and terminate.
- If the `motor_ids` parameter is empty or not set, the node will also terminate with a fatal error.
- During the computation, if a motor ID does not have an associated sensor mapping, an error is logged, and the computation continues for the remaining IDs.

## Usage Example

To run the node, add it to your ROS 2 launch file or start it using the command line:

```sh
ros2 run repo_controller proportional_controller
```

## Node Flow

1. The node starts and initializes parameters.
2. Subscriptions to sensor data (`norm_forces`) and desired forces (`/cmd/desired_norm_forces`) are set up.
3. On receiving data from either subscription, the corresponding callback is triggered:
   - If both measured and desired forces have been received, the `compute_and_publish_error` method is called.
   - This method calculates the error for each motor, applying the proportional gain, and publishes the results.
4. If a client sends a service request to update the gain, the `set_gain_callback` method is invoked to handle the request

## Depedencies

- ROS 2 Foxy or later.
- `uclv_seed_robotics_ros_interfaces`: Custom message and service definitions package.


## Installation

To install this package, follow these steps:

1. Clone the repository into your ROS 2 workspace:
```sh
git clone https://github.com/robertochello/repo_controller.git
```
2. Build the package using `colcon`:
```sh
colcon build --packages-select repo_controller
```
3. Source the workspace:
```sh
source install/setup.bash
```

## Conclusion
`proportional_controller_node` is a flexible and dynamic proportional control node that calculates motor errors based on normalized forces. Its design allows easy integration into larger robotics systems that require precise control over motor forces, with dynamic adjustment capabilities for the control gain.

