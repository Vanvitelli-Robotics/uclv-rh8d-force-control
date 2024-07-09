# Proportional Controller Node

## Overview
The `ProportionalController` node implements a proportional control strategy for a robotic hand using force-torque sensors. It receives desired and measured force values, computes the error, and publishes the corrected forces based on a proportional gain.

## Parameters
- **gain** (double): The proportional gain used in the control algorithm. Must be non-negative.
- **motor_ids** (vector<int64_t>): A list of motor IDs that the node controls.

## Topics

### Subscribed Topics
- **/sensor_state** (`uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors`): Receives the measured forces from the sensors.
- **/desired_forces** (`uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors`): Receives the desired forces to be applied.

### Published Topics
- **/result_proportional_controller** (`uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors`): Publishes the corrected forces computed by the proportional controller.

## Node Functionality

### Initialization
- The node is initialized with parameters `gain` and `motor_ids`.
- A check ensures that the `gain` parameter is non-negative.
- A check ensures that the `motor_ids` parameter is not empty.

### Motor-to-Sensor Mapping
The following mappings are established between motor IDs and sensor IDs:
- Motor ID 35 -> Sensor ID 0
- Motor ID 36 -> Sensor ID 1
- Motor ID 37 -> Sensor ID 2
- Motor ID 38 -> Sensor IDs 3 and 4

### Callbacks
#### `sensor_state_callback`
- Receives the measured forces and updates the `measured_forces_` member variable.
- Logs the received forces for each sensor ID.
- Calls `compute_and_publish_result`.

#### `desired_forces_callback`
- Receives the desired forces and updates the `desired_forces_` member variable.
- Logs the received forces for each sensor ID.
- Calls `compute_and_publish_result`.

### Compute and Publish Results
The `compute_and_publish_result` method performs the following steps:
1. Checks if either `desired_forces_` or `measured_forces_` are empty and logs a warning if true.
2. Initializes the `result_msg` to store the computed forces.
3. Iterates over each motor ID in `motor_ids_`:
   - Retrieves the corresponding sensor IDs using the motor-to-sensor mapping.
   - For each sensor ID:
     - Finds the indices of the sensor ID in the `measured_forces_` and `desired_forces_` messages.
     - Computes the error between the desired and measured forces.
     - Applies the proportional gain to compute the result force.
     - Adds the result force and motor ID to `result_msg`.
     - Logs the computed result force for the motor ID.
4. Publishes the `result_msg` on the `/result_proportional_controller` topic.

## Example Usage

### Launch File
Create a Python launch file as shown below:

```python
from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            output='screen',
            package='repo_controller',
            executable='proportional_controller',
            name='proportional_controller',
            parameters=[
                {"motor_ids": [31, 32, 33, 34, 35, 36, 37, 38]},
                {"gain": 1.0}
            ]
        ),
    ])
```

### Running the Node
To run the node using ROS 2:

1. Build your ROS 2 package:
    ```bash
    colcon build --packages-select repo_controller
    ```

2. Source the ROS 2 workspace:
    ```bash
    source install/setup.bash
    ```

3. Launch the node using the launch file:
    ```bash
    ros2 launch repo_controller controller.launch.py
    ```


## Error Handling

- If `gain` is negative, the node logs a fatal error and throws an `std::invalid_argument` exception.
- If `motor_ids` is empty, the node logs a fatal error and throws a `std::runtime_error` exception.
- If no mapping is found for a motor ID, the node logs a fatal error and continues processing other motor IDs.
- If sensor IDs are not found in the received messages, the node logs a fatal error and continues processing other sensor IDs.

## Conclusion

The `ProportionalController` node provides a proportional control mechanism for robotic hands, leveraging force-torque sensors to ensure desired forces are achieved. Proper configuration and parameter settings are essential for optimal performance.
