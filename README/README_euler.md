# Euler Integrator Node

## Overview

`euler_integrator_node` implements an Euler integration strategy to compute the desired motor positions for a robotic system based on the forces calculated by a proportional controller. The node subscribes to the corrected forces, applies Euler integration to update motor positions, and publishes the desired motor positions.

## Parameters

- **dt** (double): The integration time step. Must be a positive value. Default is 0.1.
- **motor_ids** (vector<int64_t>): A list of motor IDs that the node controls. This parameter is required and must not be empty.

## Topics

### Subscribed Topics
- **/result_proportional_controller** (`uclv_seed_robotics_ros_interfaces::msg::FTS3Sensors`): Receives the corrected forces computed by the proportional controller.
Published Topics
- ***/desired_position*** (`uclv_seed_robotics_ros_interfaces::msg::MotorPositions`): Publishes the desired motor positions computed by the Euler integration.

## Services

- **/startstop** (`std_srvs::srv::SetBool`): Service to start or stop the integration process.

    - Request:
      - **data** (bool): True to start the integration, False to stop it.

    - Response:
      - **success** (bool): True if the operation was successful, otherwise False.
      - **message** (string): Provides feedback about the operation.
        
## Node Functionality

### Initialization
The node initializes with parameters `dt` (time step for integration) and `motor_ids` (list of motor IDs).
If motor_ids is empty or not set, the node logs a fatal error and throws an std::runtime_error.

### Callbacks

#### `proportional_result_callback`
- Receives the corrected forces from the proportional controller.
- Updates the `proportional_result_` member variable and sets the `proportional_result_received_` flag to true.
  
#### `service_callback`

- Handles requests to start or stop the integration process.
- When starting:
  - Waits for the initial motor positions from the `/motor_state` topic.
  - Filters motor positions based on motor_ids.
  - Resets and starts the integration timer.
- When stopping:
  - Cancels the integration timer if it is running.
 
### Integration Process
The `integrate` method performs the following steps:
1. Checks if the corrected forces (`proportional_result_`) have been received.
2. For each motor ID in `motor_ids`, updates the motor position using Euler integration:
  - Computes the new position by adding the product of the force and the time step (dt).
3. Publishes the computed motor positions on the /desired_position topic.


## Example Usage

### Launch File
Create a Python launch file as shown below:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            output='screen',
            package='repo_controller',
            executable='euler_integrator',
            name='euler_integrator',
            parameters=[
                {"motor_ids": [31, 32, 33, 34, 35, 36, 37, 38]},
                {"dt": 0.1}
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
- If `motor_ids` is empty or not set, the node logs a fatal error and throws an exception.
- If a motor ID from `motor_ids` is not found in the received motor positions, the node logs a fatal error and throws an exception.
- If the initial motor positions cannot be received, the node logs a fatal error and throws an exception.
- If the integration is attempted before receiving the corrected forces, the node logs a warning.

## Conclusion
`euler_integrator_node` provides an Euler integration mechanism to compute desired motor positions in a robotic system, using corrected forces from a proportional controller. Proper configuration and parameter settings are essential for optimal performance.
