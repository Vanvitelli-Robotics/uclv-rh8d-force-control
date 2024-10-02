
# TaskNode - ROS2 Node

## Description
This ROS2 node is designed to execute a sequence of operations that involve publishing desired normal forces and calling various services. This node is particularly useful for controlling motors or actuators that use normal forces to avoid slippage or similar operations.

## Operation
### Sequence of Operations
1. **Publishing desired normal forces**:
   - The node publishes a message containing the desired forces and their respective IDs on the `/cmd/desired_norm_forces` topic.
2. **Calling the close service** (`/close`):
   - After publishing, the node waits for the user to press ENTER, then calls the `/close` service to begin the operation.
3. **Calling the slipping avoidance service** (`/slipping`):
   - The `/slipping` service is called, which receives the published data of desired forces and the sensor IDs.
4. **Calling the open service** (`/open`):
   - Finally, the `/open` service is called to complete the operation.

If one of the services is unavailable or the call fails, the node handles the error and stops execution.

## Dependencies
The node uses the following packages and messages:
- `rclcpp` - The ROS2 client library for creating nodes.
- `std_srvs/srv/SetBool` - For handling simple boolean services (closing and opening).
- `uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped` - For publishing desired normal forces and their IDs.
- `uclv_seed_robotics_ros_interfaces/srv/SlippingAvoidance` - For handling the slipping avoidance service.

## Parameters
The node uses two parameters that can be defined via the launch file:
- `desired_norm_data` (default: `[0.3, 0.3, 0.3, 0.3, 0.3]`): A vector of desired normal forces.
- `desired_norm_ids` (default: `[0, 1, 2, 3, 4]`): A vector of IDs associated with the desired normal forces.

## Usage

### Compilation

1. Make sure your ROS2 workspace is set up correctly.
2. Clone or copy this package into the workspace.
3. Run the build command:
   ```bash
   colcon build
   ```
4. Source the setup:
   ```bash
   source install/setup.bash
   ```

### Running the Node

Once compiled, you can run the node as follows:

```bash
ros2 run <package_name> task_node
```

### Execution Parameters

Parameters can be specified via the launch file.

For example, create a launch file `task_node_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='<package_name>',
            executable='task_node',
            name='task_node',
            output='screen',
            parameters=[{
                'desired_norm_data': [0.5, 0.5, 0.5, 0.5, 0.5],
                'desired_norm_ids': [0, 1, 2, 3, 4]
            }]
        )
    ])
```

Then, run the launch file:

```bash
ros2 launch <package_name> task_node_launch.py
```

## Service Calls

- **/close**: Called to activate a safety closure, sending a boolean value `true`.
- **/slipping**: Receives the desired forces and IDs to avoid slippage.
- **/open**: Called to reopen the system, sending a boolean value `true`.

## Error Handling
- If a service is unavailable, an error message is logged, and the process stops.
- If a service does not respond within 5 seconds, a timeout is handled, and the node stops execution.
- If the service responds but the result is negative (`success = false`), the node logs an error message.
