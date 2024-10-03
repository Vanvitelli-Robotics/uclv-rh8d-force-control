
# Open Node

`Open` node is responsible for controlling motors by interacting with external controllers and services. It can stop the integrator and proportional controllers, set motor positions, and trigger a calibration sequence.

## Node Details

### Published Topics
- **`desired_position_topic_name`** (`uclv_seed_robotics_ros_interfaces/msg/MotorPositions`): 
  The topic where the node publishes the desired motor positions to control the motors.

### Services
- **`node_service_name`** (`std_srvs/srv/SetBool`): 
  A service to activate or deactivate the open node. 
  - When activated, the node will stop the integrator and proportional controllers and set the motors to specified positions.
  - When deactivated, no action will be taken.

### Parameters
- **`integrator_service_name`** (`string`): 
  The name of the service to stop the integrator controller.
  
- **`proportional_service_name`** (`string`): 
  The name of the service to stop the proportional controller.

- **`calibrate_client_name`** (`string`): 
  The service name used to trigger a sensor calibration.

- **`desired_position_topic_name`** (`string`): 
  The topic name for publishing the motor positions.

- **`motor_ids`** (`array of int64`): 
  The IDs of the motors controlled by this node.

- **`motor_positions`** (`array of double`): 
  The desired positions to be set for the motors.

- **`node_service_name`** (`string`): 
  The service name for starting or stopping the node.

## Node Logic

1. **Activation**: When the node is activated via the service, it stops the integrator and proportional controllers by sending `false` requests to the respective services. Once the controllers are stopped, the node sets the motors to the specified positions by publishing to the `desired_position_topic_name`.

2. **Calibration**: After setting the motor positions, the node triggers a calibration sequence by calling the calibration service specified by `calibrate_client_name`.

3. **Service Clients**: 
   - The node communicates with the integrator and proportional controllers using ROS2 services (`SetBool`).
   - The node also triggers a sensor calibration by calling the service defined in `calibrate_client_name`.

## Usage Example

In your launch file, set the required parameters and run the node:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='open_node',
            name='open_node',
            parameters=[
                {"motor_position":[3000.0, 100.0, 100.0, 100.0, 100.0]},
                {"motor_ids": [34, 35, 36, 37, 38]},
                {"desired_position_topic_name": "desired_position"},
                {"integrator_service_name": "startstop"},
                {"proportional_service_name": "activate_controller"},
                {"calibrate_client_name": "calibrate"},
                {"node_service_name": "open"}
            ]
        )
    ])
```

This will start the node with the specified parameters and control the motors 35, 36, and 37 by setting them to the positions [100.0, 200.0, 300.0]. The node will stop the integrator and proportional controllers, then trigger the calibration after setting the motor positions.

## Dependencies

- ROS2 Foxy or newer
- `uclv_seed_robotics_ros_interfaces` package for the custom messages
- Standard ROS2 packages (`rclcpp`, `std_srvs`)
