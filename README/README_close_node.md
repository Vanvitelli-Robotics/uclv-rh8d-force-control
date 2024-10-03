
# Close Node

`Close` node is responsible for controlling motors based on force norms received from sensors. The node listens to the normalized forces from sensors, checks if the forces exceed a threshold, and either commands motor velocities or activates a proportional controller depending on the sensor data.

## Node Details

### Subscribed Topics
- **`measured_norm_topic`** (`uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped`): 
  The topic that provides normalized force readings from the sensors. These forces are associated with specific sensor IDs.

### Published Topics
- **`measured_velocity_topic`** (`uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped`): 
  The topic where the node publishes the velocities of the motors when the forces are below the threshold.

### Services
- **`node_service_name`** (`std_srvs/srv/SetBool`): 
  A service to activate or deactivate the close node. 
  - When activated, the node will start monitoring the forces and publishing velocities to the motors.
  - When deactivated, the node will stop monitoring forces and trigger the proportional controller instead.

### Parameters
- **`measured_norm_topic`** (`string`): 
  The topic name for receiving normalized forces from sensors.
  
- **`measured_velocity_topic`** (`string`): 
  The topic name for publishing motor velocities.

- **`motor_ids`** (`array of int64`): 
  The IDs of the motors controlled by this node.

- **`threshold`** (`double`): 
  The threshold above which the node triggers the proportional controller.

- **`initial_velocity`** (`double`): 
  The initial velocity to command the motors when the node is activated.

- **`motor_sensor_mappings`** (`array of strings`): 
  Mappings between motor IDs and corresponding sensor IDs. The format for each mapping string should be:
  ```
  motor_id: sensor_id1, sensor_id2, ...
  ```

- **`node_service_name`** (`string`): 
  The service name for starting or stopping the node.

- **`proportional_service_name`** (`string`): 
  The service name to activate or deactivate the proportional controller.

- **`integrator_service_name`** (`string`): 
  The service name to activate or deactivate the integrator.

## Node Logic

1. **Activation**: When the node is activated via the service, it starts listening to the `measured_norm_topic` for normalized force data. It then checks whether the forces corresponding to the sensor IDs mapped to each motor exceed the threshold. If the forces are below the threshold, the node publishes motor velocities to `measured_velocity_topic`.

2. **Deactivation**: When the node is deactivated via the service, it stops the integrator controller using the `integrator_service_name`. When the node is deactivated via check of thresholds, it also active the `proportional_service_name`.


3. **Force Threshold Handling**: 
   - The node maps motors to sensor IDs via the parameter `motor_sensor_mappings`.
   - For each motor, it checks whether any of the associated sensor IDs report a force above the threshold.
   - If all associated sensors report forces above the threshold, the node stops commanding motor velocities and triggers the proportional controller.

4. **Service Clients**: 
   - The node communicates with the integrator and proportional controller using ROS2 services (`SetBool`).
   - When activated, the node activates the integrator service.
   - When deactivated, it activates the proportional controller service.

## Usage Example

In your launch file, set the required parameters and run the node:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='close_node',
            name='close_node',
            parameters=[
                {"measured_norm_topic": "/sensor_state"},
                {"measured_velocity_topic": "/desired_velocity"},
                {"motor_ids": [35, 36, 37]},
                {"threshold': 0.1},
                {"initial_velocity": 300},
                {"motor_sensor_mappings": ["35:0", "36:1", "37:2", "38:3,4"]},
                {"node_service_name": "/close_node_service"},
                {"proportional_service_name': "/proportional_controller_service"},
                {"integrator_service_name': "/integrator_controller_service"}
            ]
        )
    ])
```

This will start the node with the appropriate parameters and monitor forces for motors 35, 36, and 37. If the threshold is exceeded for all sensors associated with a motor, the node will stop controlling the motors and activate the proportional controller.

## Dependencies

- ROS2 Foxy or newer
- `uclv_seed_robotics_ros_interfaces` package for the custom messages
- Standard ROS2 packages (`rclcpp`, `std_srvs`)
