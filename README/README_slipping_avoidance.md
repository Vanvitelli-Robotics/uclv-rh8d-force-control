
# Slipping Avoidance Node

This node provides a slipping avoidance mechanism by subscribing to force sensor data and applying a set of coefficients to calculate new force commands. The node can be activated or deactivated using a service call. When activated, the node adjusts the desired forces by calculating deviations from the initial sensor state and applying predefined coefficients.

## Overview

### Subscribed Topics:
- **`sensor_state_topic`** (uclv_seed_robotics_ros_interfaces/msg/FTS3Sensors): 
  Receives the current forces from sensors.
  
- **`desired_norm_topic`** (uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped): 
  Publishes the calculated desired force values with their respective IDs.

- **`difference_topic`** (`geometry_msgs/msg/Vector3Stamped`): 
  Publishes the difference in x and y components of the forces, calculated as deviations from the initial sensor state.

### Services:
- **`activation_service`** (uclv_seed_robotics_ros_interfaces/srv/SlippingAvoidance): 
  Allows activation or deactivation of the node. Upon activation, the node stores the initial sensor state for subsequent calculations.

## Parameters:
- **`coefficients`** (`std::vector<double>`): 
  A vector of coefficients used in the force adjustment calculations. Each coefficient corresponds to a sensor force.
  
- **`sensor_state_topic`** (`std::string`): 
  The topic name where the current force sensor data is published.
  
- **`activation_service`** (`std::string`): 
  The service name to activate or deactivate the node.
  
- **`desired_norm_topic`** (`std::string`): 
  The topic name to which the calculated desired norm forces are published.

- **`difference_topic`** (`std::string`): 
  The topic name where the difference in x and y components will be published.

## Node Behavior

### Activation:
- Upon receiving an activation service call, the node switches its state to active and records the initial sensor state.
- When active, it calculates force deviations by comparing the current force readings to the initial state and adjusting them based on the predefined coefficients.
  
### Deactivation:
- On receiving a deactivation service call, the node stops performing any further calculations and remains inactive.

## Callbacks and Functions

- **sensor_state_callback**: 
  - Processes the current force sensor data when the node is activated, calculates force deviations from the initial state, and publishes the adjusted forces.
  - Publishes the difference between the x and y components of the forces on the `difference_topic`.

- **initialize_vectors**: 
  - Initializes the `data_vec` and `ids_vec` vectors with values from the service request.
  
- **activate_callback**: 
  - Toggles the node's activation state and stores the initial sensor state when activated.

## Usage Example

1. Set the necessary parameters:
   - `sensor_state_topic`: The topic for sensor state data.
   - `desired_norm_topic`: The topic where adjusted forces will be published.
   - `difference_topic`: The topic where the difference in forces will be published.
   - `coefficients`: The set of coefficients to apply to the force deviations.
   
2. Call the activation service to start the node:
   ```bash
   ros2 service call /activate_slipping_avoidance uclv_seed_robotics_ros_interfaces/srv/SlippingAvoidance
   ```

3. The node will begin processing force data and publishing adjusted forces and force differences when activated.
4. To stop the node, call the activation service again to deactivate it.

## Error Handling

- If any required parameters are missing or invalid, the node will log an error and shut down.
