
# Slipping Avoidance Node

This node provides a slipping avoidance mechanism by subscribing to force sensor data and applying a set of coefficients to calculate new force commands. The node can be activated or deactivated using a service call. When activated, the node adjusts the desired forces by calculating deviations from the initial sensor state and applying predefined coefficients.

## Overview

### Subscribed Topics:
- **`sensor_state_topic`** (uclv_seed_robotics_ros_interfaces/msg/FTS3Sensors): 
  Receives the current forces from sensors.
  
- **`desired_norm_topic`** (uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped): 
  Publishes the calculated desired force values with their respective IDs.

### Published Topics:
- **`desired_norm_topic`** (uclv_seed_robotics_ros_interfaces/msg/Float64WithIdsStamped): 
  Publishes the adjusted forces after applying the coefficients.

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
  
- **`desired_norm_topic`** (`std::string`): 
  The topic name to which the calculated desired norm forces are published.

- **`difference_topic`** (`std::string`): 
  The topic name where the difference in x and y components will be published.

## Node Behavior

### Activation:
- Upon receiving an activation service call, the node switches its state to active and records the initial sensor state by waiting for the first message on `sensor_state_topic`.
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
  - Toggles the node's activation state and stores the initial sensor state when activated. Uses `rclcpp::wait_for_message` to wait for the first sensor state message to initialize the reference forces.

## Error Handling

- If any required parameters are missing or invalid, the node will log an error and shut down. The node also stops execution if any fatal exceptions are caught.
