# ForceNorme Node

## Overview

The `ForceNorme` ROS2 node calculates the norm (magnitude) of force vectors received from sensors and publishes these norms along with their corresponding sensor IDs. This node is useful for converting force vectors into scalar magnitudes for further processing.

## Node Functionality

### 1. Subscriptions and Publications

- **Subscription:**
  - **Topic:** `sensor_state`
  - **Message Type:** `uclv_seed_robotics_interfaces/msg/FTS3Sensors`
  - **Purpose:** Receives sensor force data, including force vectors and their associated IDs.

- **Publication:**
  - **Topic:** `norm_forces`
  - **Message Type:** `uclv_seed_robotics_interfaces/msg/SensorsNorm`
  - **Purpose:** Publishes norms of the forces, associating each norm with its corresponding sensor ID.

### 2. Message Types

- **`FTS3Sensors`**
  - **Fields:**
    - `header`: Standard message header.
    - `ids`: Array of sensor IDs.
    - `forces`: Array of force vectors with `x`, `y`, and `z` components.

- **`SensorsNorm`**
  - **Fields:**
    - `header`: Standard message header.
    - `ids`: Array of sensor IDs.
    - `norm`: Array of `Float64Stamped` messages, where each contains:
      - `header`: Message header.
      - `data`: Computed norm (magnitude) for the corresponding sensor.

### 3. Node Execution

#### Subscription Callback (`sensorStateCallback`)

The `sensorStateCallback` function processes the incoming sensor data to compute and publish norms.

1. **Create and Initialize Messages:**
   - **`norm_msg`**: An instance of `SensorsNorm` to be published.
   - Copies the sensor IDs from the incoming `FTS3Sensors` message to `norm_msg.ids`.

2. **Compute Norms:**
   - Initializes an `unordered_map<uint16_t, double>` named `norms_map` to store norms for each sensor ID.
   - Iterates through each force vector in the `FTS3Sensors` message:
     - Extracts the force vector and its corresponding sensor ID.
     - Computes the norm (magnitude) of each force vector using the formula:
       ```cpp
       double norm = std::sqrt(std::pow(force.x, 2) + std::pow(force.y, 2) + std::pow(force.z, 2)) / 1000.0;
       ```
     - Stores the computed norm in `norms_map` with the sensor ID as the key.

3. **Populate Norm Message:**
   - Iterates through the sensor IDs in `norm_msg.ids`:
     - For each sensor ID, retrieves the norm from `norms_map`. If the ID is not found in the map, assigns a default value of `0.0`.
     - Creates a `Float64Stamped` message for each ID with the computed norm and header information.
     - Adds the `Float64Stamped` message to `norm_msg.norm`.

4. **Publish Result:**
   - Publishes the `SensorsNorm` message containing the norms and their corresponding sensor IDs.
