from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define common parameters
    common_params = {
        "motor_ids": [35,36,37]
    }

    return LaunchDescription([
        Node(
            output='screen',
            package='repo_controller',
            executable='euler_integrator_controller',
            name='euler_integrator',
            parameters=[
                common_params,  # Use common parameters
                {
                    "dt": 0.001,
                    "motor_thresholds": [100, 3995]
                }   # Additional parameters specific to this node
            ]
        ),
        Node(
            output='screen',
            package='repo_controller',
            executable='proportional_controller',
            name='proportional_controller',
            parameters=[
                common_params,# Use common parameters
                {
                    "motor_sensor_mappings": ["35:0", "36:1", "37:2", "38:3,4"] # Mapping motor ID - Sensor
                }   # Additional parameters specific to this node
            ]
        ),
        Node(
            output='screen',
            package='repo_controller',
            executable='force_norm',
            name='force_norm',
            parameters=[
                # {'sensor_state_topic': 'your_new_sensor_state_topic'},
                # {'norm_forces_topic': 'your_new_norm_forces_topic'},
            ]
        ),
    ])
