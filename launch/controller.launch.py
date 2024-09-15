from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Common parameters for all nodes
    common_params = {
        "motor_ids": [35, 36, 37],  # Motor IDs used by both controllers
        "proportional_result_topic": "result_proportional_controller",  # Topic for proportional result
    }

    # Parameters specific to each node
    euler_integrator_params = {
        "dt": 0.001,  # Time step for integration
        "motor_thresholds": [100, 3995],  # Thresholds for motors
        "desired_position_topic": "desired_position",  # Topic for desired positions
        "start_stop_service_name": "startstop"  # Service name to start/stop the integration
    }

    proportional_controller_params = {
        "motor_sensor_mappings": ["35:0", "36:1", "37:2", "38:3,4"],  # Mapping motor ID to sensors
        "gain": 1.0,  # Proportional gain for the controller
        "desired_norm_topic": "/cmd/desired_norm_forces",  # Topic for desired forces
        "set_gain_service_name": "set_gain"  # Service name to set the gain
    }

    force_norm_params = {
        "sensor_state_topic": "sensor_state",
        "norm_forces_topic": "norm_forces"
    }

    return LaunchDescription([
        Node(
            output='screen',
            package='repo_controller',
            executable='euler_integrator_controller',
            name='euler_integrator',
            parameters=[
                common_params,  # Common parameters
                euler_integrator_params  # Node-specific parameters
            ]
        ),
        Node(
            output='screen',
            package='repo_controller',
            executable='proportional_controller',
            name='proportional_controller',
            parameters=[
                common_params,  # Common parameters
                proportional_controller_params  # Node-specific parameters
            ]
        ),
        Node(
            output='screen',
            package='repo_controller',
            executable='force_norm',
            name='force_norm',
            parameters=[
                force_norm_params  # Node-specific parameters
            ]
        ),
    ])
