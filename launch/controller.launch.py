from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Common parameters for all nodes
    common_params = {
        "motor_ids": [35, 36, 37, 38],  # Motor IDs used by both controllers
        "measured_velocity_topic": "measured_velocity",  # Topic for proportional result
    }

    # Parameters for norm forces topic (shared by proportional_controller and force_norm)
    norm_forces_params = {
        "measured_norm_topic": "norm_forces"  # Topic for measured norm forces
    }

    # Parameters specific to the euler integrator node
    euler_integrator_params = {
        "dt": 0.001,  # Time step for integration
        "motor_thresholds": [100, 3995],  # Thresholds for motors (same for hand driver)
        "desired_position_topic": "desired_position",  # Topic for desired positions
        "start_stop_service_name": "startstop"  # Service name to start/stop the integration
    }

    # Parameters specific to the proportional controller node
    proportional_controller_params = {
        "motor_sensor_mappings": ["35:0", "36:1", "37:2", "38:3,4"],
        "sensor_weight_mappings": ["0:1", "1:1", "2:1", "3:0.6", "4:0.4"],
        "gain": 100.0,  # Proportional gain for the controller
        "desired_norm_topic": "/cmd/desired_norm_forces",  # Topic for desired forces
        "set_gain_service_name": "set_gain"  # Service name to set the gain
    }

    # Parameters specific to the force norm node
    force_norm_params = {
        "sensor_state_topic": "sensor_state"  # Topic for sensor state
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
                norm_forces_params,  # Shared parameters for norm forces
                proportional_controller_params  # Node-specific parameters
            ]
        ),
        Node(
            output='screen',
            package='repo_controller',
            executable='force_norm',
            name='force_norm',
            parameters=[
                norm_forces_params,  # Shared parameters for norm forces
                force_norm_params  # Node-specific parameters
            ]
        ),
    ])
