from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Define common parameters
    common_params = {
        "motor_ids": [36, 37],
        "motor_thresholds": [100, 3995]
    }

    return LaunchDescription([
        Node(
            output='screen',
            package='repo_controller',
            executable='euler_integrator_controller',
            name='euler_integrator',
            parameters=[
                common_params,  # Use common parameters
                {"dt": 0.001}   # Additional parameters specific to this node
            ]
        ),
        Node(
            output='screen',
            package='repo_controller',
            executable='proportional_controller',
            name='proportional_controller',
            parameters=[
                common_params,  # Use common parameters
                {"gain": 200.0}  # Additional parameters specific to this node
            ]
        ),
        Node(
            output='screen',
            package='repo_controller',
            executable='force_norm',
            name='force_norm',
            parameters=[
                # Add parameters specific to force_norm node if needed
            ]
        ),
    ])
