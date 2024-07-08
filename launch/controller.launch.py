from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     output='screen',
        #     package='repo_controller',
        #     executable='euler_integrator_controller',
        #     name='euler_integrator',
        #     parameters=[
        #         {"motor_ids": [31, 32, 33, 34, 35, 36, 37, 38]},
        #         {"dt": 0.1}
        #     ]
        # ),
        Node(
            output='screen',
            package='repo_controller',
            executable='proportional_controller',
            name='proportional_controller',
            parameters=[
                {"gain": 1.0}
            ]
        ),
    ])
