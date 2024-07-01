from typing import List
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            output='screen',
            package='repo_controller',
            # namespace='',
            executable='euler_integrator',
            name='euler_integrator',
            parameters=[
                {},
                {}
            ]
        ),
    ])
    