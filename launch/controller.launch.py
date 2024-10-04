from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('repo_controller'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            output='screen',
            package='uclv_seed_robotics_ros',
            executable='hand_driver',
            name='hand_driver',
            emulate_tty=True,
            parameters=[
                config
            ]
        ),
        Node(
            output='screen',
            package="uclv_seed_robotics_ros",
            executable='fingertip_sensors',
            name='fingertip_sensors',
            emulate_tty=True,
            parameters=[
                config
            ]
        ),
        Node(
            output='screen',
            package='repo_controller',
            executable='euler_integrator_controller',
            name='euler_integrator_controller',
            emulate_tty=True,
            parameters=[
                config
            ]
        ),
        Node(
            output='screen',
            package='repo_controller',
            executable='proportional_controller',
            name='proportional_controller',
            emulate_tty=True,
            parameters=[
                config
            ]
        ),
        Node(
            output='screen',
            package='repo_controller',
            executable='force_norm',
            name='force_norm',
            emulate_tty=True,
            parameters=[
                config
            ]
        ),
        Node(
            output='screen',
            package='repo_controller',
            executable='open',
            name='open',
            emulate_tty=True,
            parameters=[
                config
            ]
        ),
        Node(
            output='screen',
            package='repo_controller',
            executable='close',
            name='close',
            emulate_tty=True,
            parameters=[
                config
            ]
        ),
        Node(
            output='screen',
            package='repo_controller',
            executable='slipping_avoidance',
            name='slipping_avoidance',
            emulate_tty=True,
            parameters=[
                config
            ]
        ),
    ])