import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Include the hand_driver launch file
    hand_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('uclv_seed_robotics_ros'),
                'launch/bringup_hand.launch.py'
            )
        )
    )

    # Include the controller_driver launch file
    controller_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('repo_controller'),
                'launch/controller.launch.py'
            )
        )
    )

    return LaunchDescription([
        hand_driver_launch,
        controller_driver_launch
    ])
