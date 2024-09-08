from launch import LaunchDescription
from launch_ros.actions import IncludeLaunchDescription
from launch_ros.descriptions import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Include the first launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('repo_controller'),
                '/launch/controller.launch.py'
            ]),
            launch_arguments={}.items()
        ),
        
        # Include the second launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('uclv_seed_robotics_ros'),
                '/launch/bringup_hand.launch.py'
            ]),
            launch_arguments={}.items()
        ),
    ])
