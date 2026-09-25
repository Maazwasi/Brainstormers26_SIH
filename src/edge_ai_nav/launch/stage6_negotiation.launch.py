"""Stage 6 wrapper: all five AMRs, local detectors and local negotiation."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    edge = get_package_share_directory('edge_ai_nav')
    return LaunchDescription([
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(edge, 'launch', 'stage5_conflict_detection.launch.py')),
            launch_arguments={
                'scenario': 'negotiation', 'negotiation': 'true',
                'enabled': LaunchConfiguration('enabled'),
                'use_rviz': LaunchConfiguration('use_rviz'),
            }.items())
    ])
