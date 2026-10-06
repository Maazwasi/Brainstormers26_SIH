"""Attach ALPHA 1 SLAM and a recording view to an already-running V2 fleet.

Requires the fleet launch's use_rviz:=true observer for world odometry TF.
Does not launch Gazebo or publish any robot movement commands.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    edge = get_package_share_directory('edge_ai_nav')
    slam = get_package_share_directory('slam_toolbox')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                slam, 'launch', 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'autostart': 'false',
                'slam_params_file': os.path.join(edge, 'config', 'swarmx_v2_slam.yaml'),
            }.items()),
        Node(package='edge_ai_nav', executable='slam_lifecycle_guard',
             name='slam_lifecycle_guard', output='screen'),
        Node(package='rviz2', executable='rviz2', name='swarmx_live_slam_rviz',
             arguments=['-d', os.path.join(edge, 'config', 'swarmx_v2_slam.rviz')],
             parameters=[{'use_sim_time': True}], output='screen'),
    ])
