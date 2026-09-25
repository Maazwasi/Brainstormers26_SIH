import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='edge_ai_nav',
            executable='navigation_node',
            name='edge_ai_navigation_node',
            output='screen',
            parameters=[
                {'rate_hz': 50.0},
                {'web_port': 8080},
                {'ref_lat': 28.6139},
                {'ref_lon': 77.2090},
                {'ref_alt': 216.0},
            ]
        )
    ])
