"""Stage 4: independent DDS peer exchange layered on Stage 3 autonomy."""
import os, yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    edge = get_package_share_directory('edge_ai_nav'); sim = get_package_share_directory('amr_simulation')
    config = os.path.join(sim, 'config', 'warehouse_sih_demo.yaml')
    with open(config) as f: metadata = yaml.safe_load(f)['warehouse']['stage4_peer_metadata']
    actions = [DeclareLaunchArgument('enabled', default_value='false'),
               DeclareLaunchArgument('use_rviz', default_value='true'),
               IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(edge, 'launch','stage3_autonomy.launch.py')), launch_arguments={'enabled':LaunchConfiguration('enabled'), 'use_rviz':LaunchConfiguration('use_rviz')}.items())]
    for logical_id, meta in metadata.items():
        actions.append(Node(package='edge_ai_nav', executable='peer_state_node', namespace=meta['namespace'].lstrip('/'), name='peer_state', output='screen', parameters=[{'robot_id':logical_id, 'config_file':config, 'publish_rate_hz':5.0, 'peer_stale_warning':1.0, 'peer_remove_timeout':2.0}]))
    return LaunchDescription(actions)
