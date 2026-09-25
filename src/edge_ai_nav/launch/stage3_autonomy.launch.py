"""Stage 2 infrastructure plus independently enabled local controllers."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    edge = get_package_share_directory('edge_ai_nav')
    sim = get_package_share_directory('amr_simulation')
    config = os.path.join(sim, 'config', 'warehouse_sih_demo.yaml')
    with open(config) as f:
        warehouse = yaml.safe_load(f)['warehouse']
    actions = [DeclareLaunchArgument('enabled', default_value='false'),
               DeclareLaunchArgument('use_rviz', default_value='true'),
               IncludeLaunchDescription(PythonLaunchDescriptionSource(
                   os.path.join(edge, 'launch', 'five_amr_demo.launch.py')),
                   launch_arguments={'use_rviz': LaunchConfiguration('use_rviz')}.items())]
    nodes = [Node(package='edge_ai_nav', executable='local_waypoint_controller',
                  namespace=rid, name='local_controller', output='screen',
                  parameters=[{'robot_id': rid, 'config_file': config,
                               'enabled': LaunchConfiguration('enabled')}])
             for rid in warehouse['robot_spawns']]
    obstacle = warehouse['zones']['obstacle_demo_area']['dynamic_obstacle_spawn']
    nodes.append(Node(package='ros_gz_sim', executable='create', name='stage3_obstacle',
                      arguments=['-name', 'dynamic_obstacle_1', '-file',
                                 os.path.join(sim, 'models', 'dynamic_obstacle_1', 'model.sdf'),
                                 '-x', str(obstacle[0]), '-y', str(obstacle[1]), '-z', str(obstacle[2])]))
    actions.append(TimerAction(period=7.0, actions=nodes))
    return LaunchDescription(actions)
