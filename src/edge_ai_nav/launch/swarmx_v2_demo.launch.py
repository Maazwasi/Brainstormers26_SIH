"""SWARMX V2 isolated five-AMR warehouse stack."""
import importlib.util
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = get_package_share_directory('amr_simulation')
    edge = get_package_share_directory('edge_ai_nav')
    config_file = os.path.join(sim, 'config', 'warehouse_sih_v2.yaml')
    with open(config_file, encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)['warehouse']
    world = os.path.join(sim, 'worlds', cfg['world_file'])
    robot_sdf = os.path.join(sim, 'models', 'warehouse_amr_v2.sdf.xacro')
    bridge = os.path.join(edge, 'config', 'five_amr_bridge.yaml')
    rviz = os.path.join(edge, 'config', 'swarmx_v2_demo.rviz')
    helper_path = os.path.join(edge, 'launch', 'five_amr_demo.launch.py')
    spec = importlib.util.spec_from_file_location('swarmx_spawn_helper', helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    enabled = LaunchConfiguration('enabled')
    dashboard_port = LaunchConfiguration('dashboard_port')
    allow_reroute = LaunchConfiguration('allow_reroute')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    actions = [
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('dashboard_port', default_value='8091'),
        DeclareLaunchArgument('allow_reroute',default_value=str(cfg['stage6']['allow_reroute']).lower()),
        DeclareLaunchArgument('max_linear_speed',default_value='0.60'),
        DeclareLaunchArgument('max_angular_speed',default_value='0.90'),
        ExecuteProcess(cmd=['gz', 'sim', '-r', world], output='screen',
                       condition=UnlessCondition(headless)),
        ExecuteProcess(cmd=['gz', 'sim', '-s', '-r', world], output='screen',
                       condition=IfCondition(headless)),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
             output='screen'),
    ]
    for index, (rid, pose) in enumerate(cfg['robot_spawns'].items()):
        visual = cfg['robot_visuals'][rid]
        actions.append(helper.robot_actions(rid, pose, visual['colour'],
                                           robot_sdf, bridge, 3.0 + 0.4 * index,
                                           max_linear_velocity=max_linear_speed))
        common = {'config_file': config_file}
        actions.append(TimerAction(period=7.0, actions=[
            Node(package='edge_ai_nav', executable='local_waypoint_controller',
                 namespace=rid, name='local_controller', output='screen',
                 parameters=[dict(common, robot_id=rid, enabled=enabled,
                                  odom_coordinates='world', smooth_steering=True,
                                  lookahead_distance=0.75, max_linear_speed=max_linear_speed,
                                  max_angular_speed=max_angular_speed,
                                  waypoint_tolerance=0.45,
                                  goal_tolerance=0.45)]),
            Node(package='edge_ai_nav', executable='task_bidder',
                 namespace=rid, name='task_bidder', output='screen',
                 parameters=[dict(common, robot_id=rid)]),
            Node(package='edge_ai_nav', executable='peer_state_node',
                 namespace=rid, name='peer_state', output='screen',
                 parameters=[dict(common, robot_id=visual['label'],
                                  conflict_detection=True, negotiation=True,
                                  allow_reroute=allow_reroute,
                                  apply_spawn_transform=False)]),
        ]))
    actions.append(TimerAction(period=5.5, actions=[
        Node(package='edge_ai_nav', executable='five_amr_spawn_observer',
             name='swarmx_v2_visual_observer', output='screen',
             parameters=[{'config_file': config_file, 'world_odom': True,
                          'use_sim_time': True}],
             condition=IfCondition(use_rviz)),
        Node(package='edge_ai_nav', executable='fleet_dashboard',
             name='fleet_dashboard', output='screen',
             parameters=[{'config_file': config_file, 'port': dashboard_port}]),
        Node(package='rviz2', executable='rviz2', name='swarmx_v2_rviz',
             arguments=['-d', rviz], parameters=[{'use_sim_time': True}],
             output='screen', condition=IfCondition(use_rviz)),
    ]))
    return LaunchDescription(actions)
