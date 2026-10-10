"""Five-AMR warehouse with Gazebo, dashboard and live LiDAR/SLAM RViz."""
import importlib.util
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    sim = get_package_share_directory('amr_simulation')
    edge = get_package_share_directory('edge_ai_nav')
    config_file = os.path.join(sim, 'config', 'warehouse_sih_v2_large.yaml')
    with open(config_file, encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)['warehouse']
    front_overhang=max(cfg['amr_collision_bounds_m']['x'])-cfg['scale_speed_upgrade']['lidar_pose_m'][0]
    world = os.path.join(sim, 'worlds', cfg['world_file'])
    robot_sdf = os.path.join(sim, 'models', 'warehouse_amr_v2_large.sdf.xacro')
    bridge = os.path.join(edge, 'config', 'five_amr_bridge_large.yaml')
    if not os.path.isfile(bridge):
        raise FileNotFoundError(
            f'Missing installed Gazebo bridge config: {bridge}. '
            'Run colcon build --symlink-install --packages-select amr_simulation edge_ai_nav '
            'before starting the simulation.'
        )
    rviz = os.path.join(edge, 'config', 'swarmx_v2_fleet.rviz')
    slam_config = os.path.join(edge, 'config', 'swarmx_v2_slam.yaml')
    slam_launch = os.path.join(get_package_share_directory('slam_toolbox'),
                               'launch', 'online_async_launch.py')
    nav2_params = os.path.join(sim, 'config', 'nav2_params.yaml')
    helper_path = os.path.join(edge, 'launch', 'five_amr_demo.launch.py')
    spec = importlib.util.spec_from_file_location('swarmx_spawn_helper', helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)

    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    use_nav2 = LaunchConfiguration('use_nav2')
    use_dashboard = LaunchConfiguration('use_dashboard')
    enabled = LaunchConfiguration('enabled')
    dashboard_port = LaunchConfiguration('dashboard_port')
    allow_reroute = LaunchConfiguration('allow_reroute')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    actions = [
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        # Nav2 is an optional passive companion and is not installed on every
        # demo machine.  Keep it opt-in so a missing Nav2 component cannot
        # tear down Gazebo, RViz, SLAM, and the SWARMX controller fleet.
        DeclareLaunchArgument('use_nav2', default_value='false'),
        DeclareLaunchArgument('use_dashboard', default_value='true'),
        DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('dashboard_port', default_value='8091'),
        DeclareLaunchArgument('allow_reroute',default_value=str(cfg['stage6']['allow_reroute']).lower()),
        DeclareLaunchArgument('max_linear_speed',default_value='3.80'),
        DeclareLaunchArgument('max_angular_speed',default_value='7.50'),
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
                                           max_linear_velocity=max_linear_speed,
                                           max_angular_velocity=max_angular_speed))
        common = {'config_file': config_file}
        actions.append(TimerAction(period=7.0, actions=[
            Node(package='edge_ai_nav', executable='local_waypoint_controller',
                 namespace=rid, name='local_controller', output='screen',
                 parameters=[dict(common, robot_id=rid, enabled=enabled,
                                  odom_coordinates='world', smooth_steering=True,
                                  lookahead_distance=1.20, max_linear_speed=max_linear_speed,
                                  max_angular_speed=max_angular_speed,
                                  linear_acceleration=0.50,
                                  waypoint_tolerance=0.18,
                                  performance_profile_dir='/tmp/swarmx-large-profiles',
                                  obstacle_stop_distance=front_overhang+0.64,
                                  obstacle_prepare_distance=front_overhang+6.76,
                                  obstacle_clear_distance=front_overhang+0.96,
                                  # Matches the model's measured emergency
                                  # braking limit; acceleration remains 0.8.
                                  emergency_deceleration=3.0,
                                  route_corridor_margin=0.15,
                                  emergency_close_distance=1.30,
                                  bypass_clearance_margin=0.35,
                                  bypass_exit_angle=0.78,
                                  bypass_rear_clearance=0.55,
                                  turn_angular_speed=2.20,
                                  goal_tolerance=0.22)]),
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
        # Nav2 is a passive, namespaced companion to the existing SWARMX
        # controller.  SLAM Toolbox remains the only map/localization stack,
        # and every Nav2 motion output is isolated from the Gazebo cmd_vel
        # bridge so local_waypoint_controller keeps exclusive motion control.
        nav2_parameters = ParameterFile(
            RewrittenYaml(
                source_file=nav2_params,
                root_key=rid,
                param_rewrites={
                    'base_frame_id': f'{rid}/base_footprint',
                    'robot_base_frame': f'{rid}/base_link',
                    'base_frame': f'{rid}/base_link',
                    'odom_frame_id': f'{rid}/odom',
                    'odom_frame': f'{rid}/odom',
                    'local_frame': f'{rid}/odom',
                    'fixed_frame': f'{rid}/odom',
                    'global_frame_id': 'slam_map',
                    'global_frame': 'slam_map',
                },
                value_rewrites={
                    'KEEPOUT_ZONE_ENABLED': 'false',
                    'SPEED_ZONE_ENABLED': 'false',
                },
                convert_types=True,
            ),
            allow_substs=True,
        )
        nav2_remappings = [
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('map', '/map'),
            ('cmd_vel', 'nav2_cmd_vel'),
        ]
        nav2_nodes = [
            Node(package='nav2_controller', executable='controller_server',
                 namespace=rid, name='controller_server', output='screen',
                 parameters=[nav2_parameters], remappings=nav2_remappings),
            Node(package='nav2_planner', executable='planner_server',
                 namespace=rid, name='planner_server', output='screen',
                 parameters=[nav2_parameters], remappings=nav2_remappings),
            Node(package='nav2_smoother', executable='smoother_server',
                 namespace=rid, name='smoother_server', output='screen',
                 parameters=[nav2_parameters], remappings=nav2_remappings),
            Node(package='nav2_behaviors', executable='behavior_server',
                 namespace=rid, name='behavior_server', output='screen',
                 parameters=[nav2_parameters], remappings=nav2_remappings),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 namespace=rid, name='lifecycle_manager_navigation_core', output='screen',
                 parameters=[nav2_parameters, {
                     'autostart': True,
                     'bond_timeout': 15.0,
                     'node_names': [
                         'controller_server', 'smoother_server', 'planner_server',
                         'behavior_server',
                     ],
                 }]),
        ]
        nav2_bt_nodes = [
            Node(package='nav2_bt_navigator', executable='bt_navigator',
                 namespace=rid, name='bt_navigator', output='screen',
                 parameters=[nav2_parameters], remappings=nav2_remappings),
            Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                 namespace=rid, name='lifecycle_manager_navigation', output='screen',
                 parameters=[nav2_parameters, {
                     'autostart': True,
                     'bond_timeout': 15.0,
                     'node_names': ['bt_navigator'],
                 }]),
        ]
        # SLAM must publish slam_map before costmaps activate.  Staggering the
        # five stacks also avoids lifecycle bond timeouts on this simulation.
        actions.append(TimerAction(period=42.0 + 6.0 * index, actions=nav2_nodes,
                                   condition=IfCondition(use_nav2)))
        actions.append(TimerAction(period=52.0 + 6.0 * index, actions=nav2_bt_nodes,
                                   condition=IfCondition(use_nav2)))
    actions.append(TimerAction(period=5.5, actions=[
        Node(package='edge_ai_nav', executable='five_amr_spawn_observer',
             name='swarmx_v2_visual_observer', output='screen',
             parameters=[{'config_file': config_file, 'world_odom': True,
                          'use_sim_time': True}],
             condition=IfCondition(use_rviz)),
        Node(package='edge_ai_nav', executable='fleet_dashboard',
             name='fleet_dashboard', output='screen',
             parameters=[{'config_file': config_file, 'port': dashboard_port}],
             condition=IfCondition(use_dashboard)),
        Node(package='edge_ai_nav', executable='fleet_route_visualizer',
             name='fleet_route_visualizer', output='screen',
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(use_rviz)),
        Node(package='rviz2', executable='rviz2', name='swarmx_v2_rviz',
             arguments=['-d', rviz], parameters=[{'use_sim_time': True}],
             output='screen', condition=IfCondition(use_rviz)),
    ]))
    actions.append(TimerAction(period=20.0, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'use_sim_time': 'true',
                'autostart': 'false',
                'slam_params_file': slam_config,
            }.items(),
            condition=IfCondition(use_rviz)),
        Node(package='edge_ai_nav', executable='slam_lifecycle_guard',
             name='slam_lifecycle_guard', output='screen',
             condition=IfCondition(use_rviz)),
    ]))
    return LaunchDescription(actions)
