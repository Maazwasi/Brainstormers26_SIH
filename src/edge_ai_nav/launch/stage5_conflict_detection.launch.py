"""Five independent local predictors; crossing missions stop at safe hold points."""
import importlib.util
import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def create(context):
    sim = get_package_share_directory('amr_simulation')
    edge = get_package_share_directory('edge_ai_nav')
    scenario = LaunchConfiguration('scenario').perform(context)
    with open(os.path.join(sim, 'config', 'warehouse_sih_demo.yaml')) as stream:
        config = yaml.safe_load(stream)
    cfg = config['warehouse']
    for key, values in cfg['stage5']['scenarios'][scenario].items():
        cfg[key].update(values)
    # One derived configuration is shared by spawn, odometry transforms, routes
    # and passive labels; canonical source coordinates remain in warehouse YAML.
    with tempfile.NamedTemporaryFile(mode='w', prefix='sih_stage5_', suffix='.yaml', delete=False) as stream:
        yaml.safe_dump(config, stream)
        config_file = stream.name
    spec = importlib.util.spec_from_file_location('stage2_spawn', os.path.join(edge, 'launch', 'five_amr_demo.launch.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actions = [ExecuteProcess(cmd=['gz', 'sim', '-r', os.path.join(sim, 'worlds', cfg['world_file'])], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])]
    for i, (rid, pose) in enumerate(cfg['robot_spawns'].items()):
        actions.append(module.robot_actions(rid, pose, cfg['robot_visuals'][rid]['colour'],
            os.path.join(sim, 'models', 'warehouse_amr.sdf.xacro'),
            os.path.join(edge, 'config', 'five_amr_bridge.yaml'), 3.0+0.35*i))
        logical = cfg['robot_visuals'][rid]['label']
        actions.append(Node(package='edge_ai_nav', executable='peer_state_node', namespace=rid,
            name='peer_state', output='screen', parameters=[{'robot_id':logical,
            'config_file':config_file, 'conflict_detection':True,
            'negotiation':LaunchConfiguration('negotiation'),
            'apply_spawn_transform':LaunchConfiguration('apply_spawn_transform')}]))
        actions.append(Node(package='edge_ai_nav', executable='task_bidder', namespace=rid,
            name='task_bidder', output='screen', parameters=[{'robot_id':rid,
            'config_file':config_file}]))
        actions.append(TimerAction(period=7.0, actions=[Node(package='edge_ai_nav',
            executable='local_waypoint_controller', namespace=rid, name='local_controller',
            output='screen', parameters=[{'robot_id':rid, 'config_file':config_file,
                'enabled':LaunchConfiguration('enabled'),
                'odom_coordinates':LaunchConfiguration('odom_coordinates')}])]))
    actions.append(TimerAction(period=5.2, actions=[Node(package='edge_ai_nav',
        executable='five_amr_spawn_observer', name='stage2_spawn_observer',
        parameters=[{'config_file':config_file}]), Node(package='rviz2', executable='rviz2',
        arguments=['-d', os.path.join(edge, 'config', 'five_amr_demo.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')))]))
    if scenario == 'safe':
        x, y, z = cfg['zones']['obstacle_demo_area']['dynamic_obstacle_spawn']
        actions.append(TimerAction(period=7.0, actions=[Node(package='ros_gz_sim', executable='create',
            arguments=['-name','dynamic_obstacle_1','-file',os.path.join(sim,'models','dynamic_obstacle_1','model.sdf'),
                       '-x',str(x),'-y',str(y),'-z',str(z)])]))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('scenario', default_value='safe',
        choices=['safe','crossing','different_time','negotiation']), DeclareLaunchArgument('enabled', default_value='false'),
        DeclareLaunchArgument('negotiation', default_value='false'),
        DeclareLaunchArgument('apply_spawn_transform', default_value='true'),
        DeclareLaunchArgument('odom_coordinates', default_value='local'),
        DeclareLaunchArgument('use_rviz', default_value='true'), OpaqueFunction(function=create)])
