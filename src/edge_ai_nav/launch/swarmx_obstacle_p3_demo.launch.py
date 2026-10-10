"""One-AMR, demo-only physical obstacle recording scenario."""
import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim = get_package_share_directory('amr_simulation')
    edge = get_package_share_directory('edge_ai_nav')
    world = os.path.join(sim, 'worlds', 'warehouse_sih_v2.sdf')
    robot = os.path.join(sim, 'models', 'warehouse_amr_v2.sdf.xacro')
    obstacle = os.path.join(sim, 'models', 'dynamic_obstacle_1', 'model.sdf')
    bridge = os.path.join(edge, 'config', 'five_amr_bridge.yaml')
    rviz = os.path.join(edge, 'config', 'p3_obstacle_demo.rviz')
    helper_path = os.path.join(edge, 'launch', 'five_amr_demo.launch.py')
    spec = importlib.util.spec_from_file_location('p3_spawn_helper', helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    headless = LaunchConfiguration('headless')
    use_rviz = LaunchConfiguration('use_rviz')
    dashboard_port = LaunchConfiguration('dashboard_port')
    goal_sdf = '''<sdf version="1.9"><model name="p3_goal"><static>true</static>
      <link name="goal"><visual name="marker"><geometry><cylinder><radius>0.65</radius><length>0.04</length></cylinder></geometry>
      <material><ambient>0.05 1 0.15 1</ambient><diffuse>0.05 1 0.15 1</diffuse></material></visual></link></model></sdf>'''
    actions = [
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('dashboard_port', default_value='8092'),
        ExecuteProcess(cmd=['gz','sim','-r',world], output='screen', condition=UnlessCondition(headless)),
        ExecuteProcess(cmd=['gz','sim','-s','-r',world], output='screen', condition=IfCondition(headless)),
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),
        helper.robot_actions('alpha_1', [25.0,28.0,-1.57079632679], [1.0,0.05,0.05],
                             robot, bridge, 3.0, max_linear_velocity='0.30',
                             max_angular_velocity='0.70'),
        TimerAction(period=3.5, actions=[
            Node(package='ros_gz_sim', executable='create', name='p3_obstacle_spawn', output='screen',
                 arguments=['-name','p3_obstacle','-file',obstacle,'-x','25.0','-y','24.0','-z','0.0']),
            Node(package='ros_gz_sim', executable='create', name='p3_goal_spawn', output='screen',
                 arguments=['-name','p3_goal','-string',goal_sdf,'-x','25.0','-y','16.5','-z','0.03']),
        ]),
        TimerAction(period=6.0, actions=[
            Node(package='edge_ai_nav', executable='p3_obstacle_demo', namespace='alpha_1',
                 name='p3_obstacle_demo', output='screen'),
            Node(package='edge_ai_nav', executable='fleet_dashboard', name='fleet_dashboard',
                 output='screen', parameters=[{'config_file':os.path.join(sim,'config','warehouse_sih_v2.yaml'),
                                                'port':dashboard_port}]),
            Node(package='rviz2', executable='rviz2', name='p3_demo_rviz', output='screen',
                 arguments=['-d',rviz], parameters=[{'use_sim_time':True}],
                 condition=IfCondition(use_rviz)),
        ]),
    ]
    return LaunchDescription(actions)
