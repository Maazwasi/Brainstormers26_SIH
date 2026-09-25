"""Stage 1.5 warehouse-only preview, with an optional single ALPHA robot."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    spawn_alpha = LaunchConfiguration("spawn_alpha")
    spawn_obstacle = LaunchConfiguration("spawn_obstacle")
    sim_share = get_package_share_directory("amr_simulation")
    edge_share = get_package_share_directory("edge_ai_nav")
    world = os.path.join(sim_share, "worlds", "warehouse_sih_demo.sdf")
    robot_sdf = os.path.join(sim_share, "models", "amr_waffle.sdf.xacro")
    bridge_config = os.path.join(edge_share, "config", "five_amr_bridge.yaml")

    alpha_actions = [
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", namespace="amr_alpha",
            name="preview_bridge", output="screen",
            parameters=[{"config_file": bridge_config, "expand_gz_topic_names": True, "use_sim_time": True}],
            condition=IfCondition(spawn_alpha),
        ),
        Node(
            package="ros_gz_sim", executable="create", namespace="amr_alpha", name="preview_spawn",
            output="screen", condition=IfCondition(spawn_alpha),
            arguments=[
                "-name", "amr_alpha",
                "-string", Command([
                    FindExecutable(name="xacro"), " ", robot_sdf,
                    " namespace:=amr_alpha color_r:=1.0 color_g:=0.05 color_b:=0.05",
                ]),
                "-x", "-9.2", "-y", "-4.8", "-z", "0.02", "-Y", "0.0",
            ],
        ),
        Node(
            package="ros_gz_sim", executable="create", name="preview_dynamic_obstacle",
            output="screen", condition=IfCondition(spawn_obstacle),
            arguments=[
                "-name", "dynamic_obstacle_1",
                "-file", os.path.join(sim_share, "models", "dynamic_obstacle_1", "model.sdf"),
                "-x", "-3.0", "-y", "0.0", "-z", "0.32",
            ],
        ),
    ]
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("spawn_alpha", default_value="false"),
        DeclareLaunchArgument("spawn_obstacle", default_value="false"),
        ExecuteProcess(cmd=["gz", "sim", "-r", world], output="screen", condition=UnlessCondition(headless)),
        ExecuteProcess(cmd=["gz", "sim", "-s", "-r", world], output="screen", condition=IfCondition(headless)),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="preview_clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen",
        ),
        TimerAction(period=3.0, actions=alpha_actions),
    ])
