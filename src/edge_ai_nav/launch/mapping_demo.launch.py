"""ALPHA-only SLAM proof using its real namespaced LaserScan and odometry."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    sim_share = get_package_share_directory("amr_simulation")
    edge_share = get_package_share_directory("edge_ai_nav")
    world = os.path.join(sim_share, "worlds", "warehouse.sdf")
    robot_sdf = os.path.join(sim_share, "models", "amr_waffle.sdf.xacro")
    bridge_config = os.path.join(edge_share, "config", "five_amr_bridge.yaml")
    slam_config = os.path.join(sim_share, "config", "slam_params.yaml")
    rviz_config = os.path.join(edge_share, "config", "five_amr_demo.rviz")

    delayed = [
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", namespace="amr_alpha",
            name="bridge", output="screen",
            parameters=[{"config_file": bridge_config, "expand_gz_topic_names": True, "use_sim_time": True}],
            remappings=[("tf", "/tf")],
        ),
        Node(
            package="ros_gz_sim", executable="create", namespace="amr_alpha", name="spawn", output="screen",
            arguments=[
                "-name", "amr_alpha", "-string", Command([
                    FindExecutable(name="xacro"), " ", robot_sdf,
                    " namespace:=amr_alpha color_r:=1.0 color_g:=0.05 color_b:=0.05",
                ]),
                "-x", "-2.0", "-y", "-2.0", "-z", "0.02",
            ],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="map_seed_to_odom",
            arguments=["--x", "-2.0", "--y", "-2.0", "--z", "0", "--frame-id", "map_seed", "--child-frame-id", "amr_alpha/odom"],
        ),
        Node(
            package="tf2_ros", executable="static_transform_publisher", name="base_to_scan",
            arguments=["--x", "-0.064", "--y", "0", "--z", "0.281", "--frame-id", "amr_alpha/base_footprint", "--child-frame-id", "amr_alpha/base_scan"],
        ),
        Node(
            package="slam_toolbox", executable="async_slam_toolbox_node", name="slam_toolbox",
            output="screen", parameters=[slam_config, {
                "use_sim_time": True,
                "odom_frame": "amr_alpha/odom",
                "map_frame": "map",
                "base_frame": "amr_alpha/base_footprint",
                "scan_topic": "/amr_alpha/scan",
            }],
        ),
        Node(
            package="edge_ai_nav", executable="fleet_robot", namespace="amr_alpha", name="mapping_driver",
            output="screen", parameters=[{"robot_id": "amr_alpha", "mode": "decentralized", "odom_is_local": True}],
        ),
        Node(
            package="rviz2", executable="rviz2", name="mapping_rviz", output="screen",
            arguments=["-d", rviz_config], condition=IfCondition(use_rviz),
        ),
    ]
    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        ExecuteProcess(cmd=["gz", "sim", "-r", world], output="screen", condition=UnlessCondition(headless)),
        ExecuteProcess(cmd=["gz", "sim", "-s", "-r", world], output="screen", condition=IfCondition(headless)),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen",
        ),
        TimerAction(period=3.0, actions=delayed),
    ])
