"""Stage 2: five isolated AMRs in the canonical SIH warehouse, no autonomy."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def robot_actions(robot_id, pose, colour, robot_sdf, bridge_config, delay,
                  max_linear_velocity=None, max_angular_velocity=None):
    """Reusable bridge/spawn pair for one fully namespaced AMR."""
    x, y, yaw = map(float, pose)
    red, green, blue = map(float, colour)
    sdf_command=[FindExecutable(name="xacro"), " ", robot_sdf,
                 " namespace:=", robot_id,
                 " color_r:=", str(red),
                 " color_g:=", str(green),
                 " color_b:=", str(blue)]
    if max_linear_velocity is not None:
        sdf_command.extend([" max_linear_velocity:=",max_linear_velocity])
    if max_angular_velocity is not None:
        sdf_command.extend([" max_angular_velocity:=",max_angular_velocity])
    return TimerAction(period=delay, actions=[
        Node(
            package="ros_gz_bridge", executable="parameter_bridge",
            namespace=robot_id, name="bridge", output="screen",
            parameters=[{
                "config_file": bridge_config,
                "expand_gz_topic_names": True,
                "use_sim_time": True,
            }],
        ),
        Node(
            package="ros_gz_sim", executable="create",
            namespace=robot_id, name="spawn", output="screen",
            arguments=[
                "-name", robot_id,
                "-string", Command(sdf_command),
                "-x", str(x), "-y", str(y), "-z", "0.02", "-Y", str(yaw),
            ],
        ),
    ])


def generate_launch_description():
    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    sim_share = get_package_share_directory("amr_simulation")
    edge_share = get_package_share_directory("edge_ai_nav")
    config_file = os.path.join(sim_share, "config", "warehouse_sih_demo.yaml")
    with open(config_file, "r", encoding="utf-8") as stream:
        warehouse = yaml.safe_load(stream)["warehouse"]

    world = os.path.join(sim_share, "worlds", warehouse["world_file"])
    robot_sdf = os.path.join(sim_share, "models", "warehouse_amr.sdf.xacro")
    bridge_config = os.path.join(edge_share, "config", "five_amr_bridge.yaml")
    rviz_config = os.path.join(edge_share, "config", "five_amr_demo.rviz")

    actions = [
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        ExecuteProcess(
            cmd=["gz", "sim", "-r", world], output="screen",
            condition=UnlessCondition(headless),
        ),
        ExecuteProcess(
            cmd=["gz", "sim", "-s", "-r", world], output="screen",
            condition=IfCondition(headless),
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="clock_bridge",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"], output="screen",
        ),
    ]

    for index, (robot_id, pose) in enumerate(warehouse["robot_spawns"].items()):
        colour = warehouse["robot_visuals"][robot_id]["colour"]
        actions.append(robot_actions(
            robot_id, pose, colour, robot_sdf, bridge_config, 3.0 + 0.35 * index
        ))

    actions.append(TimerAction(period=5.2, actions=[
        Node(
            package="edge_ai_nav", executable="five_amr_spawn_observer",
            name="stage2_spawn_observer", output="screen",
            parameters=[{"config_file": config_file}],
        ),
        Node(
            package="rviz2", executable="rviz2", name="stage2_rviz", output="screen",
            arguments=["-d", rviz_config], condition=IfCondition(use_rviz),
        ),
    ]))
    return LaunchDescription(actions)
