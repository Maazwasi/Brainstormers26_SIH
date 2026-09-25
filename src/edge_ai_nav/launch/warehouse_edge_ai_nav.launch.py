
"""Master Integrated Launch File for Warehouse AMR Simulation + Edge AI GNSS/IDR."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():

    # ============================================================
    # Package paths
    # ============================================================

    amr_sim_share = get_package_share_directory("amr_simulation")
    edge_ai_share = get_package_share_directory("edge_ai_nav")
    nav2_bringup_share = get_package_share_directory("nav2_bringup")

    # ============================================================
    # File paths
    # ============================================================

    warehouse_world_sdf = os.path.join(
        amr_sim_share,
        "worlds",
        "warehouse.sdf"
    )

    robot_sdf_xacro = os.path.join(
        amr_sim_share,
        "models",
        "amr_waffle.sdf.xacro"
    )

    map_yaml_file = "/home/maaz-wasi/amr_ws/maps/warehouse_map.yaml"

    nav2_params_file = os.path.join(
        amr_sim_share,
        "config",
        "nav2_params.yaml"
    )

    rviz_config_file = os.path.join(
        edge_ai_share,
        "config",
        "amr_warehouse_rviz.rviz"
    )

    # ============================================================
    # Launch arguments
    # ============================================================

    use_sim_time = LaunchConfiguration(
        "use_sim_time"
    )

    use_rviz = LaunchConfiguration(
        "use_rviz"
    )

    use_nav2 = LaunchConfiguration(
        "use_nav2"
    )

    headless = LaunchConfiguration(
        "headless"
    )

    # ============================================================
    # Initial AMR position
    # ============================================================

    spawn_x = "-5.0"
    spawn_y = "3.5"
    spawn_z = "0.01"
    spawn_yaw = "0.0"

    # ============================================================
    # 1. Gazebo Harmonic
    # ============================================================

    gazebo_sim = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            warehouse_world_sdf
        ],
        output="screen",
        condition=UnlessCondition(headless),
    )

    gazebo_sim_headless = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-s",
            "-r",
            warehouse_world_sdf,
        ],
        output="screen",
        condition=IfCondition(headless),
    )

    # ============================================================
    # 2. Spawn AMR
    # ============================================================

    spawn_amr = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory(
                    "nav2_minimal_tb3_sim"
                ),
                "launch",
                "spawn_tb3.launch.py"
            )
        ),
        launch_arguments={
            "robot_name": "amr",
            "robot_sdf": robot_sdf_xacro,
            "x_pose": spawn_x,
            "y_pose": spawn_y,
            "z_pose": spawn_z,
            "yaw": spawn_yaw,
        }.items()
    )

    # ============================================================
    # 3. Robot State Publisher
    # ============================================================

    robot_description = Command(
        [
            "xacro ",
            robot_sdf_xacro
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
    )

    # ============================================================
    # 4. ROS-GZ Bridge
    #
    # Gazebo <--> ROS 2 communication
    #
    # /clock
    # /scan
    # /cmd_vel
    # /odom
    # /imu
    # /tf
    # ============================================================

    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        output="screen",
        arguments=[
    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
    "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
    "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
],
    )

    # ============================================================
    # 5. Warehouse GNSS Simulator
    # ============================================================

    node_gnss_sim = Node(
        package="edge_ai_nav",
        executable="warehouse_gnss_sim",
        name="warehouse_gnss_simulator",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "ref_lat": 28.6139,
                "ref_lon": 77.2090,
                "ref_alt": 216.0,
                "indoor_x_threshold": -1.5,
                "canopy_x_threshold": -3.5,
                "publish_rate_hz": 5.0,
            }
        ],
    )

    # ============================================================
    # 6. Edge AI + IDR Fusion
    # ============================================================

    node_edge_ai_fusion = Node(
        package="edge_ai_nav",
        executable="amr_fusion_node",
        name="edge_ai_amr_fusion_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "ref_lat": 28.6139,
                "ref_lon": 77.2090,
                "ref_alt": 216.0,
                "rate_hz": 50.0,
            }
        ],
    )

    # ============================================================
    # 7. Nav2 Localization
    # ============================================================

    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_share,
                "launch",
                "localization_launch.py"
            )
        ),
        launch_arguments={
            "map": map_yaml_file,
            "params_file": nav2_params_file,
            "use_sim_time": use_sim_time,
            "autostart": "true",
        }.items(),
        condition=IfCondition(use_nav2),
    )

    # ============================================================
    # 8. Nav2 Navigation
    # ============================================================

    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_share,
                "launch",
                "navigation_launch.py"
            )
        ),
        launch_arguments={
            "params_file": nav2_params_file,
            "use_sim_time": use_sim_time,
            "autostart": "true",
        }.items(),
        condition=IfCondition(use_nav2),
    )

    # ============================================================
    # 9. RViz2
    # ============================================================

    node_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            rviz_config_file
        ],
        parameters=[
            {
                "use_sim_time": True
            }
        ],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    # ============================================================
    # Launch description
    # ============================================================

    return LaunchDescription(

        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation time",
            ),

            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Launch RViz2",
            ),

            DeclareLaunchArgument(
                "use_nav2",
                default_value="true",
                description="Launch Nav2",
            ),

            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run Gazebo without GUI",
            ),

            gazebo_sim,

            gazebo_sim_headless,

            spawn_amr,

            node_robot_state_publisher,

            ros_gz_bridge,

            node_gnss_sim,

            node_edge_ai_fusion,

            nav2_localization,

            nav2_navigation,

            node_rviz,
        ]
    )
