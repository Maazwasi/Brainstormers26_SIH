from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():

    warehouse = (
        "/home/maaz-wasi/amr_ws/"
        "install/amr_simulation/share/"
        "amr_simulation/worlds/warehouse.sdf"
    )

    robot_sdf = (
    "/home/maaz-wasi/amr_ws/src/"
    "amr_simulation/models/amr_waffle.sdf.xacro"
    )

    gazebo = ExecuteProcess(
        cmd=[
            "gz",
            "sim",
            "-r",
            warehouse
        ],
        output="screen"
    )

    spawn_robot = ExecuteProcess(
        cmd=[
            "ros2", "launch", "nav2_minimal_tb3_sim",
            "spawn_tb3.launch.py",
            "robot_name:=amr1",
            "robot_sdf:=" + robot_sdf
        ],
        output="screen"
    )

    robot_description = Command([
        "xacro",
        " ",
        robot_sdf
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True
            }
        ]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        spawn_robot,
    ])