from launch import LaunchDescription
from launch.actions import ExecuteProcess


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
            "ros2",
            "launch",
            "nav2_minimal_tb3_sim",
            "spawn_tb3.launch.py",
            "robot_name:=amr1",
            "robot_sdf:=" + robot_sdf
        ],
        output="screen"
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
    ])