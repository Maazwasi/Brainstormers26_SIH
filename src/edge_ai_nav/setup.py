import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'edge_ai_nav'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name] if os.path.exists('resource/' + package_name) else []),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scenarios'), glob('scenarios/*')),
        (os.path.join('share', package_name, 'visualization', 'static'), glob('edge_ai_nav/visualization/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maaz-wasi',
    maintainer_email='maaz-wasi@todo.todo',
    description='Edge AI + GNSS / Intelligent Dead Reckoning Navigation Prototype',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'local_waypoint_controller = edge_ai_nav.ros_nodes.local_waypoint_controller:main',
            'peer_state_node = edge_ai_nav.fleet.peer_state_node:main',
            'navigation_node = edge_ai_nav.ros_nodes.navigation_node:main',
            'edge_ai_demo = edge_ai_nav.pipeline:main',
            'amr_fusion_node = edge_ai_nav.ros_nodes.edge_ai_amr_fusion_node:main',
            'warehouse_gnss_sim = edge_ai_nav.sensor_layer.warehouse_gnss_simulator:main',
            'amr_warehouse_navigator = edge_ai_nav.ros_nodes.amr_warehouse_navigator:main',
            'fleet_robot = edge_ai_nav.fleet.decentralized_coordinator:main',
            'centralized_baseline = edge_ai_nav.fleet.centralized_baseline:main',
            'fleet_visualizer = edge_ai_nav.fleet.fleet_visualizer:main',
            'five_amr_spawn_observer = edge_ai_nav.visualization.five_amr_spawn_observer:main',
            'fleet_dashboard = edge_ai_nav.visualization.fleet_dashboard_node:main',
            'fleet_route_visualizer = edge_ai_nav.visualization.fleet_route_visualizer:main',
            'task_bidder = edge_ai_nav.fleet.task_bidder_node:main',
            'slam_lifecycle_guard = edge_ai_nav.ros_nodes.slam_lifecycle_guard:main',
            'p2p_conflict_demo = edge_ai_nav.fleet.p2p_conflict_demo:main',
            'p3_obstacle_demo = edge_ai_nav.ros_nodes.p3_obstacle_demo:main',
        ],
    },
)
