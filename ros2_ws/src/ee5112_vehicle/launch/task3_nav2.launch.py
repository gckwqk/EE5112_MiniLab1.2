# Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
"""Task 3 Nav2: rear-axle TF, planner, controller and lifecycle.

Run arena, AMCL, converter, colour detector and task3.launch.py separately.
The mission supplies /task3/navigation_map and gates /task3/nav_cmd_vel.
It calls ComputePathToPose/FollowPath directly; no BT navigator is needed.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    share = get_package_share_directory('ee5112_vehicle')
    config = os.path.join(share, 'config')
    sim = LaunchConfiguration('use_sim_time')
    sim_param = ParameterValue(sim, value_type=bool)
    nav_params = RewrittenYaml(source_file=LaunchConfiguration('nav_params_file'),
                               root_key='', param_rewrites={'use_sim_time': sim}, convert_types=True)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('nav_params_file', default_value=os.path.join(config, 'nav2_task3.yaml')),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='nav_rear_axle_tf', output='screen',
             arguments=['--x', '-0.10', '--y', '0', '--z', '0', '--roll', '0', '--pitch', '0',
                        '--yaw', '0', '--frame-id', 'base_footprint', '--child-frame-id', 'nav_base'],
             parameters=[{'use_sim_time': sim_param}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[nav_params],
             arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]),
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[nav_params], remappings=[('cmd_vel', '/task3/nav_cmd_vel')],
             arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_task3', output='screen',
             arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')], parameters=[{
                 'use_sim_time': sim_param, 'autostart': True, 'bond_timeout': 4.0,
                 'node_names': ['planner_server', 'controller_server'],
             }]),
    ])
