# Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
"""Localisation only; run the existing arena.launch.py first.

Developed with AI coding assistance.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('ee5112_vehicle')
    params = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')
    sim = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file', default_value=os.path.join(share, 'config', 'amcl.yaml')),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(share, 'maps', 'arena_map.yaml')),
        DeclareLaunchArgument(
            'publish_odom_tf', default_value='true',
            description='Disable if another node already publishes odom -> base_footprint.'),
        # Run from package share so this works with either ament_cmake or
        # ament_python when scripts/ is installed as package data.
        ExecuteProcess(
            cmd=[sys.executable, os.path.join(share, 'scripts', 'odom_to_tf.py'),
                 '--ros-args', '-p', ['use_sim_time:=', LaunchConfiguration('use_sim_time')]],
            output='screen',
            condition=IfCondition(LaunchConfiguration('publish_odom_tf'))),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[params, {'use_sim_time': sim,
                                'yaml_filename': ParameterValue(map_file, value_type=str)}]),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl',
            output='screen', parameters=[params, {'use_sim_time': sim}]),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{'use_sim_time': sim, 'autostart': True,
                         'node_names': ['map_server', 'amcl']}]),
    ])
