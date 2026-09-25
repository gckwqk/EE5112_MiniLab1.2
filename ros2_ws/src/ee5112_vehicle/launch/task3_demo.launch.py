"""Reproduce Task 3 with the Task 2 platform and no manual navigation trigger.

Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
Developed with AI coding assistance.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('ee5112_vehicle')

    def include(name, arguments=None, condition=None):
        kwargs = {'launch_arguments': (arguments or {}).items()}
        if condition is not None:
            kwargs['condition'] = condition
        # Several existing launches call their YAML argument params_file.
        # Give each include its own scope so AMCL's file cannot leak into the
        # detector or converter when starting the complete stack together.
        return GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)), **kwargs)])

    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('start_arena', default_value='true'),
        DeclareLaunchArgument('start_mission', default_value='true',
                             description='false: run task3.launch.py in a separate recording terminal'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('nav_log_level', default_value='warn'),
        include('arena.launch.py', condition=IfCondition(LaunchConfiguration('start_arena'))),
        include('amcl.launch.py', sim),
        include('cmd_vel_to_ackermann.launch.py', sim),
        include('colour_detector.launch.py', dict(sim, log_level=LaunchConfiguration('log_level'))),
        include('task3_nav2.launch.py', dict(sim, log_level=LaunchConfiguration('nav_log_level'))),
        include('task3.launch.py', dict(sim, log_level=LaunchConfiguration('log_level')),
                condition=IfCondition(LaunchConfiguration('start_mission'))),
    ])
