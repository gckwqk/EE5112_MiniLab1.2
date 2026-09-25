# Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
"""Start only the new confirmation node in the existing vehicle package."""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory('ee5112_vehicle')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('target_colour', default_value='Red'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'colour_confirmation.yaml')),
        ExecuteProcess(cmd=[
            sys.executable, '-u', os.path.join(share, 'scripts', 'colour_confirmation.py'),
            '--ros-args', '--params-file', LaunchConfiguration('params_file'),
            '-p', ['use_sim_time:=', LaunchConfiguration('use_sim_time')],
            '-p', ['target_colour:=', LaunchConfiguration('target_colour')],
        ], output='screen'),
    ])
