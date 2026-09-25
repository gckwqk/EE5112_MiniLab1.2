# Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
"""Start the Twist converter alongside the existing arena and AMCL launches."""
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
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            share, 'config', 'cmd_vel_to_ackermann.yaml')),
        ExecuteProcess(
            cmd=[sys.executable, os.path.join(share, 'scripts', 'cmd_vel_to_ackermann.py'),
                 '--ros-args', '--params-file', LaunchConfiguration('params_file'),
                 '-p', ['use_sim_time:=', LaunchConfiguration('use_sim_time')]],
            output='screen'),
    ])
