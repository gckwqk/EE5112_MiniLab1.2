# Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
"""Task 3 mission only. Start Nav2 and the colour detector separately.

The mission also supplies the block-aware navigation map and velocity gate.
Developed with AI coding assistance.
"""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


class MissionConsoleFormat(str):
    """Highlight console milestones without adding ANSI codes to ROS messages."""

    def format(self, *, line, this):
        if 'NO_COLOR' in os.environ:
            return line
        if line.startswith(('[FOUND]', '[MISSION] status=SUCCESS')):
            return '\033[1;92m' + line + '\033[0m'
        if line.startswith('[MISSION] status=FAIL'):
            return '\033[1;91m' + line + '\033[0m'
        if line.startswith('[CMD]'):
            return '\033[1;96m' + line + '\033[0m'
        return line


def generate_launch_description():
    share = get_package_share_directory('ee5112_vehicle')
    sim = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info',
                             description='info for recording; debug for motion/planning diagnostics'),
        DeclareLaunchArgument('mission_params_file', default_value=os.path.join(
            share, 'config', 'task3_mission.yaml')),
        ExecuteProcess(cmd=[
            sys.executable, '-u', os.path.join(share, 'scripts', 'task3_mission.py'),
            '--ros-args', '--params-file', LaunchConfiguration('mission_params_file'),
            '-p', ['use_sim_time:=', sim],
            '--log-level', ['task3_mission:=', LaunchConfiguration('log_level')],
        ], output='screen', output_format=MissionConsoleFormat('{line}'),
            additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT': '{message}'}),
    ])
