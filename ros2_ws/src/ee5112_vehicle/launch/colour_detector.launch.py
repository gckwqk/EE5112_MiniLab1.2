# Task 3 owner: Mohammad Asif Bin Abdul Sahid (A0313732M), Group 13.
"""Start the camera colour detector in the existing ee5112_vehicle package."""
import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share=get_package_share_directory('ee5112_vehicle')
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='true'),
        DeclareLaunchArgument('log_level',default_value='info'),
        # This launch argument overrides image_topic in the parameter file.
        DeclareLaunchArgument('image_topic',default_value='/camera/color/camera/image_raw'),
        DeclareLaunchArgument('params_file',default_value=os.path.join(share,'config','colour_detector.yaml')),
        ExecuteProcess(cmd=[sys.executable,os.path.join(share,'scripts','colour_detector.py'),
                            '--ros-args','--params-file',LaunchConfiguration('params_file'),
                            '-p',['use_sim_time:=',LaunchConfiguration('use_sim_time')],
                            '-p',['image_topic:=',LaunchConfiguration('image_topic')],
                            '--log-level',['colour_detector:=',LaunchConfiguration('log_level')]],output='screen'),
    ])
