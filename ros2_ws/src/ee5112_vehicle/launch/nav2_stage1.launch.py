"""ROS 2 Humble Nav2 only; run existing Gazebo, AMCL and converter separately.

Created with AI assistance for EE5112. Add group authorship before submission.
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
    config_dir = os.path.join(share, 'config')
    sim_time = LaunchConfiguration('use_sim_time')
    params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        root_key='',
        param_rewrites={'use_sim_time': sim_time},
        convert_types=True,
    )
    sim_parameter = ParameterValue(sim_time, value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(config_dir, 'nav2_stage1.yaml'),
        ),
        # A new leaf frame; existing base_link, sensors and AMCL stay valid.
        # Pure-pursuit's no-lateral-slip reference is the rear axle.
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='nav_rear_axle_tf', output='screen',
            arguments=[
                '--x', '-0.10', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_footprint', '--child-frame-id', 'nav_base',
            ],
            parameters=[{'use_sim_time': sim_parameter}],
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            name='planner_server', output='screen', parameters=[params],
        ),
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen', parameters=[params],
            remappings=[('cmd_vel', '/cmd_vel')],
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            name='bt_navigator', output='screen',
            parameters=[params, {
                'default_nav_to_pose_bt_xml': os.path.join(
                    config_dir, 'nav2_stage1_to_pose.xml'),
                'default_nav_through_poses_bt_xml': os.path.join(
                    config_dir, 'nav2_stage1_through_poses.xml'),
            }],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_stage1', output='screen',
            parameters=[{
                'use_sim_time': sim_parameter,
                'autostart': True,
                'bond_timeout': 4.0,
                'node_names': ['planner_server', 'controller_server', 'bt_navigator'],
            }],
        ),
    ])
