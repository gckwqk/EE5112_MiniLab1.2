#!/usr/bin/env python3

import os
import re

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
)

from launch.event_handlers import OnProcessExit

from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    # ==========================================================
    # PACKAGE
    # ==========================================================

    package_name = 'ee5112_vehicle'

    pkg_share = get_package_share_directory(
        package_name
    )

    gazebo_ros_share = get_package_share_directory(
        'gazebo_ros'
    )


    # ==========================================================
    # FILE PATHS
    # ==========================================================

    # Main verified Task 2 vehicle:
    #
    #   - Ackermann vehicle
    #   - RGB camera
    #   - 2D LiDAR
    #   - ros2_control
    #   - ground-truth /odom

    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'vehicle.urdf.xacro'
    )


    # ros2_control configuration

    controllers_file = os.path.join(
        pkg_share,
        'config',
        'controllers.yaml'
    )


    # Open world used only for Task 2 trajectory validation

    world_file = os.path.join(
        pkg_share,
        'worlds',
        'trajectory_test.world'
    )


    # ==========================================================
    # PROCESS XACRO
    # ==========================================================

    # IMPORTANT:
    #
    # Keep the same Xacro processing method as the verified
    # arena.launch.py.
    #
    # This avoids the robot_description parameter parsing issue
    # encountered previously with Gazebo Classic / ROS 2 Humble.

    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={
            'controllers_file': controllers_file
        }
    ).toxml()


    # ==========================================================
    # ROS 2 HUMBLE / GAZEBO CLASSIC WORKAROUND
    # ==========================================================

    # Remove XML comments.

    robot_description = re.sub(
        r'<!--.*?-->',
        '',
        robot_description_xml,
        flags=re.DOTALL
    )


    # Remove XML declaration.

    robot_description = re.sub(
        r'<\?xml.*?\?>',
        '',
        robot_description
    ).strip()


    # Collapse whitespace so robot_description can safely be
    # passed as a ROS parameter to Gazebo / ros2_control.

    robot_description = ' '.join(
        robot_description.split()
    )


    # ==========================================================
    # ROBOT STATE PUBLISHER
    # ==========================================================

    robot_state_publisher = Node(

        package='robot_state_publisher',

        executable='robot_state_publisher',

        output='screen',

        parameters=[
            {
                'robot_description':
                    robot_description,

                'use_sim_time':
                    True
            }
        ]
    )


    # ==========================================================
    # GAZEBO CLASSIC
    # ==========================================================

    gazebo = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(
                gazebo_ros_share,
                'launch',
                'gazebo.launch.py'
            )
        ),

        launch_arguments={

            'world':
                world_file,

            'verbose':
                'false'

        }.items()
    )


    # ==========================================================
    # SPAWN TASK 2 VEHICLE
    # ==========================================================

    # The trajectory-validation world intentionally starts the
    # robot at the world origin:
    #
    #   x   = 0.0 m
    #   y   = 0.0 m
    #   yaw = 0.0 rad
    #
    # This makes the planned-vs-recorded trajectory plots easier
    # to interpret.
    #
    # The actual MiniLab arena continues to use the prescribed
    # START pose (0.55, 0.35, 0).

    spawn_robot = Node(

        package='gazebo_ros',

        executable='spawn_entity.py',

        output='screen',

        arguments=[

            '-topic',
            'robot_description',

            '-entity',
            'ee5112_vehicle',

            '-x',
            '0.0',

            '-y',
            '0.0',

            '-z',
            '0.02',

            '-Y',
            '0.0'

        ]
    )


    # ==========================================================
    # JOINT STATE BROADCASTER
    # ==========================================================

    joint_state_broadcaster_spawner = Node(

        package='controller_manager',

        executable='spawner',

        output='screen',

        arguments=[

            'joint_state_broadcaster',

            '--controller-manager',
            '/controller_manager',

            '--controller-manager-timeout',
            '60'

        ]
    )


    # ==========================================================
    # FRONT STEERING CONTROLLER
    # ==========================================================

    steering_controller_spawner = Node(

        package='controller_manager',

        executable='spawner',

        output='screen',

        arguments=[

            'steering_controller',

            '--controller-manager',
            '/controller_manager',

            '--controller-manager-timeout',
            '60'

        ]
    )


    # ==========================================================
    # REAR WHEEL CONTROLLER
    # ==========================================================

    rear_wheel_controller_spawner = Node(

        package='controller_manager',

        executable='spawner',

        output='screen',

        arguments=[

            'rear_wheel_controller',

            '--controller-manager',
            '/controller_manager',

            '--controller-manager-timeout',
            '60'

        ]
    )


    # ==========================================================
    # CONTROLLER STARTUP SEQUENCE
    # ==========================================================

    # Do NOT start the controllers using a fixed TimerAction.
    #
    # Wait until spawn_entity.py exits successfully. At this
    # point the vehicle has been inserted into Gazebo and the
    # gazebo_ros2_control plugin can create controller_manager.

    start_joint_state_broadcaster = RegisterEventHandler(

        OnProcessExit(

            target_action=spawn_robot,

            on_exit=[
                joint_state_broadcaster_spawner
            ]

        )

    )


    # ==========================================================
    # START STEERING CONTROLLER AFTER JSB
    # ==========================================================

    start_steering_controller = RegisterEventHandler(

        OnProcessExit(

            target_action=
                joint_state_broadcaster_spawner,

            on_exit=[
                steering_controller_spawner
            ]

        )

    )


    # ==========================================================
    # START REAR WHEEL CONTROLLER AFTER STEERING
    # ==========================================================

    start_rear_wheel_controller = RegisterEventHandler(

        OnProcessExit(

            target_action=
                steering_controller_spawner,

            on_exit=[
                rear_wheel_controller_spawner
            ]

        )

    )


    # ==========================================================
    # LAUNCH DESCRIPTION
    # ==========================================================

    return LaunchDescription([

        # Start Gazebo with open trajectory world
        gazebo,

        # Publish URDF / TF
        robot_state_publisher,

        # Spawn vehicle
        spawn_robot,

        # Sequential controller startup
        start_joint_state_broadcaster,
        start_steering_controller,
        start_rear_wheel_controller,

    ])
