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

    # Main verified Task 2 vehicle.
    #
    # This should be the Milestone 5 version containing:
    #
    #   - Ackermann vehicle
    #   - camera
    #   - LiDAR
    #   - ros2_control

    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'vehicle.urdf.xacro'
    )


    # ros2_control controller configuration

    controllers_file = os.path.join(
        pkg_share,
        'config',
        'controllers.yaml'
    )


    # Milestone 6 prescribed arena

    world_file = os.path.join(
        pkg_share,
        'worlds',
        'arena.world'
    )


    # ==========================================================
    # PROCESS XACRO
    # ==========================================================

    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={
            'controllers_file': controllers_file
        }
    ).toxml()


    # ----------------------------------------------------------
    # ROS 2 Humble / Gazebo Classic workaround
    #
    # Keep the same processing that was used in the working
    # gazebo_test.launch.py.
    #
    # Remove XML comments and XML declaration before supplying
    # robot_description as a ROS parameter.
    # ----------------------------------------------------------

    robot_description = re.sub(
        r'<!--.*?-->',
        '',
        robot_description_xml,
        flags=re.DOTALL
    )

    robot_description = re.sub(
        r'<\?xml.*?\?>',
        '',
        robot_description
    ).strip()

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
    # GAZEBO
    # ==========================================================

    # Start Gazebo Classic using arena.world.

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

    # Official START pose from:
    #
    # MiniLab1.2_platform_specs_5112.json
    #
    # x   = 0.55 m
    # y   = 0.35 m
    # yaw = 0 rad
    #
    # yaw = 0 means the vehicle faces +x.
    #
    # z=0.02 gives Gazebo a small amount of clearance while
    # spawning. The vehicle then settles onto the floor.

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
            '0.55',

            '-y',
            '0.35',

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
    # REAR-WHEEL DRIVE CONTROLLER
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

    # ----------------------------------------------------------
    # Sequence:
    #
    # Gazebo starts
    #       |
    #       v
    # robot is spawned
    #       |
    #       v
    # gazebo_ros2_control creates /controller_manager
    #       |
    #       v
    # joint_state_broadcaster
    #       |
    #       v
    # steering_controller
    # rear_wheel_controller
    #
    # This is the same sequencing strategy used by the verified
    # test-world launch.
    # ----------------------------------------------------------


    # Start joint_state_broadcaster after spawn_entity exits.

    start_joint_state_broadcaster = RegisterEventHandler(

        OnProcessExit(

            target_action=spawn_robot,

            on_exit=[
                joint_state_broadcaster_spawner
            ]
        )
    )


    # Once the joint state broadcaster spawner finishes,
    # start both vehicle command controllers.

    start_vehicle_controllers = RegisterEventHandler(

        OnProcessExit(

            target_action=joint_state_broadcaster_spawner,

            on_exit=[

                steering_controller_spawner,

                rear_wheel_controller_spawner

            ]
        )
    )


    # ==========================================================
    # LAUNCH DESCRIPTION
    # ==========================================================

    return LaunchDescription([

        # Start the prescribed arena.
        gazebo,

        # Publish vehicle TF / robot_description.
        robot_state_publisher,

        # Spawn vehicle at official START.
        spawn_robot,

        # Spawn joint state broadcaster after robot spawn.
        start_joint_state_broadcaster,

        # Spawn steering and rear-drive controllers afterward.
        start_vehicle_controllers,

    ])
