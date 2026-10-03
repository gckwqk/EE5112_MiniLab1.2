#!/usr/bin/env python3

import os

from glob import glob

from setuptools import find_packages
from setuptools import setup


# ==============================================================
# PACKAGE
# ==============================================================

package_name = 'ee5112_vehicle'


setup(

    # ==========================================================
    # PACKAGE INFORMATION
    # ==========================================================

    name=package_name,

    version='0.0.0',

    packages=find_packages(
        exclude=[
            'test',
            'tests'
        ]
    ),


    # ==========================================================
    # DATA FILES
    # ==========================================================

    data_files=[

        # ------------------------------------------------------
        # Ament package index
        # ------------------------------------------------------

        (
            'share/ament_index/resource_index/packages',

            [
                'resource/' + package_name
            ]
        ),


        # ------------------------------------------------------
        # package.xml
        # ------------------------------------------------------

        (
            'share/' + package_name,

            [
                'package.xml'
            ]
        ),


        # ------------------------------------------------------
        # Launch files
        # ------------------------------------------------------

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),

            glob(
                'launch/*.launch.py'
            )
        ),


        # ------------------------------------------------------
        # URDF / Xacro files
        # ------------------------------------------------------

        (
            os.path.join(
                'share',
                package_name,
                'urdf'
            ),

            glob(
                'urdf/*'
            )
        ),


        # ------------------------------------------------------
        # Gazebo world files
        # ------------------------------------------------------

        (
            os.path.join(
                'share',
                package_name,
                'worlds'
            ),

            glob(
                'worlds/*'
            )
        ),


        # ------------------------------------------------------
        # Configuration files
        # ------------------------------------------------------

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),

            glob('config/*.yaml')
            + glob('config/*.json')
            + glob('config/*.xml')
        ),


        # ------------------------------------------------------
        # ROS occupancy maps
        # ------------------------------------------------------

        (
            os.path.join(
                'share',
                package_name,
                'maps'
            ),

            glob('maps/*.yaml')
            + glob('maps/*.pgm')
            + glob('maps/*.png')
        ),


        # ------------------------------------------------------
        # Helper scripts
        # ------------------------------------------------------

        (
            os.path.join(
                'share',
                package_name,
                'scripts'
            ),

            glob(
                'scripts/*.py'
            )
        ),

    ],


    # ==========================================================
    # PYTHON DEPENDENCIES
    # ==========================================================

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,


    # ==========================================================
    # MAINTAINER INFORMATION
    # ==========================================================

    maintainer='EE5112 Group13',

    maintainer_email='gckgck71@gmail.com',


    # ==========================================================
    # DESCRIPTION
    # ==========================================================

    description=(
        'EE5112 Ackermann vehicle simulation with '
        'RGB camera, 2D LiDAR, Gazebo control, '
        'trajectory experiments and live trajectory '
        'visualisation'
    ),

    license='MIT',


    # ==========================================================
    # ROS 2 EXECUTABLES
    # ==========================================================

    entry_points={

        'console_scripts': [

            # --------------------------------------------------
            # Task 2 trajectory experiment
            # --------------------------------------------------
            #
            # Example:
            #
            # ros2 run ee5112_vehicle trajectory_experiment \
            #   --ros-args \
            #   -p experiment:=gentle_left
            #

            'trajectory_experiment = '
            'ee5112_vehicle.trajectory_experiment:main',


            # --------------------------------------------------
            # Task 2 live trajectory visualisation
            # --------------------------------------------------
            #
            # Run before trajectory_experiment:
            #
            # ros2 run ee5112_vehicle live_trajectory_plot
            #

            'live_trajectory_plot = '
            'ee5112_vehicle.live_trajectory_plot:main',


            # --------------------------------------------------
            # Task 2 keyboard Ackermann teleoperation
            # --------------------------------------------------

            'ackermann_teleop = '
            'ee5112_vehicle.ackermann_teleop:main',

        ],

    },

)
