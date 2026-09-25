from setuptools import find_packages, setup

import os

from glob import glob


package_name = 'ee5112_vehicle'


setup(

    name=package_name,

    version='0.0.0',

    packages=find_packages(
        exclude=['test']
    ),


    # ==========================================================
    # PACKAGE DATA
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

            glob('config/*.yaml') + glob('config/*.json') + glob('config/*.xml')
        ),

        # ------------------------------------------------------
        # ROS occupancy maps (keep each YAML beside its image)
        # ------------------------------------------------------

        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*.yaml')
            + glob('maps/*.pgm')
            + glob('maps/*.png')
        ),


        # ------------------------------------------------------
        # Helper scripts launched from the package share folder
        # ------------------------------------------------------

        (
            os.path.join('share', package_name, 'scripts'),
            glob('scripts/*.py')
        ),

    ],


    # ==========================================================
    # PYTHON PACKAGE SETTINGS
    # ==========================================================

    install_requires=[
        'setuptools'
    ],

    zip_safe=True,


    # ==========================================================
    # PACKAGE INFORMATION
    # ==========================================================

    maintainer='EE5112 Group13',

    maintainer_email='gckgck71@gmail.com',

    description=(
        'EE5112 Ackermann vehicle simulation with '
        'RGB camera, 2D LiDAR, Gazebo control and '
        'trajectory experiments'
    ),

    license='MIT',


    # ==========================================================
    # ROS 2 EXECUTABLES
    # ==========================================================

    entry_points={

        'console_scripts': [

            # Task 2 trajectory experiment
            #
            # Run using:
            #
            # ros2 run ee5112_vehicle trajectory_experiment
            #
            # Select experiment using:
            #
            # --ros-args -p experiment:=straight
            #
            'trajectory_experiment = '
            'ee5112_vehicle.trajectory_experiment:main',

        ],

    },

)
