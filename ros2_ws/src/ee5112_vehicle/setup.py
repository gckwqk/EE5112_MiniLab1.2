from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'ee5112_vehicle'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(
        exclude=['test']
    ),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        (
            'share/' + package_name,
            ['package.xml']
        ),

        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'urdf'
            ),
            glob('urdf/*')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'worlds'
            ),
            glob('worlds/*')
        ),

        (
            os.path.join(
                'share',
                package_name,
                'config'
            ),
            glob('config/*')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='EE5112 Group13',
    maintainer_email='gckgck71@gmail.com',

    description='EE5112 Ackermann vehicle simulation',
    license='MIT',

    entry_points={
        'console_scripts': [
        ],
    },
)
