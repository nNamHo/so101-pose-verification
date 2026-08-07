from setuptools import setup
import os
from glob import glob

package_name = 'so101_pose_milestone'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Marker file so ROS 2 can find this package in the install space
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch and config must be explicitly installed, or ros2 launch
        # will not find them after colcon build.
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jackie',
    maintainer_email='you@example.com',
    description='SO-101 task-space pose commander with independent PoE FK verification.',
    license='MIT',
    entry_points={
        'console_scripts': [
            # These names become: ros2 run so101_pose_milestone <name>
            'pose_commander = so101_pose_milestone.pose_commander:main',
            'verify_pose = so101_pose_milestone.verify_pose:main',
            'bus_check = so101_pose_milestone.bus_check:main',
        ],
    },
)
