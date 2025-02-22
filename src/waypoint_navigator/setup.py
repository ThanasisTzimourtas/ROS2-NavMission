from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'waypoint_navigator'

setup(
    name=package_name,  # Must use underscore, not hyphen
    version='0.0.1',
    packages=[package_name],  # Changed from find_packages()
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
         glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Waypoint navigation package with random path selection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigator = waypoint_navigator.waypoint_navigator:main',  # Match actual filename
            'circle_node = waypoint_navigator.circle_node:main',
            'mission_manager = waypoint_navigator.mission_manager:main',
            'server_node = waypoint_navigator.server_node:main',
            'path_selector_node = waypoint_navigator.path_selector_node:main',
        ],
    },
)