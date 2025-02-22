#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # ========================
        # Core Navigation Stack
        # ========================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('clearpath_nav2_demos'),
                    'launch/localization.launch.py'
                ])
            ]),
            launch_arguments={'map': '/home/lablaptop/map/clearpath_map.yaml'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('clearpath_nav2_demos'),
                    'launch/nav2.launch.py'
                ])
            ])
        ),

        # ========================
        # Transform Publishers
        # ========================
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        # ========================
        # Twist Mux Configuration
        # ========================
        Node(
            package='twist_mux',
            executable='twist_mux',
            namespace='a200_0000',
            name='twist_mux',
            remappings=[
                ('cmd_vel_out', 'platform/cmd_vel_unstamped'),  # To platform
                ('cmd_vel_nav', 'cmd_vel_nav'),  # From nav2
                ('cmd_vel', 'cmd_vel')  # From circle node
            ],
            parameters=[{
                'use_sim_time': True,
                'twist_mux': {
                    'topics': {
                        'navigation': {
                            'topic': 'cmd_vel_nav',
                            'timeout': 0.5,
                            'priority': 50
                        },
                        'circle': {
                            'topic': 'cmd_vel',
                            'timeout': 0.5,
                            'priority': 100
                        }
                    }
                }
            }]
        ),

        # ========================
        # Status Server
        # ========================
        Node(
            package='waypoint_navigator',
            executable='server_node',
            name='robot_status_server',
            output='screen',
            parameters=[{'use_sim_time': True}]
        ),

        # ========================
        # Path Selection and Navigation
        # ========================
        LifecycleNode(
            package='waypoint_navigator',
            executable='path_selector_node',
            name='path_selector',
            namespace='a200_0000',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        ),

        LifecycleNode(
            package='waypoint_navigator',
            executable='waypoint_navigator',
            name='waypoint_navigator',
            namespace='a200_0000',
            output='screen',
            parameters=[{
                'use_sim_time': True
            }],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav')
            ]
        ),

        LifecycleNode(
            package='waypoint_navigator',
            executable='circle_node',
            name='circle_node',
            namespace='a200_0000',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'linear_velocity': 0.3,
                'angular_velocity': 0.5
            }],
            remappings=[
                ('cmd_vel', 'cmd_vel'),
                ('platform/cmd_vel_unstamped', 'platform/cmd_vel_unstamped')
            ]
        ),

        # ========================
        # Mission Control
        # ========================
        TimerAction(
            period=15.0,  # Wait time for nav stack initialization
            actions=[
                Node(
                    package='waypoint_navigator',
                    executable='mission_manager',
                    name='mission_manager',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True
                    }]
                )
            ]
        )
    ])