#!/usr/bin/env python3


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    odrive_prefix = get_package_share_directory('odrive_ros')
    odrive_config_dir = LaunchConfiguration('odrive_config_dir',
                                            default=os.path.join(odrive_prefix, 'config'))

    connect_on_startup = LaunchConfiguration('connect_on_startup', default=False)
    calibrate_on_startup = LaunchConfiguration('calibrate_on_startup', default=False)
    engage_on_startup = LaunchConfiguration('engage_on_startup', default=False)
    publish_odom = LaunchConfiguration('publish_odom', default=True)
    publish_odom_tf = LaunchConfiguration('publish_odom_tf', default=True)

    wheel_track = LaunchConfiguration('wheel_track', default=0.285)
    tire_circumference = LaunchConfiguration('tire_circumference', default=0.341)

    return LaunchDescription([
        DeclareLaunchArgument(
            'odrive_config_dir',
            default_value=odrive_config_dir,
            description='path to the config yaml dir'),
        DeclareLaunchArgument(
            'connect_on_startup',
            default_value=connect_on_startup,
            description='Connect to odrive right away or not'),
        DeclareLaunchArgument(
            'calibrate_on_startup',
            default_value=calibrate_on_startup,
            description='Calibrate odrive right away or not'),
        DeclareLaunchArgument(
            'engage_on_startup',
            default_value=engage_on_startup,
            description='Enable motors on odrive right away or not'),
        DeclareLaunchArgument(
            'publish_odom',
            default_value=publish_odom,
            description='Publish odom messages or not'),
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value=publish_odom_tf,
            description='Publish odom tf messages or not'),
        Node(
            name='odrive',
            package='odrive_ros',
            namespace='odrive',
            executable='odrive_node',
            output='screen',
            parameters=[{
                'wheel_track': wheel_track,
                'tyre_circumference': tire_circumference,
                'serial_number': '20713588524B',
                'connect_on_startup': connect_on_startup,
                'calibrate_on_startup': calibrate_on_startup,
                'engage_on_startup': engage_on_startup,
                'publish_odom': publish_odom,
                'publish_odom_tf': publish_odom_tf
            }],
            arguments=[],
        )
    ])
