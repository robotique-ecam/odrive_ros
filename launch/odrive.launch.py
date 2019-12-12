import launch
import launch.actions
import launch.substitutions
import launch_ros.actions

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='odrive_ros',
            node_namespace='odrive',
            node_executable='odrive_node',
            node_name='odrive',
            output='screen',
            parameters=[{
                'wheel_track': 0.285,
                'tyre_circumference': 0.341,
                'connect_on_startup': "$(arg connect_on_startup)",
                'calibrate_on_startup': "$(arg calibrate_on_startup)",
                'engage_on_startup': "$(arg engage_on_startup)",
                'publish_odom': "$(arg publish_odom)",
                'publish_odom_tf': "$(arg publish_odom_tf)"
            }],
            remappings=None,
            arguments=[{
                'connect_on_startup': True,
                'calibrate_on_startup': True,
                'engage_on_startup': True,
                'publish_odom': True,
                'publish_odom_tf' : False
            }]
        )
    ])