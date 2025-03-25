#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params = {
        'L1': 0.3,
        'L2': 0.3,
        'L3': 0.1,
        'theta1': 0.5,
        'theta2': 0.3,
        'theta3': -0.2
    }

    return LaunchDescription([
        Node(
            package='cpp_pkg',
            executable='publisher_node',
            name='publisher_node',
            parameters=[params],
            output='screen'
        ),
        Node(
            package='cpp_pkg',
            executable='subscriber_node',
            name='subscriber_node',
            output='screen'
        )
    ])
