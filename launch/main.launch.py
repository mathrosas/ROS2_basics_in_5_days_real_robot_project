#!/usr/bin/env python3
import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        # Launch the record_odom action server node
        Node(
            package='wall_follower',
            executable='record_odom_node',
            name='record_odom'
        ),
        # Launch the find_wall service server node
        Node(
            package='wall_follower',
            executable='wall_finder_node',
            name='wall_finder'
        ),
        # Launch the wall_follower node which calls the service and action
        Node(
            package='wall_follower',
            executable='wall_follower_node',
            name='wall_follower'
        )
    ])
