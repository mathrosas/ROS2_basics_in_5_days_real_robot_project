#!/usr/bin/env python3
import launch
from launch_ros.actions import Node

def generate_launch_description():
    return launch.LaunchDescription([
        # Launch the find-wall service server node
        Node(
            package='wall_follower',
            executable='wall_finder_node',
            name='wall_finder'
        ),
        # Launch the wall following node which calls the service
        Node(
            package='wall_follower',
            executable='wall_follower_node',
            name='wall_follower_node'
        )
    ])
