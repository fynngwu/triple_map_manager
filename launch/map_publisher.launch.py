#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for the triple map manager.
    Starts the map publisher node and rviz2 with default configuration.
    """
    
    # Map publisher node
    map_publisher_node = Node(
        package='triple_map_manager',
        executable='map_publisher',
        name='map_publisher',
        output='screen',
        parameters=[],
        remappings=[]
    )
    
    # RViz2 node with default configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    # KFS visualizer node
    kfs_visualizer_node = Node(
        package='triple_map_manager',
        executable='kfs_visualizer',
        name='kfs_visualizer',
        output='screen',
        parameters=[],
        remappings=[]
    )
    
    return LaunchDescription([
        map_publisher_node,
        kfs_visualizer_node,
        rviz_node
    ])
