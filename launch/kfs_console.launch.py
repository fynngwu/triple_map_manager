#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description for KFS system using console interface."""
    

    
    # Map publisher node with unique name
    map_publisher_node = Node(
        package='triple_map_manager',
        executable='map_publisher',
        name='map_publisher_main',
        output='screen'
    )
    
    # KFS visualizer node with unique name
    kfs_visualizer_node = Node(
        package='triple_map_manager',
        executable='kfs_visualizer',
        name='kfs_visualizer_main',
        output='screen'
    )
    
    # KFS Console Node (no rosbridge needed!)
    kfs_console_node = Node(
        package='triple_map_manager',
        executable='kfs_console',
        name='kfs_console_main',
        output='screen'
    )
    
    # RViz2 using standard Node action with FindPackageShare
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('triple_map_manager'),
        'config',
        'default.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
        environment={'QT_QPA_PLATFORM': 'xcb'} 
    )
    
    # Delay all other nodes to allow cleanup to complete
    delayed_nodes = TimerAction(
        period=1.0,
        actions=[
            map_publisher_node,
            kfs_visualizer_node,
            kfs_console_node,
            rviz_node,
        ]
    )
    
    return LaunchDescription([
        delayed_nodes,
    ])

