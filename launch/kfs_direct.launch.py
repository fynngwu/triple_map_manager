#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    """Generate launch description for KFS system - with cleanup first."""
    
    # Cleanup process - run first to kill any existing nodes
    cleanup_process = ExecuteProcess(
        cmd=['bash', '-c', 'cd /home/wufy/ros2_ws/src/triple_map_manager && ./kill_kfs_nodes.sh && sleep 2'],
        name='cleanup_process',
        output='screen'
    )
    
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
    
    # ROSBridge websocket server with unique name
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_main',
        parameters=[{
            'port': 9090,
            'default_call_service_timeout': 5.0,
            'call_services_in_new_thread': True,
            'send_action_goals_in_new_thread': True,
        }],
        output='screen'
    )
    
    # Open HTML file with 2 second delay
    html_path = PathJoinSubstitution([
        FindPackageShare('triple_map_manager'),
        'web',
        'kfs_grid.html'
    ])
    
    open_html_process = ExecuteProcess(
        cmd=['xdg-open', html_path],
        name='open_html_process',
        output='screen'
    )
    
    # Delay HTML opening by 2 seconds
    delayed_html_process = TimerAction(
        period=2.0,
        actions=[open_html_process]
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
    )
    
    # Delay all other nodes to allow cleanup to complete
    delayed_nodes = TimerAction(
        period=5.0,
        actions=[
            map_publisher_node,
            kfs_visualizer_node,
            rosbridge_node,
            rviz_node,
            delayed_html_process,
        ]
    )
    
    return LaunchDescription([
        cleanup_process,
        delayed_nodes,
    ])
