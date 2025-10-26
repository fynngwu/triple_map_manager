#!/usr/bin/env python3

import numpy as np
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class GridVisualizer:
    """
    Python class for visualizing map grids using RViz markers.
    Reads parameters from maps_config.yaml and generates visualization_msgs/Marker messages.
    """
    
    def __init__(self, map_info, map_id, obstacles_config=None):
        """
        Initialize the grid visualizer.
        
        Args:
            map_info (dict): Map information from config
            map_id (int): Map identifier (1, 2, or 3)
            obstacles_config (dict): Obstacles configuration dictionary
        """
        self.map_info = map_info
        self.map_id = map_id
        self.resolution = map_info['resolution']
        self.width = map_info['width']
        self.height = map_info['height']
        self.origin = map_info['origin']
        self.obstacles_config = obstacles_config or {}
        
        # Define colors for different maps
        self.colors = {
            1: [0.0, 0.0, 1.0, 0.8],  # Blue for map1
            2: [0.0, 1.0, 0.0, 0.8],  # Green for map2
            3: [0.0, 0.0, 1.0, 0.8]   # Blue for map3
        }
        
        # Grid line spacing (every 1 meter)
        self.grid_spacing = map_info['resolution']
        
    
    def create_grid_marker(self, frame_id="map"):
        """
        Create a grid marker for visualization.
        
        Args:
            frame_id (str): Frame ID for the marker
            
        Returns:
            visualization_msgs.msg.Marker: Grid marker message
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = f"map{self.map_id}_grid"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.01  # Line width
        marker.color.r = self.colors[self.map_id][0]
        marker.color.g = self.colors[self.map_id][1]
        marker.color.b = self.colors[self.map_id][2]
        marker.color.a = self.colors[self.map_id][3]
        
        # Generate grid lines
        self._add_grid_lines(marker)
        
        return marker
    
    def create_border_marker(self, frame_id="map"):
        """
        Create a border marker for the map boundary.
        
        Args:
            frame_id (str): Frame ID for the marker
            
        Returns:
            visualization_msgs.msg.Marker: Border marker message
        """
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.ns = f"map{self.map_id}_border"
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05  # Thicker line for border
        marker.color.r = self.colors[self.map_id][0] * 0.8
        marker.color.g = self.colors[self.map_id][1] * 0.8
        marker.color.b = self.colors[self.map_id][2] * 0.8
        marker.color.a = self.colors[self.map_id][3]
        
        # Create border rectangle
        origin_x, origin_y = self.origin[0], self.origin[1]
        
        # Border points (clockwise from bottom-left)
        border_points = [
            [origin_x, origin_y, 0.0],                           # Bottom-left
            [origin_x + self.width, origin_y, 0.0],            # Bottom-right
            [origin_x + self.width, origin_y + self.height, 0.0], # Top-right
            [origin_x, origin_y + self.height, 0.0],           # Top-left
            [origin_x, origin_y, 0.0]                           # Back to start
        ]
        
        for point in border_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]
            marker.points.append(p)
        
        return marker
    
    def _add_grid_lines(self, marker):
        """
        Add grid lines to the marker.
        
        Args:
            marker: Marker object to add points to
        """
        origin_x, origin_y = self.origin[0], self.origin[1]
        
        # Vertical lines
        x = origin_x
        while x <= origin_x + self.width:
            # Bottom point
            p1 = Point()
            p1.x = x
            p1.y = origin_y
            p1.z = 0.0
            marker.points.append(p1)
            
            # Top point
            p2 = Point()
            p2.x = x
            p2.y = origin_y + self.height
            p2.z = 0.0
            marker.points.append(p2)
            
            x += self.grid_spacing
        
        # Horizontal lines
        y = origin_y
        while y <= origin_y + self.height:
            # Left point
            p1 = Point()
            p1.x = origin_x
            p1.y = y
            p1.z = 0.0
            marker.points.append(p1)
            
            # Right point
            p2 = Point()
            p2.x = origin_x + self.width
            p2.y = y
            p2.z = 0.0
            marker.points.append(p2)
            
            y += self.grid_spacing
    
    def create_recover_areas_marker(self, frame_id="map"):
        """
        Create markers for recover areas (red filled rectangles).
        
        Args:
            frame_id (str): Frame ID for the marker
            
        Returns:
            list: List of Marker messages for recover areas
        """
        markers = []
        map_key = f"map{self.map_id}_obstacles"
        
        if map_key not in self.obstacles_config:
            return markers
        
        map_obstacles_config = self.obstacles_config[map_key]
        if 'obstacles' not in map_obstacles_config:
            return markers
        
        # Handle case where obstacles might be None instead of empty list
        obstacles_list = map_obstacles_config['obstacles']
        if obstacles_list is None:
            return markers
        
        marker_id = 0
        for obstacle in obstacles_list:
            obstacle_type = obstacle.get('type', '')
            coords = obstacle.get('coordinates', [])
            
            if obstacle_type == 'recover' and len(coords) == 4:
                marker = Marker()
                marker.header.frame_id = frame_id
                marker.ns = f"map{self.map_id}_recover"
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose.orientation.w = 1.0
                
                x1, y1, x2, y2 = coords
                
                # Calculate center position (considering origin offset)
                center_x = (x1 + x2) / 2.0 + self.origin[0]
                center_y = (y1 + y2) / 2.0 + self.origin[1]
                center_z = 0.0
                
                marker.pose.position.x = center_x
                marker.pose.position.y = center_y
                marker.pose.position.z = center_z
                
                # Calculate dimensions
                marker.scale.x = abs(x2 - x1)
                marker.scale.y = abs(y2 - y1)
                marker.scale.z = 0.1  # Small height for visibility
                
                # Set red color
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                
                markers.append(marker)
                marker_id += 1
        
        return markers