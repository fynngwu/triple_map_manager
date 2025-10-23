#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import json
import numpy as np


class KFSVisualizerNode(Node):
    """
    ROS2 node that subscribes to KFS grid data and publishes visualization markers.
    Displays KFS markers on map2's grid at 1.2m resolution.
    """
    
    def __init__(self):
        super().__init__('kfs_visualizer')
        
        # Map2 configuration
        self.map2_origin = [3.2, 1.2, 0.0]  # [x, y, theta] from maps_config.yaml
        self.grid_resolution = 1.2  # meters per cell
        self.grid_rows = 4
        self.grid_cols = 3
        
        # Create publisher for KFS markers
        self.marker_publisher = self.create_publisher(
            Marker, 
            '/map2_kfs_markers', 
            10
        )
        
        # Create subscriber for grid data
        self.grid_subscriber = self.create_subscription(
            String,
            '/kfs_grid_data',
            self.grid_callback,
            10
        )
        
        # Store current grid state
        self.current_grid = np.zeros((self.grid_rows, self.grid_cols), dtype=int)
        
        # Marker colors
        self.colors = {
            1: [0.0, 0.0, 1.0, 0.8],  # kfs1 - Blue
            2: [1.0, 0.0, 0.0, 0.8],  # kfs2 - Red
            3: [0.5, 0.5, 0.5, 0.8]   # kfs_fake - Gray
        }
        
        # Marker names
        self.marker_names = {
            1: 'kfs1',
            2: 'kfs2', 
            3: 'kfs_fake'
        }
        
        self.get_logger().info('KFS Visualizer Node started')
        self.get_logger().info(f'Map2 origin: {self.map2_origin}')
        self.get_logger().info(f'Grid resolution: {self.grid_resolution}m')
        self.get_logger().info(f'Grid size: {self.grid_rows} rows × {self.grid_cols} columns')
    
    def grid_callback(self, msg):
        """
        Callback function for receiving grid data from web interface.
        
        Args:
            msg (std_msgs.msg.String): JSON string containing grid data
        """
        try:
            # Parse JSON data
            grid_data = json.loads(msg.data)
            grid = np.array(grid_data['grid'])
            
            self.get_logger().info(f'Received grid data: {grid.tolist()}')
            
            # Validate grid dimensions
            if grid.shape != (self.grid_rows, self.grid_cols):
                self.get_logger().warn(f'Invalid grid dimensions: {grid.shape}, expected ({self.grid_rows}, {self.grid_cols})')
                return
            
            # Update current grid
            self.current_grid = grid
            
            # Publish markers
            self.publish_markers()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON data: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing grid data: {e}')
    
    def publish_markers(self):
        """
        Publish visualization markers for all KFS markers in the grid.
        """
        current_time = self.get_clock().now().to_msg()
        
        # First, delete all existing markers
        self.delete_all_markers(current_time)
        
        # Create markers for each KFS type
        marker_id = 0
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                marker_type = self.current_grid[row, col]
                
                if marker_type != 0:  # Non-empty cell
                    marker = self.create_kfs_marker(
                        marker_type, marker_id, row, col, current_time
                    )
                    self.marker_publisher.publish(marker)
                    marker_id += 1
        
        self.get_logger().info(f'Published {marker_id} KFS markers')
    
    def create_kfs_marker(self, marker_type, marker_id, row, col, timestamp):
        """
        Create a KFS marker for the specified position and type.
        
        Args:
            marker_type (int): Type of marker (1=kfs1, 2=kfs2, 3=kfs_fake)
            marker_id (int): Unique ID for the marker
            row (int): Grid row (0-3)
            col (int): Grid column (0-2)
            timestamp: ROS timestamp
            
        Returns:
            visualization_msgs.msg.Marker: KFS marker
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = timestamp
        marker.ns = "map2_kfs"
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Calculate position in map coordinates
        # Map2 origin: [3.2, 1.2, 0.0]
        # Grid cell (row, col) -> map position
        x = self.map2_origin[0] + col * self.grid_resolution
        y = self.map2_origin[1] + row * self.grid_resolution
        z = 0.0
        
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        
        # Set marker size (0.6m cubes)
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        
        # Set color based on marker type
        marker.color.r = self.colors[marker_type][0]
        marker.color.g = self.colors[marker_type][1]
        marker.color.b = self.colors[marker_type][2]
        marker.color.a = self.colors[marker_type][3]
        
        self.get_logger().debug(f'Created {self.marker_names[marker_type]} marker at ({x:.1f}, {y:.1f})')
        
        return marker
    
    def delete_all_markers(self, timestamp):
        """
        Delete all existing KFS markers by publishing DELETE actions.
        
        Args:
            timestamp: ROS timestamp
        """
        # Delete markers with IDs 0-11 (maximum possible markers in 4x3 grid)
        for marker_id in range(12):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = timestamp
            marker.ns = "map2_kfs"
            marker.id = marker_id
            marker.action = Marker.DELETE
            self.marker_publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = KFSVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
