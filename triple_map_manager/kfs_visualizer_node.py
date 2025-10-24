#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import json
import numpy as np
import time


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
        
        # Create publisher for KFS markers using MarkerArray
        self.marker_publisher = self.create_publisher(
            MarkerArray, 
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
            self.get_logger().info(f'Received grid data: {msg.data}')
            
            # Parse JSON data
            grid_data = json.loads(msg.data)
            grid = np.array(grid_data['grid'])
            
            self.get_logger().info(f'Parsed grid shape: {grid.shape}, content: {grid}')
            
            # Validate grid dimensions
            if grid.shape != (self.grid_rows, self.grid_cols):
                self.get_logger().warn(f'Invalid grid dimensions: {grid.shape}, expected ({self.grid_rows}, {self.grid_cols})')
                return
            
            # Update current grid
            self.current_grid = grid
            
            # Publish markers
            self.get_logger().info('Publishing markers using MarkerArray...')
            self.publish_markers()
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON data: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing grid data: {e}')
    
    def publish_markers(self):
        """
        Publish visualization markers for all KFS markers in the grid using MarkerArray.
        """
        current_time = self.get_clock().now().to_msg()
        
        # First, delete all existing markers
        self.delete_all_markers(current_time)
        
        # Wait for delete operations to propagate
        time.sleep(0.1)  # 100ms delay
        
        # Create MarkerArray for efficient batch publishing
        marker_array = MarkerArray()
        marker_count = 0
        
        for row in range(self.grid_rows):
            for col in range(self.grid_cols):
                marker_type = self.current_grid[row, col]
                
                if marker_type != 0:  # Non-empty cell
                    # Use a unique ID based on grid position
                    marker_id = row * self.grid_cols + col
                    marker = self.create_kfs_marker(
                        marker_type, marker_id, row, col, current_time
                    )
                    marker_array.markers.append(marker)
                    marker_count += 1
        
        # Publish all markers at once
        if marker_count > 0:
            self.marker_publisher.publish(marker_array)
            self.get_logger().info(f'Placed {marker_count} KFS markers on map2 using MarkerArray')
    
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
        # Grid cell (col, row) -> map position
        # Add 0.5 * grid_resolution to center the marker in the cell
        x = self.map2_origin[0] + row * self.grid_resolution + 0.5 * self.grid_resolution
        y = self.map2_origin[1] + col * self.grid_resolution + 0.5 * self.grid_resolution
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
        
        self.get_logger().info(f'Placed {self.marker_names[marker_type]} at grid({row},{col}) -> map({x:.1f}, {y:.1f})')
        
        return marker
    
    def delete_all_markers(self, timestamp):
        """
        Delete all existing KFS markers by publishing DELETE actions using MarkerArray.
        
        Args:
            timestamp: ROS timestamp
        """
        # Create MarkerArray for efficient batch deletion
        delete_array = MarkerArray()
        
        # Create a general delete for the namespace
        delete_all_marker = Marker()
        delete_all_marker.header.frame_id = "map"
        delete_all_marker.header.stamp = timestamp
        delete_all_marker.ns = "map2_kfs"
        delete_all_marker.id = 0
        delete_all_marker.action = Marker.DELETEALL
        delete_array.markers.append(delete_all_marker)
        
        # Publish the delete array
        self.marker_publisher.publish(delete_array)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = KFSVisualizerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
