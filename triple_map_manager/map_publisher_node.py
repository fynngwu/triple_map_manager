#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from ament_index_python.packages import get_package_share_directory
import yaml
import os
import cv2
import numpy as np
from .grid_visualizer import GridVisualizer


class MapPublisherNode(Node):
    """
    ROS2 node that publishes three OccupancyGrid maps and their grid visualizations.
    Reads PGM/YAML map files from the maps/ directory.
    """
    
    def __init__(self):
        super().__init__('map_publisher')
        
        # Load configuration
        self.load_config()
        
        # Create publishers for maps
        self.map1_pub = self.create_publisher(OccupancyGrid, '/map1', 10)
        self.map2_pub = self.create_publisher(OccupancyGrid, '/map2', 10)
        self.map3_pub = self.create_publisher(OccupancyGrid, '/map3', 10)
        
        # Create publishers for grid visualizations
        self.map1_grid_pub = self.create_publisher(Marker, '/map1_grid', 10)
        self.map2_grid_pub = self.create_publisher(Marker, '/map2_grid', 10)
        self.map3_grid_pub = self.create_publisher(Marker, '/map3_grid', 10)
        
        # Load maps
        self.maps = {}
        self.grid_visualizers = {}
        self.load_maps()
        
        # Create timer for publishing
        self.timer = self.create_timer(1.0, self.publish_maps)
        
        self.get_logger().info('Map Publisher Node started')
    
    def load_config(self):
        """Load maps configuration from YAML file."""
        # Try to find config file in installed package location
        package_share_directory = get_package_share_directory('triple_map_manager')
        config_path = os.path.join(package_share_directory, 'config', 'maps_config.yaml')
        
        # Fallback to source directory for development
        if not os.path.exists(config_path):
            config_path = os.path.join(
                os.path.dirname(__file__), '..', 'config', 'maps_config.yaml'
            )
        
        try:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            self.get_logger().info(f'Loaded config from {config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            raise
        
        # Load obstacles configuration
        self.load_obstacles_config()
    
    def load_obstacles_config(self):
        """Load obstacles configuration from YAML file."""
        # Try to find obstacles config file in installed package location
        package_share_directory = get_package_share_directory('triple_map_manager')
        obstacles_config_path = os.path.join(package_share_directory, 'config', 'obstacles_config.yaml')
        
        # Fallback to source directory for development
        if not os.path.exists(obstacles_config_path):
            obstacles_config_path = os.path.join(
                os.path.dirname(__file__), '..', 'config', 'obstacles_config.yaml'
            )
        
        try:
            with open(obstacles_config_path, 'r') as f:
                self.obstacles_config = yaml.safe_load(f)
            self.get_logger().info(f'Loaded obstacles config from {obstacles_config_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load obstacles config: {e}')
            self.obstacles_config = {}
    
    def load_maps(self):
        """Load all three maps from PGM/YAML files."""
        # Try to find maps directory in installed package location
        package_share_directory = get_package_share_directory('triple_map_manager')
        maps_dir = os.path.join(package_share_directory, 'maps')
        
        # Fallback to source directory for development
        if not os.path.exists(maps_dir):
            maps_dir = os.path.join(os.path.dirname(__file__), '..', 'maps')
        
        for i, (map_key, map_config) in enumerate(self.config['maps'].items(), 1):
            map_name = map_config['name']
            
            # Try to load existing map files
            pgm_path = os.path.join(maps_dir, f"{map_name}.pgm")
            yaml_path = os.path.join(maps_dir, f"{map_name}.yaml")
            
            if os.path.exists(pgm_path) and os.path.exists(yaml_path):
                self.get_logger().info(f'Loading existing map: {map_name}')
                occupancy_grid = self.load_map_from_files(pgm_path, yaml_path)
            else:
                self.get_logger().warn(f'Map files not found: {map_name}. Creating empty map.')
                occupancy_grid = self.create_empty_map(map_config)
            
            self.maps[f'map{i}'] = occupancy_grid
            
            # Create grid visualizer
            self.grid_visualizers[f'map{i}'] = GridVisualizer(map_config, i, self.obstacles_config)
    
    def load_map_from_files(self, pgm_path, yaml_path):
        """
        Load map from PGM and YAML files.
        
        Args:
            pgm_path (str): Path to PGM file
            yaml_path (str): Path to YAML file
            
        Returns:
            nav_msgs.msg.OccupancyGrid: Loaded occupancy grid
        """
        # Load YAML metadata
        with open(yaml_path, 'r') as f:
            yaml_data = yaml.safe_load(f)
        
        # Load PGM image
        image = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise ValueError(f"Could not load image: {pgm_path}")
        
        # Flip image vertically (PGM has origin at top-left, ROS at bottom-left)
        image = np.flipud(image)
        
        # Convert image to occupancy grid
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = "map"
        
        # Convert grayscale to occupancy values
        # PGM: 0=occupied, 254=free, 205=unknown
        # ROS: 100=occupied, 0=free, -1=unknown
        occupancy_data = np.zeros_like(image, dtype=np.int8)
        occupancy_data[image == 0] = 100      # Occupied
        occupancy_data[image == 254] = 0     # Free
        occupancy_data[image == 205] = -1     # Unknown
        
        occupancy_grid.data = occupancy_data.flatten().tolist()
        occupancy_grid.info.resolution = yaml_data['resolution']
        occupancy_grid.info.width = image.shape[1]
        occupancy_grid.info.height = image.shape[0]
        occupancy_grid.info.origin.position.x = yaml_data['origin'][0]
        occupancy_grid.info.origin.position.y = yaml_data['origin'][1]
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        
        return occupancy_grid
    
    def create_empty_map(self, map_config):
        """
        Create an empty occupancy grid map.
        
        Args:
            map_config (dict): Map configuration
            
        Returns:
            nav_msgs.msg.OccupancyGrid: Empty occupancy grid
        """
        resolution = map_config['resolution']
        width = map_config['width']
        height = map_config['height']
        origin = map_config['origin']
        
        # Calculate grid dimensions
        width_cells = int(width / resolution)
        height_cells = int(height / resolution)
        
        # Create empty occupancy grid
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header.frame_id = "map"
        occupancy_grid.data = [0] * (width_cells * height_cells)  # All free
        occupancy_grid.info.resolution = resolution
        occupancy_grid.info.width = width_cells
        occupancy_grid.info.height = height_cells
        occupancy_grid.info.origin.position.x = origin[0]
        occupancy_grid.info.origin.position.y = origin[1]
        occupancy_grid.info.origin.position.z = 0.0
        occupancy_grid.info.origin.orientation.w = 1.0
        
        return occupancy_grid
    
    def publish_maps(self):
        """Publish all maps and their grid visualizations."""
        current_time = self.get_clock().now().to_msg()
        
        # Publish maps
        publishers = [self.map1_pub, self.map2_pub, self.map3_pub]
        grid_publishers = [self.map1_grid_pub, self.map2_grid_pub, self.map3_grid_pub]
        
        for i, (map_key, publisher) in enumerate(zip(self.maps.keys(), publishers), 1):
            # Update header timestamp
            self.maps[map_key].header.stamp = current_time
            
            # Publish map
            publisher.publish(self.maps[map_key])
            
            # Publish grid visualization
            grid_visualizer = self.grid_visualizers[map_key]
            grid_marker = grid_visualizer.create_grid_marker()
            border_marker = grid_visualizer.create_border_marker()
            
            grid_marker.header.stamp = current_time
            border_marker.header.stamp = current_time
            
            grid_publishers[i-1].publish(grid_marker)
            grid_publishers[i-1].publish(border_marker)
            
            # Publish recover area visualizations
            for recover_marker in grid_visualizer.create_recover_areas_marker():
                recover_marker.header.stamp = current_time
                grid_publishers[i-1].publish(recover_marker)


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
