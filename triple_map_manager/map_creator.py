#!/usr/bin/env python3

import numpy as np
import yaml
import cv2
import os
from .obstacle_drawer import ObstacleDrawer


class MapCreator:
    """
    Creates and exports occupancy grid maps to PGM/YAML format.
    Integrates ObstacleDrawer for drawing operations.
    """
    
    def __init__(self, map_name, resolution, width, height, origin=[0.0, 0.0, 0.0]):
        """
        Initialize the map creator.
        
        Args:
            map_name (str): Name of the map
            resolution (float): Resolution in meters per cell
            width (float): Map width in meters
            height (float): Map height in meters
            origin (list): Origin coordinates [x, y, theta] in meters and radians
        """
        self.map_name = map_name
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin = origin
        
        # Calculate grid dimensions
        self.width_cells = int(width / resolution)
        self.height_cells = int(height / resolution)
        
        # Initialize grid data (0=free, 100=occupied, -1=unknown)
        self.grid_data = np.zeros((self.height_cells, self.width_cells), dtype=np.int8)
        
        # Initialize obstacle drawer
        self.obstacle_drawer = ObstacleDrawer(
            self.grid_data, resolution, width, height, origin[:2]
        )
    
    def get_obstacle_drawer(self):
        """
        Get the obstacle drawer instance for drawing operations.
        
        Returns:
            ObstacleDrawer: The obstacle drawer instance
        """
        return self.obstacle_drawer
    
    def draw_rectangle_border(self, x1, y1, x2, y2):
        """Draw a hollow rectangle border."""
        self.obstacle_drawer.draw_rectangle_border(x1, y1, x2, y2)
    
    def draw_filled_rectangle(self, x1, y1, x2, y2):
        """Draw a solid filled rectangle obstacle."""
        self.obstacle_drawer.draw_filled_rectangle(x1, y1, x2, y2)
    
    def erase_rectangle(self, x1, y1, x2, y2):
        """Erase/clear a rectangle area to free space."""
        self.obstacle_drawer.erase_rectangle(x1, y1, x2, y2)
    
    def load_obstacles_from_config(self, obstacles_config):
        """
        Load obstacles from configuration dictionary.
        Ignores obstacles with type 'recover' as they are handled by visualization only.
        
        Args:
            obstacles_config (dict): Dictionary containing obstacle definitions
        """
        if 'obstacles' not in obstacles_config:
            return
        
        for obstacle in obstacles_config['obstacles']:
            obstacle_type = obstacle.get('type', 'filled')
            coords = obstacle.get('coordinates', [])
            
            # Skip recover type obstacles - they are handled by visualization only
            if obstacle_type == 'recover':
                continue
            
            if len(coords) != 4:
                print(f"Warning: Invalid coordinates for obstacle: {coords}")
                continue
            
            x1, y1, x2, y2 = coords
            
            if obstacle_type == 'border':
                self.draw_rectangle_border(x1, y1, x2, y2)
            elif obstacle_type == 'filled':
                self.draw_filled_rectangle(x1, y1, x2, y2)
            elif obstacle_type == 'erase':
                self.erase_rectangle(x1, y1, x2, y2)
            else:
                print(f"Warning: Unknown obstacle type: {obstacle_type}")
    
    def export_map(self, filename_base):
        """
        Export map to PGM and YAML files.
        
        Args:
            filename_base (str): Base filename without extension
        """
        # Create maps directory if it doesn't exist
        maps_dir = os.path.join(os.path.dirname(__file__), '..', 'maps')
        os.makedirs(maps_dir, exist_ok=True)
        
        pgm_path = os.path.join(maps_dir, f"{filename_base}.pgm")
        yaml_path = os.path.join(maps_dir, f"{filename_base}.yaml")
        
        # Convert occupancy grid to image format
        # Occupancy values: -1=unknown (205), 0=free (254), 100=occupied (0)
        image_data = np.zeros_like(self.grid_data, dtype=np.uint8)
        image_data[self.grid_data == -1] = 205  # Unknown
        image_data[self.grid_data == 0] = 254   # Free
        image_data[self.grid_data == 100] = 0   # Occupied
        
        # Flip the image vertically (PGM format has origin at top-left)
        image_data = np.flipud(image_data)
        
        # Save PGM file
        cv2.imwrite(pgm_path, image_data)
        
        # Create YAML metadata
        yaml_data = {
            'image': f"{filename_base}.pgm",
            'mode': 'trinary',
            'resolution': self.resolution,
            'origin': self.origin,
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }
        
        # Save YAML file
        with open(yaml_path, 'w') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
        
        print(f"Map exported: {pgm_path}, {yaml_path}")
        return pgm_path, yaml_path
    
    def get_map_info(self):
        """
        Get map information for visualization.
        
        Returns:
            dict: Map information including dimensions and origin
        """
        return {
            'name': self.map_name,
            'resolution': self.resolution,
            'width': self.width,
            'height': self.height,
            'width_cells': self.width_cells,
            'height_cells': self.height_cells,
            'origin': self.origin
        }
