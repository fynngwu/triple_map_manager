#!/usr/bin/env python3

import numpy as np


class ObstacleDrawer:
    """
    Utility class for drawing obstacles on occupancy grid maps.
    Uses numpy arrays internally for fast operations.
    Coordinates are in real-world meters, automatically converts to grid cells.
    """
    
    def __init__(self, grid_data, resolution, width, height, origin=[0.0, 0.0]):
        """
        Initialize the obstacle drawer.
        
        Args:
            grid_data (numpy.ndarray): 2D numpy array representing the occupancy grid
            resolution (float): Resolution in meters per cell
            width (float): Map width in meters
            height (float): Map height in meters
            origin (list): Origin coordinates [x, y] in meters
        """
        self.grid_data = grid_data
        self.resolution = resolution
        self.width = width
        self.height = height
        self.origin = origin
        self.height_cells, self.width_cells = grid_data.shape
        
    def _world_to_grid(self, x, y):
        """
        Convert world coordinates to grid cell coordinates.
        
        Args:
            x (float): X coordinate in meters
            y (float): Y coordinate in meters
            
        Returns:
            tuple: (grid_x, grid_y) cell coordinates
        """
        grid_x = int((x - self.origin[0]) / self.resolution)
        grid_y = int((y - self.origin[1]) / self.resolution)
        return grid_x, grid_y
    
    def _clamp_coordinates(self, x1, y1, x2, y2):
        """
        Clamp coordinates to valid grid bounds.
        
        Args:
            x1, y1, x2, y2 (int): Grid coordinates
            
        Returns:
            tuple: Clamped coordinates (x1, y1, x2, y2)
        """
        x1 = max(0, min(x1, self.width_cells - 1))
        y1 = max(0, min(y1, self.height_cells - 1))
        x2 = max(0, min(x2, self.width_cells - 1))
        y2 = max(0, min(y2, self.height_cells - 1))
        return x1, y1, x2, y2
    
    def draw_rectangle_border(self, x1, y1, x2, y2):
        """
        Draw a hollow rectangle border (only the outline).
        
        Args:
            x1 (float): Top-left X coordinate in meters
            y1 (float): Top-left Y coordinate in meters
            x2 (float): Bottom-right X coordinate in meters
            y2 (float): Bottom-right Y coordinate in meters
        """
        # Convert to grid coordinates
        gx1, gy1 = self._world_to_grid(x1, y1)
        gx2, gy2 = self._world_to_grid(x2, y2)
        
        # Ensure proper ordering
        gx1, gx2 = min(gx1, gx2), max(gx1, gx2)
        gy1, gy2 = min(gy1, gy2), max(gy1, gy2)
        
        # Clamp to grid bounds
        gx1, gy1, gx2, gy2 = self._clamp_coordinates(gx1, gy1, gx2, gy2)
        
        # Draw horizontal lines (top and bottom)
        self.grid_data[gy1, gx1:gx2+1] = 100  # Top edge
        self.grid_data[gy2, gx1:gx2+1] = 100  # Bottom edge
        
        # Draw vertical lines (left and right)
        self.grid_data[gy1:gy2+1, gx1] = 100  # Left edge
        self.grid_data[gy1:gy2+1, gx2] = 100  # Right edge
    
    def draw_filled_rectangle(self, x1, y1, x2, y2):
        """
        Draw a solid filled rectangle obstacle.
        
        Args:
            x1 (float): Top-left X coordinate in meters
            y1 (float): Top-left Y coordinate in meters
            x2 (float): Bottom-right X coordinate in meters
            y2 (float): Bottom-right Y coordinate in meters
        """
        # Convert to grid coordinates
        gx1, gy1 = self._world_to_grid(x1, y1)
        gx2, gy2 = self._world_to_grid(x2, y2)
        
        # Ensure proper ordering
        gx1, gx2 = min(gx1, gx2), max(gx1, gx2)
        gy1, gy2 = min(gy1, gy2), max(gy1, gy2)
        
        # Clamp to grid bounds
        gx1, gy1, gx2, gy2 = self._clamp_coordinates(gx1, gy1, gx2, gy2)
        
        # Fill the rectangle
        self.grid_data[gy1:gy2+1, gx1:gx2+1] = 100
    
    def erase_rectangle(self, x1, y1, x2, y2):
        """
        Erase/clear a rectangle area to free space.
        
        Args:
            x1 (float): Top-left X coordinate in meters
            y1 (float): Top-left Y coordinate in meters
            x2 (float): Bottom-right X coordinate in meters
            y2 (float): Bottom-right Y coordinate in meters
        """
        # Convert to grid coordinates
        gx1, gy1 = self._world_to_grid(x1, y1)
        gx2, gy2 = self._world_to_grid(x2, y2)
        
        # Ensure proper ordering
        gx1, gx2 = min(gx1, gx2), max(gx1, gx2)
        gy1, gy2 = min(gy1, gy2), max(gy1, gy2)
        
        # Clamp to grid bounds
        gx1, gy1, gx2, gy2 = self._clamp_coordinates(gx1, gy1, gx2, gy2)
        
        # Clear the rectangle
        self.grid_data[gy1:gy2+1, gx1:gx2+1] = 0
