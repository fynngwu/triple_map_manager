#!/usr/bin/env python3

"""
Example script demonstrating map creation with obstacles.
Shows both programmatic API usage and configuration file loading.
"""

import os
import yaml
from triple_map_manager.map_creator import MapCreator


def create_maps_from_config():
    """
    Create maps using configuration files.
    """
    print("Creating maps from configuration files...")
    
    # Load maps configuration
    config_path = os.path.join(os.path.dirname(__file__), 'config', 'maps_config.yaml')
    obstacles_path = os.path.join(os.path.dirname(__file__), 'config', 'obstacles_config.yaml')
    
    with open(config_path, 'r') as f:
        maps_config = yaml.safe_load(f)
    
    with open(obstacles_path, 'r') as f:
        obstacles_config = yaml.safe_load(f)
    
    # Create each map
    for map_key, map_config in maps_config['maps'].items():
        print(f"\nCreating {map_config['name']}...")
        
        # Create map creator
        map_creator = MapCreator(
            map_name=map_config['name'],
            resolution=map_config['resolution'],
            width=map_config['width'],
            height=map_config['height'],
            origin=map_config['origin']
        )
        
        # Load obstacles from config
        obstacles_key = f"{map_key}_obstacles"
        if obstacles_key in obstacles_config:
            map_creator.load_obstacles_from_config(obstacles_config[obstacles_key])
        
        # Export map
        map_creator.export_map(map_config['name'])
        
        # Print map info
        info = map_creator.get_map_info()
        print(f"  Resolution: {info['resolution']} m/cell")
        print(f"  Size: {info['width']} x {info['height']} m")
        print(f"  Grid: {info['width_cells']} x {info['height_cells']} cells")
        print(f"  Origin: {info['origin']}")


def create_custom_map():
    """
    Create a custom map using the programmatic API.
    """
    print("\nCreating custom map using programmatic API...")
    
    # Create map creator
    map_creator = MapCreator(
        map_name="custom_demo_map",
        resolution=0.1,  # 10cm per cell
        width=25.0,      # 25 meters
        height=20.0,     # 20 meters
        origin=[0.0, 0.0, 0.0]
    )
    
    # Draw obstacles programmatically
    print("  Drawing obstacles...")
    
    # Draw some filled rectangles
    map_creator.draw_filled_rectangle(2.0, 2.0, 5.0, 4.0)
    map_creator.draw_filled_rectangle(8.0, 6.0, 12.0, 10.0)
    map_creator.draw_filled_rectangle(15.0, 3.0, 18.0, 8.0)
    
    # Draw some border rectangles
    map_creator.draw_rectangle_border(20.0, 12.0, 23.0, 16.0)
    map_creator.draw_rectangle_border(5.0, 12.0, 8.0, 18.0)
    
    # Erase a small area
    map_creator.erase_rectangle(10.0, 8.0, 11.0, 9.0)
    
    # Export map
    map_creator.export_map("custom_demo_map")
    
    # Print map info
    info = map_creator.get_map_info()
    print(f"  Resolution: {info['resolution']} m/cell")
    print(f"  Size: {info['width']} x {info['height']} m")
    print(f"  Grid: {info['width_cells']} x {info['height_cells']} cells")
    print(f"  Origin: {info['origin']}")


def main():
    """
    Main function demonstrating map creation.
    """
    print("Triple Map Manager - Example Usage")
    print("=" * 40)
    
    # Create maps from configuration
    create_maps_from_config()
    
    # Create custom map
    # create_custom_map()
    
    print("\n" + "=" * 40)
    print("Map creation completed!")
    print("You can now run the map publisher:")
    print("  ros2 launch triple_map_manager map_publisher.launch.py")
    print("\nOr start the node directly:")
    print("  ros2 run triple_map_manager map_publisher")


if __name__ == '__main__':
    main()
