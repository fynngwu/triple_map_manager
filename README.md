# Triple Map Manager

A Python ROS2 package for managing three occupancy grid maps with different resolutions and obstacle drawing capabilities.

## Features

- **Map Publisher Node**: Publishes three OccupancyGrid maps simultaneously
- **Obstacle Drawing**: Draw rectangle borders, filled rectangles, and erase areas
- **Grid Visualization**: RViz markers for visualizing map grids
- **Configuration System**: YAML-based map and obstacle definitions
- **Multiple Resolutions**: Support for different map resolutions and origins

## Package Structure

```
triple_map_manager/
├── package.xml
├── setup.py
├── config/
│   ├── maps_config.yaml      # Map parameters
│   └── obstacles_config.yaml # Obstacle definitions
├── maps/
│   └── (generated PGM/YAML files)
├── launch/
│   └── map_publisher.launch.py
├── triple_map_manager/
│   ├── __init__.py
│   ├── obstacle_drawer.py     # Obstacle drawing utilities
│   ├── map_creator.py        # Map creation and export
│   ├── grid_visualizer.py    # Grid visualization
│   └── map_publisher_node.py # ROS2 node
└── example_map_creation.py   # Usage example
```

## Quick Start

### 1. Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select triple_map_manager
source install/setup.bash
```

### 2. Create Maps

```bash
cd ~/ros2_ws/src/triple_map_manager
python3 example_map_creation.py
```

This will create three maps with obstacles based on the configuration files.

### 3. Run the Map Publisher

```bash
# Using launch file
ros2 launch triple_map_manager map_publisher.launch.py

# Or directly
ros2 run triple_map_manager map_publisher
```

### 4. View in RViz

Add the following displays in RViz:
- **Map** topics: `/map1`, `/map2`, `/map3`
- **Marker** topics: `/map1_grid`, `/map2_grid`, `/map3_grid`

## Configuration

### Maps Configuration (`config/maps_config.yaml`)

```yaml
maps:
  map1:
    name: "high_resolution_map"
    resolution: 0.05  # 5cm per cell
    width: 20.0       # 20 meters
    height: 15.0      # 15 meters
    origin: [0.0, 0.0, 0.0]  # [x, y, theta]
```

### Obstacles Configuration (`config/obstacles_config.yaml`)

```yaml
map1_obstacles:
  obstacles:
    - type: "filled"
      coordinates: [2.0, 2.0, 5.0, 4.0]  # [x1, y1, x2, y2]
    - type: "border"
      coordinates: [8.0, 6.0, 12.0, 10.0]
```

## API Usage

### Map Creator

```python
from triple_map_manager.map_creator import MapCreator

# Create map
map_creator = MapCreator(
    map_name="my_map",
    resolution=0.1,  # 10cm per cell
    width=25.0,     # 25 meters
    height=20.0,    # 20 meters
    origin=[0.0, 0.0, 0.0]
)

# Draw obstacles
map_creator.draw_filled_rectangle(2.0, 2.0, 5.0, 4.0)
map_creator.draw_rectangle_border(8.0, 6.0, 12.0, 10.0)
map_creator.erase_rectangle(10.0, 8.0, 11.0, 9.0)

# Export map
map_creator.export_map("my_map")
```

### Obstacle Drawer

```python
from triple_map_manager.obstacle_drawer import ObstacleDrawer
import numpy as np

# Create grid data
grid_data = np.zeros((200, 250), dtype=np.int8)

# Create drawer
drawer = ObstacleDrawer(grid_data, 0.1, 25.0, 20.0, [0.0, 0.0])

# Draw obstacles
drawer.draw_filled_rectangle(2.0, 2.0, 5.0, 4.0)
drawer.draw_rectangle_border(8.0, 6.0, 12.0, 10.0)
drawer.erase_rectangle(10.0, 8.0, 11.0, 9.0)
```

## Published Topics

### Maps
- `/map1` (nav_msgs/OccupancyGrid): High resolution map
- `/map2` (nav_msgs/OccupancyGrid): Medium resolution map  
- `/map3` (nav_msgs/OccupancyGrid): Low resolution map

### Grid Visualizations
- `/map1_grid` (visualization_msgs/Marker): Grid lines for map1
- `/map2_grid` (visualization_msgs/Marker): Grid lines for map2
- `/map3_grid` (visualization_msgs/Marker): Grid lines for map3

## Dependencies

- `rclpy`: ROS2 Python client library
- `nav_msgs`: Navigation messages
- `visualization_msgs`: Visualization messages
- `geometry_msgs`: Geometry messages
- `python3-numpy`: Numerical computing
- `python3-opencv`: Computer vision library
- `python3-yaml`: YAML parsing

## License

TODO: License declaration
