#!/usr/bin/env python3

"""
KFS Grid Console Node - Automatic Random Placement
This node automatically generates random KFS marker placement
and publishes to ROS2 topic /kfs_grid_data.

Features:
- Automatic random marker placement on startup
- Periodic publishing to ROS2
- Perfect for WSL2 environments without GUI
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import random


class KFSConsoleNode(Node):
    """
    ROS2 node that automatically places KFS markers randomly
    and publishes grid data to ROS2.
    """
    
    def __init__(self):
        super().__init__('kfs_console_node')
        
        # Create publisher for grid data
        self.grid_publisher = self.create_publisher(
            String,
            '/kfs_grid_data',
            10
        )
        
        # Grid state
        self.grid = [[0 for _ in range(3)] for _ in range(4)]
        self.counts = {'kfs1': 0, 'kfs2': 0, 'kfs_fake': 0}
        
        self.get_logger().info('KFS Console Node started')
        
        # Auto-place markers on startup
        self.random_placement()
        self.display_grid()
        
        # Auto-publish
        self.publish_grid()
        
        self.get_logger().info('Random placement completed and published')
        
    def display_grid(self):
        """Display the grid using ASCII art"""
        print("\n" + "="*60)
        print("KFS Grid State (4 rows × 3 columns)")
        print("="*60)
        print("     Column 0   Column 1   Column 2")
        print("     " + "-"*40)
        
        for row in range(4):
            print(f"R{row} |", end=" ")
            for col in range(3):
                value = self.grid[row][col]
                if value == 1:
                    display = "[KFS1]"
                elif value == 2:
                    display = "[KFS2]"
                elif value == 3:
                    display = "[FAKE]"
                else:
                    display = "[    ]"
                print(f"{display:8}", end=" ")
            print()
        print(f"Counts: KFS1={self.counts['kfs1']}/3, KFS2={self.counts['kfs2']}/4, KFS Fake={self.counts['kfs_fake']}/1")
        print("="*60)
        
    def random_placement(self):
        """Randomly place markers with maximum counts"""
        # Clear grid
        self.grid = [[0 for _ in range(3)] for _ in range(4)]
        self.counts = {'kfs1': 0, 'kfs2': 0, 'kfs_fake': 0}
        
        # Define placement rules
        placement_rules = {
            'kfs1': {'max_count': 3, 'valid_cols': [0, 2], 'valid_rows': [0, 1, 2, 3]},
            'kfs2': {'max_count': 4, 'valid_cols': [0, 1, 2], 'valid_rows': [1, 2, 3]},
            'fake': {'max_count': 1, 'valid_cols': [0, 1, 2], 'valid_rows': [1, 2, 3]}
        }
        
        # Map to storage keys
        storage_keys = {'kfs1': 'kfs1', 'kfs2': 'kfs2', 'fake': 'kfs_fake'}
        marker_values = {'kfs1': 1, 'kfs2': 2, 'fake': 3}
        
        # Place each marker type
        for marker_type, rule in placement_rules.items():
            for _ in range(rule['max_count']):
                attempts = 0
                placed = False
                while not placed and attempts < 50:
                    row = random.choice(rule['valid_rows'])
                    col = random.choice(rule['valid_cols'])
                    
                    if self.grid[row][col] == 0:
                        self.grid[row][col] = marker_values[marker_type]
                        self.counts[storage_keys[marker_type]] += 1
                        placed = True
                    attempts += 1
        
        self.get_logger().info(
            f'Random placement: {self.counts["kfs1"]} KFS1, '
            f'{self.counts["kfs2"]} KFS2, '
            f'{self.counts["kfs_fake"]} KFS Fake'
        )
        
    def publish_grid(self):
        """Publish grid data to ROS2"""
        try:
            import time
            grid_data = {
                'grid': self.grid,
                'timestamp': int(time.time() * 1000)
            }
            
            message = String()
            message.data = json.dumps(grid_data)
            
            self.grid_publisher.publish(message)
            self.get_logger().info('Published grid data to /kfs_grid_data')
            
        except Exception as e:
            self.get_logger().error(f'Failed to publish grid data: {e}')


def main(args=None):
    """Main function"""
    # Initialize ROS2
    rclpy.init(args=args)
    
    try:
        # Create node
        node = KFSConsoleNode()
        
        # Keep node alive
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

