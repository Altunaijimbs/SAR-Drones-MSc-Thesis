#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from drone_interfaces.msg import LLMCommand, SearchPattern
from geometry_msgs.msg import Point
import time


class GridSearchTester(Node):
    def __init__(self):
        super().__init__('grid_search_tester')
        
        # Publisher for LLM commands
        self.command_pub = self.create_publisher(
            LLMCommand,
            '/drone/llm_command',
            10
        )
        
        # Wait for connections
        time.sleep(1.0)
        
        # Send test grid search command
        self.send_grid_search_command()
    
    def send_grid_search_command(self):
        """Send a test grid search command"""
        cmd = LLMCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.command_id = "test_grid_001"
        cmd.action = "SEARCH"
        
        # Create search pattern
        pattern = SearchPattern()
        pattern.pattern_type = "GRID"
        
        # Define search area (30x30 meter square)
        corner1 = Point()
        corner1.x = 0.0
        corner1.y = 0.0
        corner1.z = 0.0
        
        corner2 = Point()
        corner2.x = 30.0
        corner2.y = 30.0
        corner2.z = 0.0
        
        pattern.boundaries = [corner1, corner2]
        
        # Set parameters
        pattern.parameter_keys = ['spacing', 'overlap_percentage']
        pattern.parameter_values = [5.0, 20.0]  # 5m spacing, 20% overlap
        
        pattern.altitude = 15.0
        pattern.speed = 2.0
        pattern.return_to_start = True
        pattern.mission_id = "test_mission_001"
        pattern.priority = "HIGH"
        
        # Attach pattern to command
        cmd.search_pattern = pattern
        
        # Publish command
        self.command_pub.publish(cmd)
        self.get_logger().info('Sent grid search test command')


def main(args=None):
    rclpy.init(args=args)
    node = GridSearchTester()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()