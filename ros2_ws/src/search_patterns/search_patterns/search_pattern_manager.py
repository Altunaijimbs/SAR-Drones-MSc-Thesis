#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from drone_interfaces.msg import LLMCommand, SearchPattern
from geometry_msgs.msg import Point
from std_msgs.msg import String
import re


class SearchPatternManager(Node):
    """
    Manages search pattern execution based on natural language commands
    Bridges between LLM controller and specific search pattern implementations
    """
    
    def __init__(self):
        super().__init__('search_pattern_manager')
        
        # Subscribe to LLM responses to parse search commands
        self.llm_response_sub = self.create_subscription(
            String,
            '/drone/llm_response',
            self.llm_response_callback,
            10
        )
        
        # Publish enhanced LLM commands with search patterns
        self.command_pub = self.create_publisher(
            LLMCommand,
            '/drone/llm_command',
            10
        )
        
        self.get_logger().info('Search Pattern Manager initialized')
    
    def llm_response_callback(self, msg: String):
        """Parse LLM responses for search-related commands"""
        response = msg.data.lower()
        
        # Check for search commands
        if 'search' in response:
            if 'grid' in response or 'area' in response:
                self.create_grid_search_command(response)
            elif 'spiral' in response:
                self.create_spiral_search_command(response)
            elif 'expanding' in response or 'square' in response:
                self.create_expanding_square_command(response)
    
    def create_grid_search_command(self, response: str):
        """Create a grid search command from natural language"""
        cmd = LLMCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.command_id = f"search_{self.get_clock().now().nanoseconds}"
        cmd.action = "SEARCH"
        
        # Create search pattern
        pattern = SearchPattern()
        pattern.pattern_type = "GRID"
        
        # Extract area size from response (default 50x50m)
        area_match = re.search(r'(\d+)\s*x\s*(\d+)', response)
        if area_match:
            width = float(area_match.group(1))
            height = float(area_match.group(2))
        else:
            width = 50.0
            height = 50.0
        
        # Define search area centered on current position
        # In real implementation, would use current drone position
        corner1 = Point(x=-width/2, y=-height/2, z=0.0)
        corner2 = Point(x=width/2, y=height/2, z=0.0)
        pattern.boundaries = [corner1, corner2]
        
        # Extract spacing if mentioned
        spacing_match = re.search(r'(\d+)\s*meter\s*spacing', response)
        if spacing_match:
            spacing = float(spacing_match.group(1))
        else:
            spacing = 10.0  # Default 10m spacing
        
        # Set parameters
        pattern.parameter_keys = ['spacing', 'overlap_percentage']
        pattern.parameter_values = [spacing, 20.0]  # 20% overlap default
        
        # Extract altitude if mentioned
        alt_match = re.search(r'(\d+)\s*meter\s*altitude', response)
        if alt_match:
            pattern.altitude = float(alt_match.group(1))
        else:
            pattern.altitude = 20.0  # Default 20m
        
        pattern.speed = 3.0  # Default search speed
        pattern.return_to_start = True
        pattern.mission_id = f"grid_search_{self.get_clock().now().nanoseconds}"
        pattern.priority = "HIGH"
        
        # Attach pattern to command
        cmd.search_pattern = pattern
        cmd.reasoning = f"Executing grid search over {width}x{height}m area with {spacing}m spacing"
        cmd.safety_priority = "HIGH"
        cmd.confidence = 0.9
        
        # Publish command
        self.command_pub.publish(cmd)
        self.get_logger().info(f'Created grid search command: {cmd.reasoning}')
    
    def create_spiral_search_command(self, response: str):
        """Create a spiral search command (placeholder for future implementation)"""
        self.get_logger().info('Spiral search not yet implemented')
    
    def create_expanding_square_command(self, response: str):
        """Create an expanding square search command (placeholder for future implementation)"""
        self.get_logger().info('Expanding square search not yet implemented')


def main(args=None):
    rclpy.init(args=args)
    node = SearchPatternManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()