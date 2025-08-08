#!/usr/bin/env python3
"""
Optional LLM to Pattern Bridge
This is a separate node that bridges LLM commands to pattern system
Can be added to the system without modifying existing nodes
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PatternLLMBridge(Node):
    def __init__(self):
        super().__init__('pattern_llm_bridge')
        
        # Publishers
        self.pattern_cmd_pub = self.create_publisher(String, '/pattern_command', 10)
        self.pattern_control_pub = self.create_publisher(String, '/pattern_control', 10)
        
        # Subscriber - listens to a new topic so it doesn't interfere
        self.llm_pattern_sub = self.create_subscription(
            String, '/llm/pattern_request',
            self.handle_llm_pattern_request, 10
        )
        
        self.get_logger().info('Pattern LLM Bridge initialized (optional component)')
    
    def handle_llm_pattern_request(self, msg):
        """
        Handle pattern requests from LLM
        Examples:
        - "search:expanding_square" -> Generate and start expanding square
        - "search:spiral" -> Generate and start spiral
        - "stop_pattern" -> Stop current pattern
        """
        command = msg.data.lower()
        
        if command.startswith('search:'):
            pattern_type = command.split(':')[1]
            
            # Generate pattern with default parameters
            if pattern_type == 'expanding_square':
                self.pattern_cmd_pub.publish(String(data='expanding_square:15,4'))
            elif pattern_type == 'spiral':
                self.pattern_cmd_pub.publish(String(data='spiral:25,5'))
            elif pattern_type == 'zigzag':
                self.pattern_cmd_pub.publish(String(data='zigzag:30,30,5'))
            
            # Auto-start after generation
            self.get_logger().info(f'Generating {pattern_type} pattern')
            rclpy.spin_once(self, timeout_sec=0.5)  # Small delay
            self.pattern_control_pub.publish(String(data='start'))
            
        elif command == 'stop_pattern':
            self.pattern_control_pub.publish(String(data='stop'))
            
        elif command == 'pause_pattern':
            self.pattern_control_pub.publish(String(data='pause'))
            
        elif command == 'resume_pattern':
            self.pattern_control_pub.publish(String(data='resume'))


def main(args=None):
    rclpy.init(args=args)
    node = PatternLLMBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()