#!/usr/bin/env python3
"""
Autonomous Position Router
Routes autonomous position commands to the hybrid position controller
CRITICAL: This was MISSING from the system!
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class AutonomousPositionRouter(Node):
    def __init__(self):
        super().__init__('autonomous_position_router')
        
        # Subscribe to autonomous position commands
        self.auto_pos_sub = self.create_subscription(
            String,
            '/autonomous/position_command',
            self.route_position,
            10
        )
        
        # Subscribe to priority mode to know when autonomous is active
        self.priority_sub = self.create_subscription(
            String,
            '/priority_mode',
            self.update_priority,
            10
        )
        
        # Publish to the hybrid position controller
        self.position_pub = self.create_publisher(
            String,
            '/position_command',
            10
        )
        
        self.current_priority = 'hybrid_balanced'
        self.get_logger().info('🚨 Autonomous Position Router initialized')
        self.get_logger().info('   This was the MISSING component!')
        self.get_logger().info('   Now routing /autonomous/position_command → /position_command')
    
    def update_priority(self, msg):
        """Track current priority mode"""
        self.current_priority = msg.data
        self.get_logger().debug(f"Priority mode: {self.current_priority}")
    
    def route_position(self, msg):
        """Route position commands when in autonomous mode"""
        # For now, always route autonomous position commands
        # The hybrid position controller will handle them
        self.position_pub.publish(msg)
        self.get_logger().info(f"📍 Routing position command: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousPositionRouter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()