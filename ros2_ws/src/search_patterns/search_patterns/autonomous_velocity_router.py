#!/usr/bin/env python3
"""
Autonomous Velocity Router
Routes autonomous velocity commands to MAVROS when in autonomous mode
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class AutonomousVelocityRouter(Node):
    def __init__(self):
        super().__init__('autonomous_velocity_router')
        
        # Subscribe to autonomous velocity commands
        self.auto_vel_sub = self.create_subscription(
            Twist,
            '/autonomous/velocity_command',
            self.route_velocity,
            10
        )
        
        # Subscribe to priority mode to know when autonomous is active
        self.priority_sub = self.create_subscription(
            String,
            '/priority_mode',
            self.update_priority,
            10
        )
        
        # Publish to MAVROS
        self.mavros_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )
        
        self.current_priority = 'hybrid_balanced'
        self.get_logger().info('Autonomous Velocity Router initialized')
    
    def update_priority(self, msg):
        """Track current priority mode"""
        self.current_priority = msg.data
    
    def route_velocity(self, msg):
        """Route velocity commands when in autonomous mode"""
        if self.current_priority == 'autonomous':
            # Forward to MAVROS
            self.mavros_pub.publish(msg)
            self.get_logger().debug(f"Routing velocity: vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}")
        else:
            self.get_logger().debug("Ignoring autonomous velocity - not in autonomous mode")

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousVelocityRouter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()