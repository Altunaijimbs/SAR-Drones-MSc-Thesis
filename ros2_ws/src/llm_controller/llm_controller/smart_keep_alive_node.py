#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import time

class SmartKeepAliveNode(Node):
    """
    A smarter keep-alive node that doesn't interfere with other controllers
    Publishes to a separate topic that the velocity coordinator will handle
    """
    def __init__(self):
        super().__init__('smart_keep_alive_node')
        
        # Publish to a different topic - velocity coordinator will handle priority
        self.vel_pub = self.create_publisher(
            Twist, 
            '/keepalive/velocity_command', 
            10
        )
        
        # Timer for keep-alive at 20Hz
        self.timer = self.create_timer(0.05, self.publish_keepalive)
        
        # Default hover command
        self.keepalive_velocity = Twist()  # All zeros = hover
        
        # Simple command interface for basic control
        self.cmd_sub = self.create_subscription(
            String,
            '/simple_command',
            self.command_callback,
            10
        )
        
        self.get_logger().info('Smart Keep-Alive node started')
    
    def publish_keepalive(self):
        """Continuously publish keep-alive velocity"""
        self.vel_pub.publish(self.keepalive_velocity)
    
    def command_callback(self, msg):
        """Handle simple commands for basic drone control"""
        cmd = msg.data.lower()
        
        # For rotation commands, preserve linear velocity
        if "yaw" in cmd or "rotate" in cmd:
            # Only modify angular velocity
            if "yaw_left" in cmd or "rotate_left" in cmd:
                self.keepalive_velocity.angular.z = 0.5  # Positive yaw = counter-clockwise
            elif "yaw_right" in cmd or "rotate_right" in cmd:
                self.keepalive_velocity.angular.z = -0.5  # Negative yaw = clockwise
        else:
            # For non-rotation commands, reset the velocity
            self.keepalive_velocity = Twist()
            
            # Basic movements with UE4 coordinate system
            if "forward" in cmd:
                self.keepalive_velocity.linear.y = 0.5  # UE4 Y is forward
            elif "back" in cmd:
                self.keepalive_velocity.linear.y = -0.5
            elif "left" in cmd:
                self.keepalive_velocity.linear.x = -0.5  # UE4 X: negative is left
            elif "right" in cmd:
                self.keepalive_velocity.linear.x = 0.5   # UE4 X: positive is right
            elif "up" in cmd:
                self.keepalive_velocity.linear.z = 0.5
            elif "down" in cmd:
                self.keepalive_velocity.linear.z = -0.5
            elif "stop" in cmd or "hover" in cmd:
                # Stop all movement
                self.keepalive_velocity.linear.x = 0.0
                self.keepalive_velocity.linear.y = 0.0
                self.keepalive_velocity.angular.z = 0.0
                # Don't change Z - let it maintain current vertical velocity
            
        self.get_logger().info(f'Simple command: {cmd}')

def main():
    rclpy.init()
    node = SmartKeepAliveNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()