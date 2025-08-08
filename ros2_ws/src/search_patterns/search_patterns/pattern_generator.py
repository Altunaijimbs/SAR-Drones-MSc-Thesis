#!/usr/bin/env python3
"""
Search Pattern Generator inspired by AS2 trajectory generation
Generates smooth waypoints for various search patterns
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import String
import numpy as np
import json

class PatternGenerator(Node):
    def __init__(self):
        super().__init__('pattern_generator')
        
        # QoS profile for MAVROS topics (BEST_EFFORT reliability)
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.waypoints_pub = self.create_publisher(
            String, '/pattern_waypoints', 10
        )
        
        # Subscribers
        self.pattern_cmd_sub = self.create_subscription(
            String, '/pattern_command',
            self.handle_pattern_command, 10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.update_current_pose, 
            qos_profile=self.mavros_qos
        )
        
        # State
        self.current_pose = None
        
        self.get_logger().info('Pattern Generator initialized')
    
    def update_current_pose(self, msg):
        """Update current drone position"""
        self.current_pose = msg
    
    def handle_pattern_command(self, msg):
        """
        Handle pattern generation commands
        Format: "pattern_type:param1,param2,..."
        Examples:
        - "expanding_square:10,4" (size=10m, iterations=4)
        - "spiral:20,5" (radius=20m, spacing=5m)
        - "zigzag:30,30,5" (width=30m, length=30m, spacing=5m)
        """
        try:
            parts = msg.data.split(':')
            pattern_type = parts[0]
            params = [float(x) for x in parts[1].split(',')] if len(parts) > 1 else []
            
            waypoints = []
            
            if pattern_type == 'expanding_square':
                size = params[0] if params else 10.0
                iterations = int(params[1]) if len(params) > 1 else 4
                waypoints = self.generate_expanding_square(size, iterations)
                
            elif pattern_type == 'spiral':
                radius = params[0] if params else 20.0
                spacing = params[1] if len(params) > 1 else 5.0
                waypoints = self.generate_spiral(radius, spacing)
                
            elif pattern_type == 'zigzag':
                width = params[0] if params else 30.0
                length = params[1] if len(params) > 1 else 30.0
                spacing = params[2] if len(params) > 2 else 5.0
                waypoints = self.generate_zigzag(width, length, spacing)
            
            if waypoints:
                self.publish_waypoints(waypoints)
                self.get_logger().info(f'Generated {len(waypoints)} waypoints for {pattern_type} pattern')
            
        except Exception as e:
            self.get_logger().error(f'Error parsing pattern command: {e}')
    
    def generate_expanding_square(self, initial_size, iterations):
        """Generate expanding square pattern with smooth corners"""
        if not self.current_pose:
            return []
        
        waypoints = []
        center_x = self.current_pose.pose.position.x
        center_y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        for i in range(iterations):
            size = initial_size + (i * initial_size)
            half_size = size / 2
            
            # Simple square corners only (removed intermediate points to avoid oscillation)
            corner_points = [
                (center_x + half_size, center_y + half_size),  # Top-right
                (center_x + half_size, center_y - half_size),  # Bottom-right
                (center_x - half_size, center_y - half_size),  # Bottom-left
                (center_x - half_size, center_y + half_size),  # Top-left
                (center_x + half_size, center_y + half_size),  # Back to start
            ]
            
            for x, y in corner_points:
                waypoints.append({'x': x, 'y': y, 'z': z})
        
        return waypoints
    
    def generate_spiral(self, max_radius, spacing):
        """Generate Archimedean spiral pattern"""
        if not self.current_pose:
            return []
        
        waypoints = []
        center_x = self.current_pose.pose.position.x
        center_y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        # Parameters for spiral with fewer points
        angular_step = 0.5  # radians between points (increased to reduce waypoints)
        a = spacing / (2 * np.pi)  # Spiral constant
        
        theta = 0
        while True:
            r = a * theta
            if r > max_radius:
                break
            
            x = center_x + r * np.cos(theta)
            y = center_y + r * np.sin(theta)
            waypoints.append({'x': x, 'y': y, 'z': z})
            
            theta += angular_step
        
        return waypoints
    
    def generate_zigzag(self, width, length, spacing):
        """Generate zigzag pattern with smooth turns"""
        if not self.current_pose:
            return []
        
        waypoints = []
        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        num_lines = int(width / spacing) + 1
        
        for i in range(num_lines):
            y_offset = i * spacing
            
            if i % 2 == 0:
                # Left to right
                waypoints.append({'x': start_x, 'y': start_y + y_offset, 'z': z})
                waypoints.append({'x': start_x + length, 'y': start_y + y_offset, 'z': z})
            else:
                # Right to left
                waypoints.append({'x': start_x + length, 'y': start_y + y_offset, 'z': z})
                waypoints.append({'x': start_x, 'y': start_y + y_offset, 'z': z})
        
        return waypoints
    
    def publish_waypoints(self, waypoints):
        """Publish waypoints as JSON string"""
        msg = String()
        msg.data = json.dumps({
            'waypoints': waypoints,
            'total': len(waypoints)
        })
        self.waypoints_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PatternGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()