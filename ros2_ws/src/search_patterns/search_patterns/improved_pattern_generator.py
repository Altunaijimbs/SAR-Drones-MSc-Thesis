#!/usr/bin/env python3
"""
Improved Pattern Generator with better waypoint placement and yaw control
Fixes overlapping squares and wide zigzag turns
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import json
import math

class ImprovedPatternGenerator(Node):
    def __init__(self):
        super().__init__('improved_pattern_generator')
        
        # QoS profile for MAVROS topics
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
        
        self.get_logger().info('Improved Pattern Generator initialized')
    
    def update_current_pose(self, msg):
        """Update current drone position"""
        self.current_pose = msg
    
    def calculate_yaw(self, from_x, from_y, to_x, to_y):
        """Calculate yaw angle to face from one point to another"""
        dx = to_x - from_x
        dy = to_y - from_y
        return math.atan2(dy, dx)
    
    def handle_pattern_command(self, msg):
        """Handle pattern generation commands"""
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
            
            elif pattern_type == 'lawnmower':
                width = params[0] if params else 30.0
                length = params[1] if len(params) > 1 else 30.0
                spacing = params[2] if len(params) > 2 else 5.0
                waypoints = self.generate_lawnmower(width, length, spacing)
            
            if waypoints:
                self.publish_waypoints(waypoints)
                self.get_logger().info(f'Generated {len(waypoints)} waypoints for {pattern_type} pattern')
            
        except Exception as e:
            self.get_logger().error(f'Error parsing pattern command: {e}')
    
    def generate_expanding_square(self, initial_size, iterations):
        """Generate expanding square pattern WITHOUT overlapping"""
        if not self.current_pose:
            return []
        
        waypoints = []
        center_x = self.current_pose.pose.position.x
        center_y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        for i in range(iterations):
            size = initial_size * (i + 1)  # Each square is bigger
            half_size = size / 2
            
            # Define corners (clockwise from top-right)
            corners = [
                (center_x + half_size, center_y + half_size),  # Top-right
                (center_x + half_size, center_y - half_size),  # Bottom-right
                (center_x - half_size, center_y - half_size),  # Bottom-left
                (center_x - half_size, center_y + half_size),  # Top-left
            ]
            
            # Add corners with yaw facing next corner
            for j in range(len(corners)):
                current = corners[j]
                next_corner = corners[(j + 1) % len(corners)]
                
                # Calculate yaw to face next corner
                yaw = self.calculate_yaw(current[0], current[1], 
                                        next_corner[0], next_corner[1])
                
                waypoints.append({
                    'x': current[0], 
                    'y': current[1], 
                    'z': z,
                    'yaw': yaw
                })
            
            # DON'T go back to start - this was causing overlap!
            # Move to starting position of next square if there is one
            if i < iterations - 1:
                next_size = initial_size * (i + 2)
                next_half = next_size / 2
                # Add transition waypoint to next square's start
                waypoints.append({
                    'x': center_x + next_half,
                    'y': center_y + next_half,
                    'z': z,
                    'yaw': 0
                })
        
        return waypoints
    
    def generate_spiral(self, max_radius, spacing):
        """Generate spiral with yaw control"""
        if not self.current_pose:
            return []
        
        waypoints = []
        center_x = self.current_pose.pose.position.x
        center_y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        # Smaller angular step for smoother spiral
        angular_step = 0.3  # radians between points
        a = spacing / (2 * np.pi)  # Spiral constant
        
        theta = 0
        prev_x = center_x
        prev_y = center_y
        
        while True:
            r = a * theta
            if r > max_radius:
                break
            
            x = center_x + r * np.cos(theta)
            y = center_y + r * np.sin(theta)
            
            # Calculate yaw to face tangent direction
            # For spiral, add 90 degrees to radial direction
            yaw = theta + np.pi/2
            
            waypoints.append({'x': x, 'y': y, 'z': z, 'yaw': yaw})
            
            prev_x = x
            prev_y = y
            theta += angular_step
        
        return waypoints
    
    def generate_zigzag(self, width, length, spacing):
        """Generate zigzag with SHARP turns using intermediate waypoints"""
        if not self.current_pose:
            return []
        
        waypoints = []
        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        num_lines = int(width / spacing) + 1
        turn_radius = 2.0  # Small turn radius for sharp turns
        
        for i in range(num_lines):
            y_pos = start_y + (i * spacing)
            
            if i % 2 == 0:
                # Left to right
                # Start of line
                waypoints.append({
                    'x': start_x, 
                    'y': y_pos, 
                    'z': z,
                    'yaw': 0  # Face east
                })
                
                # End of line
                waypoints.append({
                    'x': start_x + length, 
                    'y': y_pos, 
                    'z': z,
                    'yaw': 0  # Face east
                })
                
                # Add turn waypoint if not last line
                if i < num_lines - 1:
                    # Sharp 90-degree turn waypoint
                    waypoints.append({
                        'x': start_x + length,
                        'y': y_pos + spacing/2,  # Halfway to next line
                        'z': z,
                        'yaw': np.pi/2  # Face north during turn
                    })
                    waypoints.append({
                        'x': start_x + length,
                        'y': y_pos + spacing,  # Complete turn
                        'z': z,
                        'yaw': np.pi  # Face west for next line
                    })
            else:
                # Right to left (we're already positioned from turn)
                # Just fly straight back
                waypoints.append({
                    'x': start_x, 
                    'y': y_pos, 
                    'z': z,
                    'yaw': np.pi  # Face west
                })
                
                # Add turn waypoint if not last line
                if i < num_lines - 1:
                    # Sharp 90-degree turn waypoint
                    waypoints.append({
                        'x': start_x,
                        'y': y_pos + spacing/2,  # Halfway to next line
                        'z': z,
                        'yaw': np.pi/2  # Face north during turn
                    })
                    waypoints.append({
                        'x': start_x,
                        'y': y_pos + spacing,  # Complete turn
                        'z': z,
                        'yaw': 0  # Face east for next line
                    })
        
        return waypoints
    
    def generate_lawnmower(self, width, length, spacing):
        """Generate proper lawnmower pattern (like grid search)"""
        if not self.current_pose:
            return []
        
        waypoints = []
        start_x = self.current_pose.pose.position.x - length/2
        start_y = self.current_pose.pose.position.y - width/2
        z = self.current_pose.pose.position.z
        
        num_lines = int(width / spacing) + 1
        
        for i in range(num_lines):
            y_pos = start_y + (i * spacing)
            
            if i % 2 == 0:
                # Forward pass
                waypoints.append({
                    'x': start_x,
                    'y': y_pos,
                    'z': z,
                    'yaw': 0
                })
                waypoints.append({
                    'x': start_x + length,
                    'y': y_pos,
                    'z': z,
                    'yaw': 0
                })
            else:
                # Backward pass
                waypoints.append({
                    'x': start_x + length,
                    'y': y_pos,
                    'z': z,
                    'yaw': np.pi
                })
                waypoints.append({
                    'x': start_x,
                    'y': y_pos,
                    'z': z,
                    'yaw': np.pi
                })
        
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
    node = ImprovedPatternGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()