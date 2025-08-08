#!/usr/bin/env python3
"""
Optimized Pattern Generator - Better waypoint placement to prevent overshooting
Adds approach waypoints before sharp turns
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import numpy as np
import json
import math

class OptimizedPatternGenerator(Node):
    def __init__(self):
        super().__init__('optimized_pattern_generator')
        
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
        
        self.get_logger().info('Optimized Pattern Generator initialized')
    
    def update_current_pose(self, msg):
        """Update current drone position"""
        self.current_pose = msg
    
    def handle_pattern_command(self, msg):
        """Handle pattern generation commands"""
        try:
            parts = msg.data.split(':')
            pattern_type = parts[0]
            params = [float(x) for x in parts[1].split(',')] if len(parts) > 1 else []
            
            waypoints = []
            
            if pattern_type == 'square':
                size = params[0] if params else 10.0
                waypoints = self.generate_optimized_square(size)
                
            elif pattern_type == 'expanding_square':
                size = params[0] if params else 10.0
                iterations = int(params[1]) if len(params) > 1 else 3
                waypoints = self.generate_optimized_expanding_square(size, iterations)
                
            elif pattern_type == 'zigzag':
                width = params[0] if params else 20.0
                length = params[1] if len(params) > 1 else 20.0
                spacing = params[2] if len(params) > 2 else 5.0
                waypoints = self.generate_optimized_zigzag(width, length, spacing)
                
            elif pattern_type == 'spiral':
                radius = params[0] if params else 20.0
                spacing = params[1] if len(params) > 1 else 5.0
                waypoints = self.generate_smooth_spiral(radius, spacing)
            
            if waypoints:
                self.publish_waypoints(waypoints)
                self.get_logger().info(f'Generated {len(waypoints)} optimized waypoints for {pattern_type} pattern')
            
        except Exception as e:
            self.get_logger().error(f'Error parsing pattern command: {e}')
    
    def generate_optimized_square(self, size):
        """Generate single square with approach waypoints"""
        if not self.current_pose:
            return []
        
        waypoints = []
        center_x = self.current_pose.pose.position.x
        center_y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        half_size = size / 2
        approach_dist = 2.0  # Distance for approach waypoints
        
        # Define corners
        corners = [
            (center_x + half_size, center_y + half_size),  # Top-right
            (center_x + half_size, center_y - half_size),  # Bottom-right
            (center_x - half_size, center_y - half_size),  # Bottom-left
            (center_x - half_size, center_y + half_size),  # Top-left
        ]
        
        # Add first corner
        waypoints.append({'x': corners[0][0], 'y': corners[0][1], 'z': z})
        
        # Add remaining corners with approach waypoints
        for i in range(1, len(corners)):
            prev_corner = corners[i-1]
            current_corner = corners[i]
            
            # Calculate direction vector
            dx = current_corner[0] - prev_corner[0]
            dy = current_corner[1] - prev_corner[1]
            length = math.sqrt(dx**2 + dy**2)
            
            if length > 0:
                # Normalized direction
                dx_norm = dx / length
                dy_norm = dy / length
                
                # Add approach waypoint (2m before corner)
                approach_x = current_corner[0] - dx_norm * approach_dist
                approach_y = current_corner[1] - dy_norm * approach_dist
                
                # Only add approach if it's far enough from previous waypoint
                if length > approach_dist * 2:
                    waypoints.append({'x': approach_x, 'y': approach_y, 'z': z})
            
            # Add the corner
            waypoints.append({'x': current_corner[0], 'y': current_corner[1], 'z': z})
        
        # Close the square (back to first corner) with approach
        first_corner = corners[0]
        last_corner = corners[-1]
        dx = first_corner[0] - last_corner[0]
        dy = first_corner[1] - last_corner[1]
        length = math.sqrt(dx**2 + dy**2)
        
        if length > approach_dist * 2:
            dx_norm = dx / length
            dy_norm = dy / length
            approach_x = first_corner[0] - dx_norm * approach_dist
            approach_y = first_corner[1] - dy_norm * approach_dist
            waypoints.append({'x': approach_x, 'y': approach_y, 'z': z})
        
        waypoints.append({'x': first_corner[0], 'y': first_corner[1], 'z': z})
        
        return waypoints
    
    def generate_optimized_expanding_square(self, initial_size, iterations):
        """Generate expanding squares without overlap"""
        if not self.current_pose:
            return []
        
        waypoints = []
        center_x = self.current_pose.pose.position.x
        center_y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        for i in range(iterations):
            size = initial_size * (i + 1)
            half_size = size / 2
            
            # Simple corners for each square
            corners = [
                (center_x + half_size, center_y + half_size),
                (center_x + half_size, center_y - half_size),
                (center_x - half_size, center_y - half_size),
                (center_x - half_size, center_y + half_size),
            ]
            
            # Add corners
            for corner in corners:
                waypoints.append({'x': corner[0], 'y': corner[1], 'z': z})
            
            # If not last iteration, add transition to next square
            if i < iterations - 1:
                next_size = initial_size * (i + 2)
                next_half = next_size / 2
                # Transition waypoint
                waypoints.append({
                    'x': center_x + next_half,
                    'y': center_y + next_half,
                    'z': z
                })
        
        return waypoints
    
    def generate_optimized_zigzag(self, width, length, spacing):
        """Generate zigzag with better turn handling"""
        if not self.current_pose:
            return []
        
        waypoints = []
        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        num_lines = int(width / spacing) + 1
        
        for i in range(num_lines):
            y_pos = start_y + (i * spacing)
            
            if i % 2 == 0:
                # Left to right
                waypoints.append({'x': start_x, 'y': y_pos, 'z': z})
                
                # Add midpoint for long lines
                if length > 10:
                    waypoints.append({'x': start_x + length/2, 'y': y_pos, 'z': z})
                
                waypoints.append({'x': start_x + length, 'y': y_pos, 'z': z})
                
                # Add turn waypoints if not last line
                if i < num_lines - 1:
                    # Approach point before turn
                    waypoints.append({'x': start_x + length, 'y': y_pos + spacing*0.3, 'z': z})
                    # Turn point
                    waypoints.append({'x': start_x + length, 'y': y_pos + spacing*0.7, 'z': z})
                    # Exit turn
                    waypoints.append({'x': start_x + length, 'y': y_pos + spacing, 'z': z})
            else:
                # Right to left (already positioned from turn)
                # Add midpoint for long lines
                if length > 10:
                    waypoints.append({'x': start_x + length/2, 'y': y_pos, 'z': z})
                
                waypoints.append({'x': start_x, 'y': y_pos, 'z': z})
                
                # Add turn waypoints if not last line
                if i < num_lines - 1:
                    # Approach point before turn
                    waypoints.append({'x': start_x, 'y': y_pos + spacing*0.3, 'z': z})
                    # Turn point
                    waypoints.append({'x': start_x, 'y': y_pos + spacing*0.7, 'z': z})
                    # Exit turn
                    waypoints.append({'x': start_x, 'y': y_pos + spacing, 'z': z})
        
        return waypoints
    
    def generate_smooth_spiral(self, max_radius, spacing):
        """Generate spiral with more points for smoother flight"""
        if not self.current_pose:
            return []
        
        waypoints = []
        center_x = self.current_pose.pose.position.x
        center_y = self.current_pose.pose.position.y
        z = self.current_pose.pose.position.z
        
        # More points for smoother spiral
        angular_step = 0.2  # Smaller step for smoother curve
        a = spacing / (2 * np.pi)
        
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
    node = OptimizedPatternGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()