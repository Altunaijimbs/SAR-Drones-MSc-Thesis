#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Point, Twist, PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from drone_interfaces.msg import SearchPattern, LLMCommand
from std_msgs.msg import String
from enum import Enum
import math
from typing import List, Tuple


class SearchState(Enum):
    IDLE = 0
    MOVING_TO_START = 1
    EXECUTING_PATTERN = 2
    PAUSED = 3
    COMPLETED = 4


class FixedGridSearchNode(Node):
    def __init__(self):
        super().__init__('fixed_grid_search_node')
        
        # Parameters
        self.declare_parameter('grid_spacing', 10.0)
        self.declare_parameter('search_speed', 2.0)
        self.declare_parameter('altitude', 20.0)
        self.declare_parameter('overlap_percentage', 20.0)
        
        # Get parameters
        self.grid_spacing = self.get_parameter('grid_spacing').value
        self.search_speed = self.get_parameter('search_speed').value
        self.altitude = self.get_parameter('altitude').value
        self.overlap = self.get_parameter('overlap_percentage').value / 100.0
        
        # Adjust spacing based on overlap
        self.effective_spacing = self.grid_spacing * (1 - self.overlap)
        
        # QoS profile to match MAVROS (best effort)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers - publish to a different topic that velocity mux will handle
        self.vel_pub = self.create_publisher(
            Twist, 
            '/search_pattern/velocity_command', 
            10
        )
        
        # Subscribers with proper QoS
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile  # Use MAVROS QoS settings
        )
        
        # Simple command interface for testing
        self.simple_cmd_sub = self.create_subscription(
            String,
            '/search_command',
            self.simple_command_callback,
            10
        )
        
        # Subscribe to RTH commands to abort search
        self.rth_cmd_sub = self.create_subscription(
            String,
            '/rth_command',
            self.rth_command_callback,
            10
        )
        
        # State variables
        self.current_pose = None
        self.search_area = None
        self.waypoints = []
        self.current_waypoint_index = 0
        self.state = SearchState.IDLE
        self.search_active = False
        
        # Control loop timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Fixed Grid Search Node initialized')
    
    def pose_callback(self, msg: PoseStamped):
        """Update current position"""
        self.current_pose = msg.pose
    
    def rth_command_callback(self, msg: String):
        """Handle RTH commands - abort search immediately"""
        cmd = msg.data.lower()
        if 'rth' in cmd or 'return' in cmd:
            self.get_logger().warn('RTH command received - aborting search immediately')
            self.abort_search()
    
    def simple_command_callback(self, msg: String):
        """Handle simple search commands"""
        cmd = msg.data.lower()
        if 'grid' in cmd or 'search' in cmd:
            self.get_logger().info(f'Starting grid search from command: {cmd}')
            self.start_simple_grid_search()
        elif 'stop' in cmd:
            self.abort_search()
        elif 'rth' in cmd or 'return' in cmd:
            # Abort search when RTH is triggered
            self.get_logger().info('RTH activated - aborting search')
            self.abort_search()
    
    def start_simple_grid_search(self):
        """Start a simple grid search centered on current position"""
        if self.current_pose is None:
            self.get_logger().warn('Cannot start search - no position data')
            return
        
        # Create 30x30m search area centered on current position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        
        corner1 = Point()
        corner1.x = current_x - 15.0
        corner1.y = current_y - 15.0
        corner1.z = 0.0
        
        corner2 = Point()
        corner2.x = current_x + 15.0
        corner2.y = current_y + 15.0
        corner2.z = 0.0
        
        # Generate waypoints
        self.waypoints = self.generate_grid_waypoints(
            corner1, corner2, 
            self.effective_spacing,
            self.altitude
        )
        
        self.current_waypoint_index = 0
        self.state = SearchState.MOVING_TO_START
        self.search_active = True
        
        self.get_logger().info(f'Starting grid search with {len(self.waypoints)} waypoints')
    
    def generate_grid_waypoints(self, corner1: Point, corner2: Point, 
                               spacing: float, altitude: float) -> List[Point]:
        """Generate waypoints for grid pattern"""
        waypoints = []
        
        # Calculate area boundaries
        min_x = min(corner1.x, corner2.x)
        max_x = max(corner1.x, corner2.x)
        min_y = min(corner1.y, corner2.y)
        max_y = max(corner1.y, corner2.y)
        
        # Generate parallel lines (lawn mower pattern)
        y_pos = min_y
        direction = 1  # 1 for right, -1 for left
        
        while y_pos <= max_y:
            if direction == 1:
                # Moving right
                start_point = Point()
                start_point.x = min_x
                start_point.y = y_pos
                start_point.z = altitude
                waypoints.append(start_point)
                
                end_point = Point()
                end_point.x = max_x
                end_point.y = y_pos
                end_point.z = altitude
                waypoints.append(end_point)
            else:
                # Moving left
                start_point = Point()
                start_point.x = max_x
                start_point.y = y_pos
                start_point.z = altitude
                waypoints.append(start_point)
                
                end_point = Point()
                end_point.x = min_x
                end_point.y = y_pos
                end_point.z = altitude
                waypoints.append(end_point)
            
            # Move to next line
            y_pos += spacing
            direction *= -1  # Reverse direction
        
        return waypoints
    
    def control_loop(self):
        """Main control loop for executing search pattern"""
        if not self.search_active or self.current_pose is None:
            return
        
        # Double-check state is valid
        if self.state == SearchState.IDLE:
            self.search_active = False
            return
        
        if self.state == SearchState.MOVING_TO_START:
            self.move_to_waypoint(self.waypoints[0])
            
        elif self.state == SearchState.EXECUTING_PATTERN:
            if self.current_waypoint_index < len(self.waypoints):
                self.move_to_waypoint(self.waypoints[self.current_waypoint_index])
            else:
                self.state = SearchState.COMPLETED
                self.search_active = False
                self.waypoints = []  # Clear waypoints to prevent resuming
                self.get_logger().info('Grid search completed')
                # Publish stop command
                stop_cmd = Twist()
                self.vel_pub.publish(stop_cmd)
    
    def move_to_waypoint(self, waypoint: Point):
        """Move drone towards waypoint"""
        if self.current_pose is None:
            return
        
        # Calculate distance to waypoint
        dx = waypoint.x - self.current_pose.position.x
        dy = waypoint.y - self.current_pose.position.y
        dz = waypoint.z - self.current_pose.position.z
        
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Check if reached waypoint
        if distance < 1.0:  # 1 meter tolerance
            self.current_waypoint_index += 1
            if self.current_waypoint_index == 1 and self.state == SearchState.MOVING_TO_START:
                self.state = SearchState.EXECUTING_PATTERN
                self.get_logger().info('Reached start position, beginning search pattern')
            elif self.current_waypoint_index < len(self.waypoints):
                self.get_logger().info(f'Waypoint {self.current_waypoint_index}/{len(self.waypoints)} reached')
            return
        
        # Calculate velocity commands
        vel_cmd = Twist()
        
        # Normalize and scale by search speed
        if distance > 0:
            vel_cmd.linear.x = (dx / distance) * self.search_speed
            vel_cmd.linear.y = (dy / distance) * self.search_speed
            vel_cmd.linear.z = (dz / distance) * min(self.search_speed, 1.0)
        
        # Transform to drone frame (ROS to UE4 coordinates)
        # FIXED: Use direct mapping like RTH (no swapping)
        transformed_vel = Twist()
        transformed_vel.linear.x = vel_cmd.linear.x   # Direct X → X
        transformed_vel.linear.y = vel_cmd.linear.y   # Direct Y → Y
        transformed_vel.linear.z = vel_cmd.linear.z   # Direct Z → Z
        
        # Publish velocity command
        self.vel_pub.publish(transformed_vel)
    
    def abort_search(self):
        """Abort the search pattern"""
        self.state = SearchState.IDLE
        self.waypoints = []
        self.current_waypoint_index = 0
        self.search_active = False
        # Stop drone
        stop_cmd = Twist()
        self.vel_pub.publish(stop_cmd)
        self.get_logger().info('Search aborted')


def main(args=None):
    rclpy.init(args=args)
    node = FixedGridSearchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()