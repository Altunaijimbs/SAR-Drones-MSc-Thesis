#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from drone_interfaces.msg import SearchPattern, LLMCommand
from enum import Enum
import math
from typing import List, Tuple


class SearchState(Enum):
    IDLE = 0
    MOVING_TO_START = 1
    EXECUTING_PATTERN = 2
    PAUSED = 3
    COMPLETED = 4


class GridSearchNode(Node):
    def __init__(self):
        super().__init__('grid_search_node')
        
        # Parameters
        self.declare_parameter('grid_spacing', 10.0)  # meters between parallel lines
        self.declare_parameter('search_speed', 2.0)  # m/s
        self.declare_parameter('altitude', 20.0)  # meters
        self.declare_parameter('overlap_percentage', 20.0)  # overlap between passes
        
        # Get parameters
        self.grid_spacing = self.get_parameter('grid_spacing').value
        self.search_speed = self.get_parameter('search_speed').value
        self.altitude = self.get_parameter('altitude').value
        self.overlap = self.get_parameter('overlap_percentage').value / 100.0
        
        # Adjust spacing based on overlap
        self.effective_spacing = self.grid_spacing * (1 - self.overlap)
        
        # Publishers
        self.vel_pub = self.create_publisher(
            Twist, 
            '/mavros/setpoint_velocity/cmd_vel_unstamped', 
            10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10
        )
        
        self.command_sub = self.create_subscription(
            LLMCommand,
            '/drone/llm_command',
            self.command_callback,
            10
        )
        
        # State variables
        self.current_pose = None
        self.search_area = None  # Will be defined by corners
        self.waypoints = []
        self.current_waypoint_index = 0
        self.state = SearchState.IDLE
        self.search_pattern = None
        
        # Control loop timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Grid Search Node initialized')
    
    def pose_callback(self, msg: PoseStamped):
        """Update current position"""
        self.current_pose = msg.pose
    
    def command_callback(self, msg: LLMCommand):
        """Handle search commands from LLM controller"""
        if msg.action == "SEARCH" and hasattr(msg, 'search_pattern'):
            if msg.search_pattern.pattern_type == "GRID":
                self.init_grid_search(msg.search_pattern)
    
    def init_grid_search(self, pattern: SearchPattern):
        """Initialize grid search with given parameters"""
        self.search_pattern = pattern
        
        # Define search area from pattern boundaries
        if len(pattern.boundaries) >= 2:
            # Assuming rectangular area defined by two corners
            corner1 = pattern.boundaries[0]
            corner2 = pattern.boundaries[1]
            
            # Generate grid waypoints
            self.waypoints = self.generate_grid_waypoints(
                corner1, corner2, 
                pattern.parameters.get('spacing', self.effective_spacing),
                pattern.parameters.get('altitude', self.altitude)
            )
            
            self.current_waypoint_index = 0
            self.state = SearchState.MOVING_TO_START
            
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
        if self.current_pose is None or self.state == SearchState.IDLE:
            return
        
        if self.state == SearchState.MOVING_TO_START:
            self.move_to_waypoint(self.waypoints[0])
            
        elif self.state == SearchState.EXECUTING_PATTERN:
            if self.current_waypoint_index < len(self.waypoints):
                self.move_to_waypoint(self.waypoints[self.current_waypoint_index])
            else:
                self.state = SearchState.COMPLETED
                self.get_logger().info('Grid search completed')
    
    def move_to_waypoint(self, waypoint: Point):
        """Move drone towards waypoint"""
        if self.current_pose is None:
            return
        
        # Calculate distance to waypoint in ROS/MAVROS frame
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
            return
        
        # Calculate velocity commands in ROS/MAVROS frame
        vel_cmd = Twist()
        
        # Normalize and scale by search speed
        if distance > 0:
            vel_cmd.linear.x = (dx / distance) * self.search_speed
            vel_cmd.linear.y = (dy / distance) * self.search_speed
            vel_cmd.linear.z = (dz / distance) * min(self.search_speed, 1.0)  # Slower vertical
        
        # Transform from ROS/MAVROS to AirSim/UE frame
        # FIXED: Corrected transformation based on actual testing
        # ROS X (forward) -> UE Y
        # ROS Y (right) -> UE X (NOT inverted)
        # ROS Z (up) -> UE Z
        transformed_vel = Twist()
        transformed_vel.linear.x = vel_cmd.linear.y   # ROS Y → UE X (no negation)
        transformed_vel.linear.y = vel_cmd.linear.x   # ROS X → UE Y
        transformed_vel.linear.z = vel_cmd.linear.z   # ROS Z → UE Z
        
        self.vel_pub.publish(transformed_vel)
    
    def pause_search(self):
        """Pause the search pattern"""
        if self.state == SearchState.EXECUTING_PATTERN:
            self.state = SearchState.PAUSED
            # Stop drone
            stop_cmd = Twist()
            self.vel_pub.publish(stop_cmd)
    
    def resume_search(self):
        """Resume the search pattern"""
        if self.state == SearchState.PAUSED:
            self.state = SearchState.EXECUTING_PATTERN
    
    def abort_search(self):
        """Abort the search pattern"""
        self.state = SearchState.IDLE
        self.waypoints = []
        self.current_waypoint_index = 0
        # Stop drone
        stop_cmd = Twist()
        self.vel_pub.publish(stop_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GridSearchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()