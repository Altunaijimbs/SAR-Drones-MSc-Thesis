#!/usr/bin/env python3
"""
Smooth Position Controller with overshoot prevention
Implements damping and position hold when target is reached
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from mavros_msgs.msg import State
import math
import json

class SmoothPositionController(Node):
    def __init__(self):
        super().__init__('smooth_position_controller')
        
        # Define QoS profile for MAVROS topics
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.setpoint_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.update_current_pose, 
            qos_profile=self.mavros_qos
        )
        
        self.position_cmd_sub = self.create_subscription(
            String, '/position_command',
            self.handle_position_command, 10
        )
        
        self.state_sub = self.create_subscription(
            State, '/mavros/state',
            self.state_callback, 
            qos_profile=self.mavros_qos
        )
        
        # State
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.published_setpoint = PoseStamped()  # What we're actually sending
        self.is_armed = False
        self.mode = ""
        self.position_reached = True
        
        # Control parameters
        self.position_threshold = 0.5  # meters - when to consider target reached
        self.hold_threshold = 1.0      # meters - when to switch to position hold
        self.damping_distance = 3.0    # meters - start slowing down
        self.min_speed_factor = 0.2    # Minimum speed as fraction of full speed
        
        # Position hold state
        self.holding_position = False
        self.hold_position = None
        
        # Timer for publishing setpoints at 20Hz
        self.setpoint_timer = self.create_timer(0.05, self.publish_setpoint)
        
        self.get_logger().info('Smooth Position Controller initialized')
    
    def state_callback(self, msg):
        """Update drone state"""
        self.is_armed = msg.armed
        self.mode = msg.mode
    
    def update_current_pose(self, msg):
        """Update current drone position"""
        self.current_pose = msg
        
        # Check if we've reached target position
        if not self.position_reached:
            distance = self.calculate_distance_to_target()
            
            if distance < self.position_threshold:
                self.position_reached = True
                self.holding_position = True
                self.hold_position = self.current_pose.pose.position
                self.get_logger().info(f'✓ Position reached! Holding at current position')
            elif distance < self.hold_threshold and self.holding_position:
                # Update hold position to prevent drift
                self.hold_position = self.current_pose.pose.position
    
    def calculate_distance_to_target(self):
        """Calculate 3D distance to target position"""
        dx = self.target_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.target_pose.pose.position.y - self.current_pose.pose.position.y
        dz = self.target_pose.pose.position.z - self.current_pose.pose.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def handle_position_command(self, msg):
        """Handle position commands with same format as before"""
        try:
            cmd = msg.data
            self.get_logger().info(f'Received position command: {cmd}')
            
            # Reset hold state
            self.holding_position = False
            self.hold_position = None
            
            # Try to parse as JSON first
            try:
                data = json.loads(cmd)
                if 'x' in data and 'y' in data and 'z' in data:
                    self.set_absolute_position(data['x'], data['y'], data['z'])
                    return
            except json.JSONDecodeError:
                pass
            
            # Parse simple commands
            if ':' in cmd:
                command, value = cmd.split(':', 1)
                command = command.lower()
                
                if command == 'goto':
                    # Absolute position
                    coords = value.split(',')
                    if len(coords) >= 3:
                        x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
                        self.set_absolute_position(x, y, z)
                else:
                    # Relative movement
                    distance = float(value)
                    self.move_relative(command, distance)
            
            elif cmd.lower() == 'hover':
                # Hold current position
                self.holding_position = True
                self.hold_position = self.current_pose.pose.position
                self.target_pose = self.current_pose
                self.position_reached = True
                self.get_logger().info('Hovering at current position')
            
        except Exception as e:
            self.get_logger().error(f'Error parsing position command: {e}')
    
    def move_relative(self, direction, distance):
        """Move relative to current position"""
        # Copy current pose as starting point
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        self.target_pose.pose = self.current_pose.pose
        
        # Get current yaw angle to handle forward/backward correctly
        yaw = self.get_yaw_from_quaternion(self.current_pose.pose.orientation)
        
        # Apply movement based on direction
        if direction == 'forward':
            self.target_pose.pose.position.x += distance * math.cos(yaw)
            self.target_pose.pose.position.y += distance * math.sin(yaw)
        elif direction == 'backward' or direction == 'back':
            self.target_pose.pose.position.x -= distance * math.cos(yaw)
            self.target_pose.pose.position.y -= distance * math.sin(yaw)
        elif direction == 'left':
            self.target_pose.pose.position.x += distance * math.cos(yaw + math.pi/2)
            self.target_pose.pose.position.y += distance * math.sin(yaw + math.pi/2)
        elif direction == 'right':
            self.target_pose.pose.position.x += distance * math.cos(yaw - math.pi/2)
            self.target_pose.pose.position.y += distance * math.sin(yaw - math.pi/2)
        elif direction == 'up':
            self.target_pose.pose.position.z += distance
        elif direction == 'down':
            self.target_pose.pose.position.z -= distance
        else:
            self.get_logger().warn(f'Unknown direction: {direction}')
            return
        
        self.position_reached = False
        self.get_logger().info(
            f'Moving {direction} {distance}m to position: '
            f'({self.target_pose.pose.position.x:.2f}, '
            f'{self.target_pose.pose.position.y:.2f}, '
            f'{self.target_pose.pose.position.z:.2f})'
        )
    
    def set_absolute_position(self, x, y, z):
        """Set absolute target position"""
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z
        
        # Keep current orientation
        self.target_pose.pose.orientation = self.current_pose.pose.orientation
        
        self.position_reached = False
        self.get_logger().info(f'Setting absolute position: ({x:.2f}, {y:.2f}, {z:.2f})')
    
    def get_yaw_from_quaternion(self, quaternion):
        """Extract yaw angle from quaternion"""
        q = quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def calculate_smooth_setpoint(self):
        """Calculate smoothed setpoint to prevent overshoot"""
        if self.holding_position and self.hold_position:
            # When holding, use the saved hold position
            self.published_setpoint = PoseStamped()
            self.published_setpoint.header.frame_id = "map"
            self.published_setpoint.pose.position = self.hold_position
            self.published_setpoint.pose.orientation = self.current_pose.pose.orientation
            return
        
        # Calculate distance to target
        distance = self.calculate_distance_to_target()
        
        if distance < self.position_threshold:
            # Very close - use current position to prevent oscillation
            self.published_setpoint = self.current_pose
        elif distance < self.damping_distance:
            # Within damping distance - interpolate for smooth approach
            # Calculate damping factor (1.0 at damping_distance, min_speed_factor at threshold)
            damping_range = self.damping_distance - self.position_threshold
            distance_from_threshold = distance - self.position_threshold
            damping_factor = self.min_speed_factor + (1.0 - self.min_speed_factor) * (distance_from_threshold / damping_range)
            
            # Interpolate position
            self.published_setpoint = PoseStamped()
            self.published_setpoint.header.frame_id = "map"
            
            # Smooth approach to target
            alpha = damping_factor * 0.1  # Adjust speed of approach
            self.published_setpoint.pose.position.x = self.current_pose.pose.position.x + alpha * (self.target_pose.pose.position.x - self.current_pose.pose.position.x)
            self.published_setpoint.pose.position.y = self.current_pose.pose.position.y + alpha * (self.target_pose.pose.position.y - self.current_pose.pose.position.y)
            self.published_setpoint.pose.position.z = self.current_pose.pose.position.z + alpha * (self.target_pose.pose.position.z - self.current_pose.pose.position.z)
            
            # Keep current orientation
            self.published_setpoint.pose.orientation = self.current_pose.pose.orientation
        else:
            # Far from target - use target position directly
            self.published_setpoint = self.target_pose
    
    def publish_setpoint(self):
        """Continuously publish smoothed position setpoint"""
        # Initialize if needed
        if self.target_pose.pose.position.x == 0.0 and \
           self.target_pose.pose.position.y == 0.0 and \
           self.target_pose.pose.position.z == 0.0:
            if self.current_pose.pose.position.z > 0.1:  # If we're in the air
                self.target_pose = self.current_pose
                self.published_setpoint = self.current_pose
        
        # Calculate smooth setpoint
        self.calculate_smooth_setpoint()
        
        # Update timestamp
        self.published_setpoint.header.stamp = self.get_clock().now().to_msg()
        
        # Publish
        self.setpoint_pub.publish(self.published_setpoint)
        
        # Log status periodically
        if hasattr(self, '_last_log_time'):
            time_since_log = (self.get_clock().now().nanoseconds - self._last_log_time) / 1e9
            if time_since_log > 5.0:
                distance = self.calculate_distance_to_target()
                status = "HOLDING" if self.holding_position else "MOVING"
                if not self.position_reached and distance > 0.1:
                    self.get_logger().info(f'[{status}] Distance to target: {distance:.2f}m')
                self._last_log_time = self.get_clock().now().nanoseconds
        else:
            self._last_log_time = self.get_clock().now().nanoseconds


def main(args=None):
    rclpy.init(args=args)
    node = SmoothPositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()