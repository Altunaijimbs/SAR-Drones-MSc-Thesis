#!/usr/bin/env python3
"""
Hybrid Position Controller - Uses velocity commands to reach position targets
Works with existing velocity coordinator to prevent conflicts
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from mavros_msgs.msg import State
import math
import json
import threading

class HybridPositionController(Node):
    def __init__(self):
        super().__init__('hybrid_position_controller')
        
        # Define QoS profile for MAVROS topics
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers - Velocity commands that work with velocity coordinator
        self.vel_pub = self.create_publisher(
            Twist, '/hybrid_position/velocity_command', 10
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
        self.target_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.is_armed = False
        self.mode = ""
        self.position_reached = True
        self.active = False
        
        # Control parameters
        self.position_threshold = 0.3    # meters - when to stop
        self.slow_threshold = 3.0       # meters - when to slow down (increased)
        self.max_velocity = 4.0         # m/s - maximum velocity (doubled)
        self.min_velocity = 0.2         # m/s - minimum velocity (doubled)
        self.p_gain = 1.2              # Proportional gain (slightly increased)
        
        # Control loop timer (10Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        # Timeout timer
        self.timeout_duration = 30.0  # seconds
        self.movement_timer = None
        
        self.get_logger().info('Hybrid Position Controller initialized')
        self.get_logger().info('Publishes velocity commands to reach position targets')
    
    def state_callback(self, msg):
        """Update drone state"""
        self.is_armed = msg.armed
        self.mode = msg.mode
    
    def update_current_pose(self, msg):
        """Update current drone position"""
        self.current_pose = msg
    
    def handle_position_command(self, msg):
        """Handle position commands"""
        try:
            cmd = msg.data
            self.get_logger().info(f'Received position command: {cmd}')
            
            # Try to parse as JSON first
            try:
                data = json.loads(cmd)
                if 'x' in data and 'y' in data and 'z' in data:
                    self.set_target_position(data['x'], data['y'], data['z'])
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
                        self.set_target_position(x, y, z)
                else:
                    # Relative movement
                    distance = float(value)
                    self.move_relative(command, distance)
            
            elif cmd.lower() == 'hover' or cmd.lower() == 'stop':
                # Stop movement
                self.stop_movement()
            
        except Exception as e:
            self.get_logger().error(f'Error parsing position command: {e}')
    
    def move_relative(self, direction, distance):
        """Calculate target position for relative movement"""
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        current_z = self.current_pose.pose.position.z
        
        # Get current yaw angle
        yaw = self.get_yaw_from_quaternion(self.current_pose.pose.orientation)
        
        # Calculate target based on direction
        target_x = current_x
        target_y = current_y
        target_z = current_z
        
        if direction == 'forward':
            target_x += distance * math.cos(yaw)
            target_y += distance * math.sin(yaw)
        elif direction == 'backward' or direction == 'back':
            target_x -= distance * math.cos(yaw)
            target_y -= distance * math.sin(yaw)
        elif direction == 'left':
            target_x += distance * math.cos(yaw + math.pi/2)
            target_y += distance * math.sin(yaw + math.pi/2)
        elif direction == 'right':
            target_x += distance * math.cos(yaw - math.pi/2)
            target_y += distance * math.sin(yaw - math.pi/2)
        elif direction == 'up':
            target_z += distance
        elif direction == 'down':
            target_z -= distance
        else:
            self.get_logger().warn(f'Unknown direction: {direction}')
            return
        
        self.set_target_position(target_x, target_y, target_z)
        self.get_logger().info(
            f'Moving {direction} {distance}m to position: '
            f'({target_x:.2f}, {target_y:.2f}, {target_z:.2f})'
        )
    
    def set_target_position(self, x, y, z):
        """Set target position and start movement"""
        self.target_position = {'x': x, 'y': y, 'z': z}
        self.position_reached = False
        self.active = True
        
        # Start timeout timer
        if self.movement_timer:
            self.movement_timer.cancel()
        self.movement_timer = threading.Timer(self.timeout_duration, self.movement_timeout)
        self.movement_timer.start()
        
        self.get_logger().info(f'Target position set: ({x:.2f}, {y:.2f}, {z:.2f})')
    
    def stop_movement(self):
        """Stop all movement"""
        self.active = False
        self.position_reached = True
        
        # Cancel timeout timer
        if self.movement_timer:
            self.movement_timer.cancel()
            self.movement_timer = None
        
        # Send zero velocity
        vel_cmd = Twist()
        self.vel_pub.publish(vel_cmd)
        
        self.get_logger().info('Movement stopped')
    
    def movement_timeout(self):
        """Called when movement times out"""
        self.get_logger().warn('Movement timeout - stopping')
        self.stop_movement()
    
    def get_yaw_from_quaternion(self, quaternion):
        """Extract yaw angle from quaternion"""
        q = quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def control_loop(self):
        """Main control loop - converts position error to velocity commands"""
        if not self.active or self.position_reached:
            return
        
        # Calculate position error
        error_x = self.target_position['x'] - self.current_pose.pose.position.x
        error_y = self.target_position['y'] - self.current_pose.pose.position.y
        error_z = self.target_position['z'] - self.current_pose.pose.position.z
        
        # Calculate distance to target
        distance = math.sqrt(error_x**2 + error_y**2 + error_z**2)
        
        # Check if we've reached the target
        if distance < self.position_threshold:
            self.get_logger().info(f'✓ Position reached! (distance: {distance:.2f}m)')
            self.stop_movement()
            return
        
        # Calculate velocity commands using proportional control
        # Scale velocity based on distance (slow down as we approach)
        if distance < self.slow_threshold:
            # Linear scaling in slow zone
            velocity_scale = max(self.min_velocity, 
                               self.max_velocity * (distance / self.slow_threshold))
        else:
            velocity_scale = self.max_velocity
        
        # Apply proportional gain and velocity limit
        vel_x = self.p_gain * error_x
        vel_y = self.p_gain * error_y
        vel_z = self.p_gain * error_z
        
        # Normalize to max velocity
        vel_magnitude = math.sqrt(vel_x**2 + vel_y**2 + vel_z**2)
        if vel_magnitude > velocity_scale:
            scale_factor = velocity_scale / vel_magnitude
            vel_x *= scale_factor
            vel_y *= scale_factor
            vel_z *= scale_factor
        
        # Create velocity command
        vel_cmd = Twist()
        vel_cmd.linear.x = vel_x
        vel_cmd.linear.y = vel_y
        vel_cmd.linear.z = vel_z
        
        # Publish velocity command
        self.vel_pub.publish(vel_cmd)
        
        # Log progress periodically
        if not hasattr(self, '_last_log_time'):
            self._last_log_time = self.get_clock().now().nanoseconds
        
        time_since_log = (self.get_clock().now().nanoseconds - self._last_log_time) / 1e9
        if time_since_log > 2.0:
            self.get_logger().info(
                f'Distance to target: {distance:.2f}m, '
                f'Velocity: ({vel_x:.2f}, {vel_y:.2f}, {vel_z:.2f}) m/s'
            )
            self._last_log_time = self.get_clock().now().nanoseconds


def main(args=None):
    rclpy.init(args=args)
    node = HybridPositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()