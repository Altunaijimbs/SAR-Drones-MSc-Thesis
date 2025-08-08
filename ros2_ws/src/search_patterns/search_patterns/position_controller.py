#!/usr/bin/env python3
"""
Position-based movement controller for precise drone control
Handles commands like "go forward 5 meters" using position setpoints
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import String
from mavros_msgs.msg import State
import math
import json

class PositionController(Node):
    def __init__(self):
        super().__init__('position_controller')
        
        # Define QoS profile for MAVROS topics (sensor data uses BEST_EFFORT)
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
        
        # Subscribers - use MAVROS QoS for pose topic
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.update_current_pose, 
            qos_profile=self.mavros_qos
        )
        
        self.position_cmd_sub = self.create_subscription(
            String, '/position_command',
            self.handle_position_command, 10
        )
        
        # State topic might use RELIABLE, but let's use compatible QoS
        self.state_sub = self.create_subscription(
            State, '/mavros/state',
            self.state_callback, 
            qos_profile=self.mavros_qos
        )
        
        # State
        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.is_armed = False
        self.mode = ""
        self.position_reached = True
        self.position_threshold = 0.5  # meters
        
        # Timer for publishing setpoints (required for OFFBOARD mode)
        self.setpoint_timer = self.create_timer(0.05, self.publish_setpoint)  # 20Hz
        
        self.get_logger().info('Position Controller initialized')
    
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
                self.get_logger().info(f'Position reached! Distance: {distance:.2f}m')
    
    def calculate_distance_to_target(self):
        """Calculate 3D distance to target position"""
        dx = self.target_pose.pose.position.x - self.current_pose.pose.position.x
        dy = self.target_pose.pose.position.y - self.current_pose.pose.position.y
        dz = self.target_pose.pose.position.z - self.current_pose.pose.position.z
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def handle_position_command(self, msg):
        """
        Handle position commands
        Format: "command:value" or JSON
        Examples:
        - "forward:5" - Move forward 5 meters
        - "backward:3" - Move backward 3 meters
        - "left:2" - Move left 2 meters
        - "right:4" - Move right 4 meters
        - "up:2" - Move up 2 meters
        - "down:1" - Move down 1 meter
        - "goto:10,20,5" - Go to absolute position
        - {"x": 10, "y": 20, "z": 5} - JSON position
        - "hover" - Maintain current position
        """
        try:
            cmd = msg.data
            self.get_logger().info(f'Received position command: {cmd}')
            
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
                # Maintain current position
                self.target_pose = self.current_pose
                self.position_reached = False
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
            # Move forward based on current heading
            self.target_pose.pose.position.x += distance * math.cos(yaw)
            self.target_pose.pose.position.y += distance * math.sin(yaw)
        elif direction == 'backward' or direction == 'back':
            # Move backward based on current heading
            self.target_pose.pose.position.x -= distance * math.cos(yaw)
            self.target_pose.pose.position.y -= distance * math.sin(yaw)
        elif direction == 'left':
            # Move left (perpendicular to heading)
            self.target_pose.pose.position.x += distance * math.cos(yaw + math.pi/2)
            self.target_pose.pose.position.y += distance * math.sin(yaw + math.pi/2)
        elif direction == 'right':
            # Move right (perpendicular to heading)
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
        # Convert quaternion to Euler angles
        q = quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def publish_setpoint(self):
        """Continuously publish position setpoint"""
        # Only publish if we have a valid target
        if self.target_pose.pose.position.x == 0.0 and \
           self.target_pose.pose.position.y == 0.0 and \
           self.target_pose.pose.position.z == 0.0:
            # No target set yet, use current position as target
            if self.current_pose.pose.position.z > 0.1:  # If we're in the air
                self.target_pose = self.current_pose
        
        # Update timestamp
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        
        # Publish setpoint
        self.setpoint_pub.publish(self.target_pose)
        
        # Log status periodically
        if hasattr(self, '_last_log_time'):
            time_since_log = (self.get_clock().now().nanoseconds - self._last_log_time) / 1e9
            if time_since_log > 5.0 and not self.position_reached:
                distance = self.calculate_distance_to_target()
                self.get_logger().info(f'Distance to target: {distance:.2f}m')
                self._last_log_time = self.get_clock().now().nanoseconds
        else:
            self._last_log_time = self.get_clock().now().nanoseconds
    
    def get_status(self):
        """Get controller status for external queries"""
        return {
            'position_reached': self.position_reached,
            'current_position': {
                'x': self.current_pose.pose.position.x,
                'y': self.current_pose.pose.position.y,
                'z': self.current_pose.pose.position.z
            },
            'target_position': {
                'x': self.target_pose.pose.position.x,
                'y': self.target_pose.pose.position.y,
                'z': self.target_pose.pose.position.z
            },
            'distance_to_target': self.calculate_distance_to_target()
        }


def main(args=None):
    rclpy.init(args=args)
    node = PositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()