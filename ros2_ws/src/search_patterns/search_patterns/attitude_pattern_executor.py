#!/usr/bin/env python3
"""
Attitude Pattern Executor - Uses proper yaw, pitch, roll control
Flies like a real pilot: rotate to face direction, then move forward
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String, Header
import json
import math
import threading
import time
import numpy as np

class AttitudePatternExecutor(Node):
    def __init__(self):
        super().__init__('attitude_pattern_executor')
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers - Use MAVROS setpoint_raw for full control
        self.setpoint_raw_pub = self.create_publisher(
            PositionTarget, 
            '/mavros/setpoint_raw/local', 
            10
        )
        
        self.velocity_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            10
        )
        
        self.status_pub = self.create_publisher(
            String, '/pattern_status', 10
        )
        
        # Subscribers
        self.waypoints_sub = self.create_subscription(
            String, '/pattern_waypoints',
            self.load_waypoints, 10
        )
        
        self.control_sub = self.create_subscription(
            String, '/pattern_control',
            self.handle_control, 10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.update_current_pose, 
            qos_profile=self.mavros_qos
        )
        
        # State
        self.waypoints = []
        self.current_waypoint_index = 0
        self.executing = False
        self.current_pose = None
        self.current_yaw = 0.0
        
        # Control parameters
        self.position_threshold = 1.5  # Tighter for better accuracy
        self.yaw_threshold = 0.1  # radians (~6 degrees)
        self.cruise_speed = 3.0  # m/s forward speed
        self.approach_speed = 1.0  # m/s when close to waypoint
        self.yaw_rate = 0.5  # rad/s rotation speed
        
        self.execution_thread = None
        
        # Control timer - sends setpoints at 20Hz
        self.control_timer = self.create_timer(0.05, self.send_control_command)
        self.current_target = None
        self.control_mode = 'idle'  # 'idle', 'rotating', 'moving', 'approaching'
        
        # Status timer
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Attitude Pattern Executor initialized - Full flight control')
    
    def update_current_pose(self, msg):
        """Update current position and orientation"""
        self.current_pose = msg
        # Extract yaw from quaternion
        q = msg.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q)
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        # Using transforms3d would be better, but for simplicity:
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to quaternion (keeping level flight)"""
        # Roll = 0, Pitch = 0, Yaw = yaw
        q = Quaternion()
        q.w = math.cos(yaw / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2)
        return q
    
    def load_waypoints(self, msg):
        """Load waypoints from pattern generator"""
        try:
            data = json.loads(msg.data)
            self.waypoints = data['waypoints']
            self.current_waypoint_index = 0
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints for attitude control')
            
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {e}')
    
    def handle_control(self, msg):
        """Handle pattern control commands"""
        command = msg.data.lower()
        
        if command == 'start':
            self.start_execution()
        elif command == 'stop':
            self.stop_execution()
        elif command == 'pause':
            self.pause_execution()
        elif command == 'resume':
            self.resume_execution()
        elif command == 'reset':
            self.reset_execution()
    
    def start_execution(self):
        """Start executing the pattern"""
        if not self.waypoints:
            self.get_logger().warn('No waypoints loaded')
            return
        
        if self.executing:
            self.get_logger().warn('Already executing pattern')
            return
        
        self.executing = True
        self.current_waypoint_index = 0
        self.control_mode = 'idle'
        
        # Start execution in separate thread
        self.execution_thread = threading.Thread(target=self.execute_pattern)
        self.execution_thread.start()
        
        self.get_logger().info('Started attitude-based pattern execution')
    
    def stop_execution(self):
        """Stop pattern execution"""
        self.executing = False
        self.control_mode = 'idle'
        self.current_target = None
        
        # Send zero velocity to stop
        self.send_stop_command()
        
        if self.execution_thread and self.execution_thread.is_alive():
            self.execution_thread.join(timeout=2.0)
        
        self.get_logger().info('Stopped pattern execution')
    
    def pause_execution(self):
        """Pause at current position"""
        if self.executing:
            self.executing = False
            self.control_mode = 'idle'
            self.send_stop_command()
            self.get_logger().info('Paused pattern execution')
    
    def resume_execution(self):
        """Resume from current waypoint"""
        if not self.executing and self.waypoints:
            self.executing = True
            self.execution_thread = threading.Thread(target=self.execute_pattern)
            self.execution_thread.start()
            self.get_logger().info('Resumed pattern execution')
    
    def reset_execution(self):
        """Reset to first waypoint"""
        self.stop_execution()
        self.current_waypoint_index = 0
        self.get_logger().info('Reset pattern execution')
    
    def calculate_target_yaw(self, target_x, target_y):
        """Calculate yaw angle to face target position"""
        if not self.current_pose:
            return 0.0
        
        dx = target_x - self.current_pose.pose.position.x
        dy = target_y - self.current_pose.pose.position.y
        return math.atan2(dy, dx)
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def execute_pattern(self):
        """Main execution loop with proper attitude control"""
        while self.executing and self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            
            # Step 1: Calculate target yaw to face waypoint
            target_yaw = self.calculate_target_yaw(waypoint['x'], waypoint['y'])
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            
            # Step 2: Rotate to face target
            if abs(yaw_error) > self.yaw_threshold:
                self.get_logger().info(f'Rotating to face waypoint {self.current_waypoint_index + 1}')
                self.control_mode = 'rotating'
                self.current_target = {
                    'x': self.current_pose.pose.position.x,
                    'y': self.current_pose.pose.position.y,
                    'z': waypoint['z'],
                    'yaw': target_yaw
                }
                
                # Wait for rotation to complete
                while self.executing and abs(self.normalize_angle(target_yaw - self.current_yaw)) > self.yaw_threshold:
                    time.sleep(0.1)
            
            # Step 3: Move forward to waypoint
            self.get_logger().info(
                f'Moving to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
                f'({waypoint["x"]:.1f}, {waypoint["y"]:.1f}, {waypoint["z"]:.1f})'
            )
            
            self.current_target = {
                'x': waypoint['x'],
                'y': waypoint['y'],
                'z': waypoint['z'],
                'yaw': target_yaw
            }
            
            # Start moving
            while self.executing:
                if not self.current_pose:
                    time.sleep(0.1)
                    continue
                
                dx = waypoint['x'] - self.current_pose.pose.position.x
                dy = waypoint['y'] - self.current_pose.pose.position.y
                dz = waypoint['z'] - self.current_pose.pose.position.z
                distance = math.sqrt(dx**2 + dy**2 + dz**2)
                
                # Update control mode based on distance
                if distance < self.position_threshold:
                    # Reached waypoint
                    self.control_mode = 'idle'
                    break
                elif distance < 5.0:
                    # Close - slow down
                    self.control_mode = 'approaching'
                else:
                    # Far - cruise speed
                    self.control_mode = 'moving'
                
                time.sleep(0.1)
            
            if self.executing:
                # Brief stabilization at waypoint
                time.sleep(0.5)
                self.current_waypoint_index += 1
        
        # Pattern completed
        if self.current_waypoint_index >= len(self.waypoints):
            self.executing = False
            self.control_mode = 'idle'
            self.send_stop_command()
            self.get_logger().info('Pattern execution completed - hovering')
    
    def send_control_command(self):
        """Send control commands at high frequency"""
        if not self.current_pose or not self.current_target:
            return
        
        msg = PositionTarget()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        if self.control_mode == 'rotating':
            # Pure rotation - hold position, change yaw
            msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
                           PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
            
            msg.position.x = self.current_target['x']
            msg.position.y = self.current_target['y']
            msg.position.z = self.current_target['z']
            msg.yaw = self.current_target['yaw']
            
        elif self.control_mode in ['moving', 'approaching']:
            # Position + yaw control
            msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
                           PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
            
            msg.position.x = self.current_target['x']
            msg.position.y = self.current_target['y']
            msg.position.z = self.current_target['z']
            msg.yaw = self.current_target['yaw']
            
            # Could also use velocity control for smoother flight
            if self.control_mode == 'moving':
                # Calculate velocity vector towards target
                dx = self.current_target['x'] - self.current_pose.pose.position.x
                dy = self.current_target['y'] - self.current_pose.pose.position.y
                dist = math.sqrt(dx**2 + dy**2)
                
                if dist > 0:
                    # Use velocity control instead for smoother flight
                    msg.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                                   PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ
                    
                    # Forward velocity in body frame
                    msg.velocity.x = self.cruise_speed if self.control_mode == 'moving' else self.approach_speed
                    msg.velocity.y = 0.0  # No lateral movement
                    msg.velocity.z = 0.0  # Maintain altitude
                    msg.yaw = self.current_target['yaw']
        
        elif self.control_mode == 'idle':
            # Hold current position
            msg.type_mask = PositionTarget.IGNORE_VX | PositionTarget.IGNORE_VY | PositionTarget.IGNORE_VZ | \
                           PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                           PositionTarget.IGNORE_YAW_RATE
            
            msg.position.x = self.current_pose.pose.position.x
            msg.position.y = self.current_pose.pose.position.y
            msg.position.z = self.current_pose.pose.position.z
            msg.yaw = self.current_yaw
        
        self.setpoint_raw_pub.publish(msg)
    
    def send_stop_command(self):
        """Send stop command"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        self.velocity_pub.publish(msg)
    
    def publish_status(self):
        """Publish pattern execution status"""
        status = {
            'executing': self.executing,
            'control_mode': self.control_mode,
            'total_waypoints': len(self.waypoints),
            'current_waypoint': self.current_waypoint_index + 1 if self.executing else 0,
            'progress': (self.current_waypoint_index / len(self.waypoints) * 100) if self.waypoints else 0
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AttitudePatternExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()