#!/usr/bin/env python3
"""
Improved Pattern Executor with yaw control support
Executes waypoints with proper heading control for sharper turns
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import json
import math
import threading
import time

class ImprovedPatternExecutor(Node):
    def __init__(self):
        super().__init__('improved_pattern_executor')
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.position_cmd_pub = self.create_publisher(
            String, '/position_command', 10
        )
        
        self.simple_cmd_pub = self.create_publisher(
            String, '/simple_command', 10
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
        self.position_threshold = 1.5  # Tighter threshold for better accuracy
        self.yaw_threshold = 0.2  # radians (~11 degrees)
        self.execution_thread = None
        self.last_command_time = 0
        
        # Timer for status updates
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Improved Pattern Executor initialized with yaw control')
    
    def update_current_pose(self, msg):
        """Update current position"""
        self.current_pose = msg
    
    def get_current_yaw(self):
        """Extract yaw from quaternion"""
        if not self.current_pose:
            return 0
        
        q = self.current_pose.pose.orientation
        # Convert quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def load_waypoints(self, msg):
        """Load waypoints from pattern generator"""
        try:
            data = json.loads(msg.data)
            self.waypoints = data['waypoints']
            self.current_waypoint_index = 0
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints with yaw control')
            
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
        
        # Start execution in separate thread
        self.execution_thread = threading.Thread(target=self.execute_pattern)
        self.execution_thread.start()
        
        self.get_logger().info('Started improved pattern execution')
    
    def stop_execution(self):
        """Stop pattern execution"""
        self.executing = False
        
        # Send stop command
        stop_cmd = String()
        stop_cmd.data = 'stop'
        self.simple_cmd_pub.publish(stop_cmd)
        
        if self.execution_thread:
            self.execution_thread.join()
        
        self.get_logger().info('Stopped pattern execution')
    
    def pause_execution(self):
        """Pause at current position"""
        if self.executing:
            self.executing = False
            
            # Hover at current position
            hover_cmd = String()
            hover_cmd.data = 'stop'
            self.simple_cmd_pub.publish(hover_cmd)
            
            self.get_logger().info('Paused pattern execution')
    
    def resume_execution(self):
        """Resume from current waypoint"""
        if not self.executing and self.waypoints:
            self.executing = True
            
            # Resume in new thread
            self.execution_thread = threading.Thread(target=self.execute_pattern)
            self.execution_thread.start()
            
            self.get_logger().info('Resumed pattern execution')
    
    def reset_execution(self):
        """Reset to first waypoint"""
        self.stop_execution()
        self.current_waypoint_index = 0
        self.get_logger().info('Reset pattern execution')
    
    def execute_pattern(self):
        """Main pattern execution loop with yaw control"""
        while self.executing and self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            
            # Check if we need to rotate first
            if 'yaw' in waypoint:
                target_yaw = waypoint['yaw']
                current_yaw = self.get_current_yaw()
                yaw_error = self.normalize_angle(target_yaw - current_yaw)
                
                # If yaw error is significant, rotate first
                if abs(yaw_error) > self.yaw_threshold:
                    self.get_logger().info(f'Rotating to yaw: {math.degrees(target_yaw):.1f}°')
                    
                    # Send yaw commands
                    if yaw_error > 0:
                        for _ in range(int(abs(yaw_error) / 0.2)):
                            if not self.executing:
                                break
                            yaw_cmd = String()
                            yaw_cmd.data = 'yaw_left'
                            self.simple_cmd_pub.publish(yaw_cmd)
                            time.sleep(0.2)
                    else:
                        for _ in range(int(abs(yaw_error) / 0.2)):
                            if not self.executing:
                                break
                            yaw_cmd = String()
                            yaw_cmd.data = 'yaw_right'
                            self.simple_cmd_pub.publish(yaw_cmd)
                            time.sleep(0.2)
                    
                    # Brief pause after rotation
                    time.sleep(0.5)
            
            # Now fly to position
            goto_cmd = String()
            goto_cmd.data = f"goto:{waypoint['x']},{waypoint['y']},{waypoint['z']}"
            self.position_cmd_pub.publish(goto_cmd)
            self.last_command_time = time.time()
            
            self.get_logger().info(
                f'Flying to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
                f'({waypoint["x"]:.1f}, {waypoint["y"]:.1f}, {waypoint["z"]:.1f})'
            )
            
            # Wait until reached
            while self.executing and not self.has_reached_waypoint(waypoint):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.executing:
                # Brief stabilization
                time.sleep(0.3)
                self.current_waypoint_index += 1
        
        if self.current_waypoint_index >= len(self.waypoints):
            self.get_logger().info('Pattern execution completed')
            self.executing = False
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def has_reached_waypoint(self, waypoint):
        """Check if drone has reached the waypoint"""
        if not self.current_pose:
            return False
        
        dx = waypoint['x'] - self.current_pose.pose.position.x
        dy = waypoint['y'] - self.current_pose.pose.position.y
        dz = waypoint['z'] - self.current_pose.pose.position.z
        
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        return distance < self.position_threshold
    
    def publish_status(self):
        """Publish pattern execution status"""
        status = {
            'executing': self.executing,
            'total_waypoints': len(self.waypoints),
            'current_waypoint': self.current_waypoint_index + 1 if self.executing else 0,
            'progress': (self.current_waypoint_index / len(self.waypoints) * 100) if self.waypoints else 0
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImprovedPatternExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()