#!/usr/bin/env python3
"""
Coordinated Pattern Executor - Works WITH velocity coordinator
Publishes to velocity_command with proper priority
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Int32
import json
import math
import threading
import time

class CoordinatedPatternExecutor(Node):
    def __init__(self):
        super().__init__('coordinated_pattern_executor')
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers - Work with velocity coordinator
        self.velocity_pub = self.create_publisher(
            Twist, '/velocity_command_2', 10  # Priority 2 for patterns
        )
        
        self.priority_pub = self.create_publisher(
            Int32, '/velocity_priority_2', 10
        )
        
        self.simple_cmd_pub = self.create_publisher(
            String, '/simple_command', 10  # For yaw control
        )
        
        self.position_cmd_pub = self.create_publisher(
            String, '/position_command', 10  # Use existing position system
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
        self.position_threshold = 2.0
        self.yaw_threshold = 0.2  # radians
        
        self.execution_thread = None
        
        # Publish our priority
        self.priority_msg = Int32()
        self.priority_msg.data = 2
        
        # Status timer
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Coordinated Pattern Executor initialized - works with velocity coordinator')
    
    def update_current_pose(self, msg):
        """Update current position and orientation"""
        self.current_pose = msg
        # Extract yaw from quaternion
        q = msg.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q)
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
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
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints for coordinated execution')
            
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
        
        self.get_logger().info('Started coordinated pattern execution')
    
    def stop_execution(self):
        """Stop pattern execution"""
        self.executing = False
        
        # Release velocity control
        zero_vel = Twist()
        self.velocity_pub.publish(zero_vel)
        
        # Send stop via simple command
        stop_cmd = String()
        stop_cmd.data = 'stop'
        self.simple_cmd_pub.publish(stop_cmd)
        
        if self.execution_thread and self.execution_thread.is_alive():
            self.execution_thread.join(timeout=2.0)
        
        self.get_logger().info('Stopped coordinated pattern execution')
    
    def pause_execution(self):
        """Pause at current position"""
        if self.executing:
            self.executing = False
            
            # Send stop
            stop_cmd = String()
            stop_cmd.data = 'stop'
            self.simple_cmd_pub.publish(stop_cmd)
            
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
        """Main execution loop using coordinated control"""
        while self.executing and self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            
            # Calculate target yaw
            target_yaw = self.calculate_target_yaw(waypoint['x'], waypoint['y'])
            yaw_error = self.normalize_angle(target_yaw - self.current_yaw)
            
            # Step 1: Rotate to face waypoint if needed
            if abs(yaw_error) > self.yaw_threshold:
                self.get_logger().info(f'Rotating to face waypoint {self.current_waypoint_index + 1}')
                
                # Use simple commands for rotation
                rotation_count = int(abs(yaw_error) / 0.2)
                for _ in range(rotation_count):
                    if not self.executing:
                        break
                    
                    if yaw_error > 0:
                        cmd = String()
                        cmd.data = 'yaw_left'
                        self.simple_cmd_pub.publish(cmd)
                    else:
                        cmd = String()
                        cmd.data = 'yaw_right'
                        self.simple_cmd_pub.publish(cmd)
                    
                    time.sleep(0.2)
                
                # Brief pause after rotation
                time.sleep(0.5)
            
            # Step 2: Move to waypoint using position command
            self.get_logger().info(
                f'Moving to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
                f'({waypoint["x"]:.1f}, {waypoint["y"]:.1f}, {waypoint["z"]:.1f})'
            )
            
            # Send position command
            pos_cmd = String()
            pos_cmd.data = f"goto:{waypoint['x']},{waypoint['y']},{waypoint['z']}"
            self.position_cmd_pub.publish(pos_cmd)
            
            # Wait until reached
            while self.executing:
                if not self.current_pose:
                    time.sleep(0.1)
                    continue
                
                dx = waypoint['x'] - self.current_pose.pose.position.x
                dy = waypoint['y'] - self.current_pose.pose.position.y
                dz = waypoint['z'] - self.current_pose.pose.position.z
                distance = math.sqrt(dx**2 + dy**2 + dz**2)
                
                if distance < self.position_threshold:
                    break
                
                # Keep publishing priority to maintain control
                self.priority_pub.publish(self.priority_msg)
                time.sleep(0.1)
            
            if self.executing:
                # Brief stabilization
                time.sleep(0.5)
                self.current_waypoint_index += 1
        
        # Pattern completed
        if self.current_waypoint_index >= len(self.waypoints):
            self.executing = False
            
            # Send stop
            stop_cmd = String()
            stop_cmd.data = 'stop'
            self.simple_cmd_pub.publish(stop_cmd)
            
            self.get_logger().info('Pattern execution completed')
    
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
    node = CoordinatedPatternExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()