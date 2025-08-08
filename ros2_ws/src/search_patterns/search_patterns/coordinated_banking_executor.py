#!/usr/bin/env python3
"""
Coordinated Banking Pattern Executor
Works WITH the velocity coordinator system - no conflicts!
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_msgs.msg import String, Int32
import numpy as np
import json
import math

class CoordinatedBankingExecutor(Node):
    def __init__(self):
        super().__init__('coordinated_banking_executor')
        
        # Pattern parameters
        self.WAYPOINT_THRESHOLD = 2.5      # m
        self.CRUISE_SPEED = 3.0            # m/s
        self.TURN_SPEED = 1.5              # m/s
        self.MAX_YAW_RATE = 0.4            # rad/s
        self.VELOCITY_PRIORITY = 3         # Priority for velocity coordinator
        
        # Pattern state
        self.waypoints = []
        self.current_waypoint_index = 0
        self.pattern_active = False
        
        # Current state
        self.current_pose = None
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers - USE VELOCITY COORDINATOR!
        self.velocity_pub = self.create_publisher(
            Twist,  # Note: Twist, not TwistStamped for coordinator
            '/search_pattern/velocity_command',  # Correct topic for pattern execution (priority 4)
            10
        )
        
        self.priority_pub = self.create_publisher(
            Int32,
            '/velocity_priority',  # Tell coordinator our priority
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/pattern_status',
            10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile=self.mavros_qos
        )
        
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.velocity_callback,
            qos_profile=self.mavros_qos
        )
        
        self.waypoints_sub = self.create_subscription(
            String,
            '/pattern_waypoints',
            self.waypoints_callback,
            10
        )
        
        self.control_sub = self.create_subscription(
            String,
            '/pattern_control',
            self.control_callback,
            10
        )
        
        # Control timer - 10Hz
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Coordinated Banking Executor initialized')
        self.get_logger().info('Using velocity coordinator with priority 3')
    
    def pose_callback(self, msg):
        """Update current pose"""
        self.current_pose = msg.pose
    
    def velocity_callback(self, msg):
        """Update current velocity"""
        vel = msg.twist.linear
        self.current_velocity = np.array([vel.x, vel.y, vel.z])
    
    def waypoints_callback(self, msg):
        """Receive new waypoints"""
        try:
            data = json.loads(msg.data)
            raw_waypoints = data['waypoints']
            
            # Convert waypoints to list format
            self.waypoints = []
            for wp in raw_waypoints:
                if isinstance(wp, dict):
                    self.waypoints.append([wp['x'], wp['y'], wp['z']])
                else:
                    self.waypoints.append(wp)
            
            self.current_waypoint_index = 0
            self.get_logger().info(f'Received {len(self.waypoints)} waypoints')
            
        except Exception as e:
            self.get_logger().error(f'Failed to parse waypoints: {e}')
    
    def control_callback(self, msg):
        """Handle pattern control commands"""
        command = msg.data.lower()
        
        if command == 'start':
            if self.waypoints:
                self.pattern_active = True
                self.current_waypoint_index = 0
                # Set our priority
                self.publish_priority(self.VELOCITY_PRIORITY)
                self.get_logger().info('Starting coordinated pattern execution')
                self.publish_status('executing')
            else:
                self.get_logger().warn('No waypoints loaded')
        
        elif command == 'stop':
            self.pattern_active = False
            # Release priority
            self.publish_priority(0)
            self.send_stop_command()
            self.get_logger().info('Stopping pattern execution')
            self.publish_status('stopped')
        
        elif command == 'pause':
            self.pattern_active = False
            # Keep priority but stop moving
            self.send_stop_command()
            self.get_logger().info('Pausing pattern execution')
            self.publish_status('paused')
        
        elif command == 'resume':
            if self.waypoints:
                self.pattern_active = True
                self.publish_priority(self.VELOCITY_PRIORITY)
                self.get_logger().info('Resuming pattern execution')
                self.publish_status('executing')
    
    def publish_priority(self, priority):
        """Tell velocity coordinator our priority"""
        msg = Int32()
        msg.data = priority
        self.priority_pub.publish(msg)
    
    def send_velocity_command(self, vx, vy, vz, yaw_rate=0.0):
        """Send velocity through the coordinator"""
        cmd = Twist()  # Use Twist, not TwistStamped
        
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = yaw_rate
        
        self.velocity_pub.publish(cmd)
    
    def send_stop_command(self):
        """Send zero velocity command"""
        self.send_velocity_command(0.0, 0.0, 0.0, 0.0)
    
    def get_current_yaw(self):
        """Extract yaw from quaternion"""
        if not self.current_pose:
            return 0.0
        
        q = self.current_pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return yaw
    
    def control_loop(self):
        """Main control loop - works with velocity coordinator"""
        if not self.pattern_active or not self.current_pose or not self.waypoints:
            return
        
        # Get current position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_z = self.current_pose.position.z
        
        # Check if we've reached all waypoints
        if self.current_waypoint_index >= len(self.waypoints):
            self.pattern_active = False
            self.publish_priority(0)  # Release priority
            self.send_stop_command()
            self.publish_status('complete')
            self.get_logger().info('Pattern complete!')
            return
        
        # Get target waypoint
        target_wp = self.waypoints[self.current_waypoint_index]
        target_x, target_y, target_z = target_wp[0], target_wp[1], target_wp[2]
        
        # Calculate distance to target
        dx = target_x - current_x
        dy = target_y - current_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        # Check if we've reached the waypoint
        if distance < self.WAYPOINT_THRESHOLD:
            self.current_waypoint_index += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}/{len(self.waypoints)}')
            return
        
        # Calculate desired heading
        desired_yaw = math.atan2(dy, dx)
        current_yaw = self.get_current_yaw()
        
        # Calculate yaw error
        yaw_error = desired_yaw - current_yaw
        # Normalize to [-pi, pi]
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # Determine speed based on turn requirement
        if abs(yaw_error) > 0.5:  # ~29 degrees - need to turn
            speed = self.TURN_SPEED
            yaw_rate = np.clip(yaw_error * 1.5, -self.MAX_YAW_RATE, self.MAX_YAW_RATE)
        else:
            speed = self.CRUISE_SPEED
            yaw_rate = np.clip(yaw_error * 0.8, -self.MAX_YAW_RATE, self.MAX_YAW_RATE)
        
        # Calculate velocity components
        if distance > 0:
            vx = (dx / distance) * speed
            vy = (dy / distance) * speed
        else:
            vx = 0.0
            vy = 0.0
        
        # Altitude control
        vz = (target_z - current_z) * 0.5
        vz = np.clip(vz, -1.0, 1.0)
        
        # Send velocity command through coordinator
        self.send_velocity_command(vx, vy, vz, yaw_rate)
        
        # Log occasionally
        if self.get_clock().now().nanoseconds % 2000000000 < 100000000:  # Every 2 seconds
            self.get_logger().info(
                f'WP {self.current_waypoint_index+1}/{len(self.waypoints)}, '
                f'Dist: {distance:.1f}m, '
                f'Speed: {speed:.1f}m/s'
            )
    
    def publish_status(self, status):
        """Publish pattern execution status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CoordinatedBankingExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Coordinated Banking Executor')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()