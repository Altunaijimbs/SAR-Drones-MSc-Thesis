#!/usr/bin/env python3
"""
Fixed Movement Executor - Moves WITHOUT continuous rotation
Separates turning from moving for clear, predictable movement
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
from std_msgs.msg import String, Int32
import numpy as np
import json
import math

class FixedMovementExecutor(Node):
    def __init__(self):
        super().__init__('fixed_movement_executor')
        
        # Movement parameters - NO YAW CONTROL
        self.WAYPOINT_THRESHOLD = 2.0      # m
        self.CRUISE_SPEED = 3.0            # m/s
        self.APPROACH_SPEED = 1.5          # m/s
        self.VELOCITY_PRIORITY = 4         # Priority for velocity coordinator
        
        # Pattern state
        self.waypoints = []
        self.current_waypoint_index = 0
        self.pattern_active = False
        
        # Current state
        self.current_pose = None
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.velocity_pub = self.create_publisher(
            Twist,
            '/search_pattern/velocity_command',  # For velocity coordinator
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
        
        self.get_logger().info('Fixed Movement Executor initialized')
        self.get_logger().info('NO YAW CONTROL - Pure position movement only')
    
    def pose_callback(self, msg):
        """Update current pose"""
        self.current_pose = msg.pose
    
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
                self.get_logger().info('Starting pattern execution (NO YAW)')
                self.publish_status('executing')
            else:
                self.get_logger().warn('No waypoints loaded')
        
        elif command == 'stop':
            self.pattern_active = False
            self.send_stop_command()
            self.get_logger().info('Stopping pattern execution')
            self.publish_status('stopped')
        
        elif command == 'pause':
            self.pattern_active = False
            self.send_stop_command()
            self.get_logger().info('Pausing pattern execution')
            self.publish_status('paused')
        
        elif command == 'resume':
            if self.waypoints:
                self.pattern_active = True
                self.get_logger().info('Resuming pattern execution')
                self.publish_status('executing')
    
    def send_velocity_command(self, vx, vy, vz):
        """Send velocity WITHOUT yaw control"""
        cmd = Twist()
        
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.linear.z = vz
        cmd.angular.z = 0.0  # NO YAW ROTATION!
        
        self.velocity_pub.publish(cmd)
    
    def send_stop_command(self):
        """Send zero velocity command"""
        self.send_velocity_command(0.0, 0.0, 0.0)
    
    def control_loop(self):
        """Main control loop - simple position-based movement"""
        if not self.pattern_active or not self.current_pose or not self.waypoints:
            return
        
        # Get current position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_z = self.current_pose.position.z
        
        # Check if we've reached all waypoints
        if self.current_waypoint_index >= len(self.waypoints):
            self.pattern_active = False
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
        dz = target_z - current_z
        distance_xy = math.sqrt(dx*dx + dy*dy)
        
        # Check if we've reached the waypoint
        if distance_xy < self.WAYPOINT_THRESHOLD:
            self.current_waypoint_index += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_index}/{len(self.waypoints)}')
            return
        
        # Determine speed based on distance
        if distance_xy < self.WAYPOINT_THRESHOLD * 2:
            speed = self.APPROACH_SPEED
        else:
            speed = self.CRUISE_SPEED
        
        # Calculate velocity components (WORLD FRAME)
        if distance_xy > 0.1:
            vx = (dx / distance_xy) * speed
            vy = (dy / distance_xy) * speed
        else:
            vx = 0.0
            vy = 0.0
        
        # Altitude control
        vz = dz * 0.5
        vz = np.clip(vz, -1.0, 1.0)
        
        # Send velocity command (NO YAW!)
        self.send_velocity_command(vx, vy, vz)
        
        # Log occasionally
        if self.get_clock().now().nanoseconds % 2000000000 < 100000000:  # Every 2 seconds
            self.get_logger().info(
                f'WP {self.current_waypoint_index+1}/{len(self.waypoints)}, '
                f'Dist: {distance_xy:.1f}m, '
                f'Vel: ({vx:.1f}, {vy:.1f}, {vz:.1f})'
            )
    
    def publish_status(self, status):
        """Publish pattern execution status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FixedMovementExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Fixed Movement Executor')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()