#!/usr/bin/env python3
"""
Smooth Banking Pattern Executor
Simplified approach with stable control
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String
import numpy as np
import json
import math

class SmoothBankingExecutor(Node):
    def __init__(self):
        super().__init__('smooth_banking_executor')
        
        # Control parameters - simplified for stability
        self.WAYPOINT_THRESHOLD = 2.5      # m - distance to consider waypoint reached
        self.CRUISE_SPEED = 3.0            # m/s - normal speed
        self.TURN_SPEED = 1.5              # m/s - speed in turns
        self.MAX_YAW_RATE = 0.3            # rad/s - maximum yaw rate
        self.LOOKAHEAD_DISTANCE = 4.0      # m - fixed lookahead
        
        # Pattern state
        self.waypoints = []
        self.current_waypoint_index = 0
        self.pattern_active = False
        
        # Current state
        self.current_pose = None
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.last_cmd_time = 0
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.setpoint_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
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
        
        # Control timer - 10Hz for stability
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Smooth Banking Executor initialized')
    
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
            
            # Convert waypoints to simple list format
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
                self.get_logger().info('Starting smooth pattern execution')
                self.publish_status('executing')
            else:
                self.get_logger().warn('No waypoints loaded')
        
        elif command == 'stop':
            self.pattern_active = False
            self.get_logger().info('Stopping pattern execution')
            self.publish_status('stopped')
            # Send stop command
            self.send_stop_command()
        
        elif command == 'pause':
            self.pattern_active = False
            self.get_logger().info('Pausing pattern execution')
            self.publish_status('paused')
    
    def send_stop_command(self):
        """Send zero velocity command to stop drone"""
        cmd = PositionTarget()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "map"
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # Stop all motion
        cmd.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_YAW_RATE
        )
        
        cmd.velocity.x = 0.0
        cmd.velocity.y = 0.0
        cmd.velocity.z = 0.0
        
        self.setpoint_pub.publish(cmd)
    
    def get_current_yaw(self):
        """Extract yaw from quaternion"""
        if not self.current_pose:
            return 0.0
        
        q = self.current_pose.orientation
        # Yaw from quaternion
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return yaw
    
    def control_loop(self):
        """Main control loop - simplified for stability"""
        if not self.pattern_active or not self.current_pose or not self.waypoints:
            return
        
        # Get current position
        current_x = self.current_pose.position.x
        current_y = self.current_pose.position.y
        current_z = self.current_pose.position.z
        
        # Check if we've reached all waypoints
        if self.current_waypoint_index >= len(self.waypoints):
            self.pattern_active = False
            self.publish_status('complete')
            self.get_logger().info('Pattern complete!')
            self.send_stop_command()
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
        
        # Calculate desired heading to target
        desired_yaw = math.atan2(dy, dx)
        current_yaw = self.get_current_yaw()
        
        # Calculate yaw error
        yaw_error = desired_yaw - current_yaw
        # Normalize to [-pi, pi]
        while yaw_error > math.pi:
            yaw_error -= 2 * math.pi
        while yaw_error < -math.pi:
            yaw_error += 2 * math.pi
        
        # Determine if we need to turn or can move forward
        if abs(yaw_error) > 0.3:  # ~17 degrees
            # Need to turn - reduce forward speed
            speed = self.TURN_SPEED
            yaw_rate = np.clip(yaw_error * 1.0, -self.MAX_YAW_RATE, self.MAX_YAW_RATE)
        else:
            # Facing right direction - full speed
            speed = self.CRUISE_SPEED
            yaw_rate = np.clip(yaw_error * 0.5, -self.MAX_YAW_RATE, self.MAX_YAW_RATE)
        
        # Calculate velocity components
        # Move towards target with current speed
        if distance > 0:
            vx = (dx / distance) * speed
            vy = (dy / distance) * speed
        else:
            vx = 0.0
            vy = 0.0
        
        # Altitude control
        vz = (target_z - current_z) * 0.5
        vz = np.clip(vz, -1.0, 1.0)
        
        # Create control command
        cmd = PositionTarget()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "map"
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # Use velocity control with yaw rate
        cmd.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW  # Ignore yaw position, use yaw rate
        )
        
        cmd.velocity.x = vx
        cmd.velocity.y = vy
        cmd.velocity.z = vz
        cmd.yaw_rate = yaw_rate
        
        # Publish command
        self.setpoint_pub.publish(cmd)
        
        # Log occasionally
        current_time = self.get_clock().now().nanoseconds
        if current_time - self.last_cmd_time > 2000000000:  # Every 2 seconds
            self.last_cmd_time = current_time
            self.get_logger().info(
                f'WP {self.current_waypoint_index+1}/{len(self.waypoints)}, '
                f'Dist: {distance:.1f}m, '
                f'Speed: {speed:.1f}m/s, '
                f'Yaw err: {math.degrees(yaw_error):.1f}°'
            )
    
    def publish_status(self, status):
        """Publish pattern execution status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SmoothBankingExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Smooth Banking Executor')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()