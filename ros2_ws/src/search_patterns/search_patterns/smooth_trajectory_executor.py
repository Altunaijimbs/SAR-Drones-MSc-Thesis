#!/usr/bin/env python3
"""
Smooth Trajectory Executor - Inspired by AeroStack2's approach
Uses continuous position setpoints with velocity constraints
Prevents overshooting through proper trajectory planning
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Point
from mavros_msgs.msg import PositionTarget
from std_msgs.msg import String, Header
import json
import math
import numpy as np
import threading
import time

class SmoothTrajectoryExecutor(Node):
    def __init__(self):
        super().__init__('smooth_trajectory_executor')
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers - Direct MAVROS control
        self.setpoint_raw_pub = self.create_publisher(
            PositionTarget, 
            '/mavros/setpoint_raw/local', 
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
        
        # Trajectory state
        self.trajectory_start_pos = None
        self.trajectory_start_time = None
        self.trajectory_duration = 0.0
        
        # Control parameters (inspired by AeroStack2)
        self.position_threshold = 0.5  # Much tighter threshold
        self.max_speed = 3.0  # m/s
        self.max_acceleration = 2.0  # m/s^2
        self.approach_distance = 2.0  # Start slowing down at this distance
        self.min_speed = 0.5  # Minimum speed when approaching
        
        # Control loop at 20Hz (like AeroStack2)
        self.control_timer = self.create_timer(0.05, self.control_loop)
        self.current_target = None
        self.target_velocity = np.zeros(3)
        
        # Status timer
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Smooth Trajectory Executor initialized - AeroStack2 inspired')
    
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
        """Load waypoints and compute smooth trajectory"""
        try:
            data = json.loads(msg.data)
            self.waypoints = data['waypoints']
            self.current_waypoint_index = 0
            
            # Precompute trajectory segments
            self.compute_trajectory_segments()
            
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints with smooth trajectory')
            
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {e}')
    
    def compute_trajectory_segments(self):
        """Compute trajectory parameters for each segment"""
        if not self.current_pose or not self.waypoints:
            return
        
        # Add velocity and duration to each waypoint
        prev_pos = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])
        
        for waypoint in self.waypoints:
            curr_pos = np.array([waypoint['x'], waypoint['y'], waypoint['z']])
            distance = np.linalg.norm(curr_pos - prev_pos)
            
            # Compute duration based on trapezoidal velocity profile
            # (acceleration phase + constant speed + deceleration phase)
            accel_distance = (self.max_speed ** 2) / (2 * self.max_acceleration)
            
            if distance < 2 * accel_distance:
                # Short segment - triangular profile
                waypoint['duration'] = 2 * math.sqrt(distance / self.max_acceleration)
            else:
                # Long segment - trapezoidal profile
                constant_distance = distance - 2 * accel_distance
                waypoint['duration'] = (2 * self.max_speed / self.max_acceleration + 
                                       constant_distance / self.max_speed)
            
            # Store segment info
            waypoint['distance'] = distance
            waypoint['start_pos'] = prev_pos.tolist()
            
            prev_pos = curr_pos
    
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
        self.trajectory_start_time = self.get_clock().now()
        self.trajectory_start_pos = [
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ] if self.current_pose else [0, 0, 0]
        
        self.get_logger().info('Started smooth trajectory execution')
    
    def stop_execution(self):
        """Stop pattern execution and hover"""
        self.executing = False
        self.current_target = None
        self.target_velocity = np.zeros(3)
        self.get_logger().info('Stopped pattern execution')
    
    def pause_execution(self):
        """Pause at current position"""
        if self.executing:
            self.executing = False
            self.get_logger().info('Paused pattern execution')
    
    def resume_execution(self):
        """Resume from current waypoint"""
        if not self.executing and self.waypoints:
            self.executing = True
            self.trajectory_start_time = self.get_clock().now()
            self.trajectory_start_pos = [
                self.current_pose.pose.position.x,
                self.current_pose.pose.position.y,
                self.current_pose.pose.position.z
            ] if self.current_pose else [0, 0, 0]
            self.get_logger().info('Resumed pattern execution')
    
    def reset_execution(self):
        """Reset to first waypoint"""
        self.stop_execution()
        self.current_waypoint_index = 0
        self.get_logger().info('Reset pattern execution')
    
    def compute_trajectory_point(self, start, end, duration, elapsed_time):
        """
        Compute position and velocity at current time using trapezoidal profile
        Inspired by AeroStack2's trajectory generation
        """
        if elapsed_time >= duration:
            return end, np.zeros(3)
        
        t = elapsed_time
        T = duration
        
        # Trapezoidal velocity profile parameters
        accel_time = min(self.max_speed / self.max_acceleration, T / 2)
        
        start_np = np.array(start)
        end_np = np.array(end)
        direction = (end_np - start_np) / np.linalg.norm(end_np - start_np)
        total_distance = np.linalg.norm(end_np - start_np)
        
        if t < accel_time:
            # Acceleration phase
            s = 0.5 * self.max_acceleration * t * t
            v = self.max_acceleration * t
        elif t < T - accel_time:
            # Constant velocity phase
            s = 0.5 * self.max_acceleration * accel_time * accel_time + \
                self.max_speed * (t - accel_time)
            v = self.max_speed
        else:
            # Deceleration phase
            t_decel = t - (T - accel_time)
            s = total_distance - 0.5 * self.max_acceleration * (accel_time - t_decel) ** 2
            v = self.max_speed - self.max_acceleration * t_decel
        
        # Ensure we don't overshoot
        s = min(s, total_distance)
        
        position = start_np + direction * s
        velocity = direction * v
        
        return position, velocity
    
    def control_loop(self):
        """Main control loop - runs at 20Hz"""
        if not self.executing or not self.current_pose:
            # Send hover command if not executing
            if self.current_pose:
                self.send_position_setpoint(
                    self.current_pose.pose.position,
                    np.zeros(3),
                    self.current_yaw
                )
            return
        
        if self.current_waypoint_index >= len(self.waypoints):
            # Pattern completed
            self.executing = False
            self.get_logger().info('Pattern execution completed')
            return
        
        # Get current waypoint
        waypoint = self.waypoints[self.current_waypoint_index]
        
        # Compute elapsed time for this segment
        elapsed = (self.get_clock().now() - self.trajectory_start_time).nanoseconds / 1e9
        
        # Compute trajectory point
        target_pos, target_vel = self.compute_trajectory_point(
            waypoint.get('start_pos', self.trajectory_start_pos),
            [waypoint['x'], waypoint['y'], waypoint['z']],
            waypoint.get('duration', 5.0),
            elapsed
        )
        
        # Calculate desired yaw (face direction of travel)
        if np.linalg.norm(target_vel[:2]) > 0.1:
            target_yaw = math.atan2(target_vel[1], target_vel[0])
        else:
            target_yaw = self.current_yaw
        
        # Send position setpoint with velocity feedforward
        target_point = Point()
        target_point.x = target_pos[0]
        target_point.y = target_pos[1]
        target_point.z = target_pos[2]
        
        self.send_position_setpoint(target_point, target_vel, target_yaw)
        
        # Check if reached waypoint
        current_pos = np.array([
            self.current_pose.pose.position.x,
            self.current_pose.pose.position.y,
            self.current_pose.pose.position.z
        ])
        
        waypoint_pos = np.array([waypoint['x'], waypoint['y'], waypoint['z']])
        distance = np.linalg.norm(waypoint_pos - current_pos)
        
        if distance < self.position_threshold or elapsed > waypoint.get('duration', 5.0):
            self.get_logger().info(
                f'Reached waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}'
            )
            self.current_waypoint_index += 1
            self.trajectory_start_time = self.get_clock().now()
            self.trajectory_start_pos = current_pos.tolist()
    
    def send_position_setpoint(self, position, velocity, yaw):
        """Send position setpoint with velocity feedforward"""
        msg = PositionTarget()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        
        # Position + velocity control (like AeroStack2)
        msg.type_mask = (
            PositionTarget.IGNORE_AFX | 
            PositionTarget.IGNORE_AFY | 
            PositionTarget.IGNORE_AFZ
        )
        
        # Position setpoint
        msg.position.x = float(position.x)
        msg.position.y = float(position.y)
        msg.position.z = float(position.z)
        
        # Velocity feedforward
        msg.velocity.x = float(velocity[0])
        msg.velocity.y = float(velocity[1])
        msg.velocity.z = float(velocity[2])
        
        # Yaw setpoint
        msg.yaw = float(yaw)
        
        self.setpoint_raw_pub.publish(msg)
    
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
    node = SmoothTrajectoryExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()