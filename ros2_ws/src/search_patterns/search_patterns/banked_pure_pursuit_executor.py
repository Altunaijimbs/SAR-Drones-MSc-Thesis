#!/usr/bin/env python3
"""
Banked Pure Pursuit Pattern Executor
Implements smooth, banking turns with no side-sliding using pure pursuit algorithm
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
from enum import Enum

class FlightState(Enum):
    IDLE = 0
    APPROACHING = 1
    TURNING = 2
    CRUISING = 3
    DECELERATING = 4

class BankedPurePursuitExecutor(Node):
    def __init__(self):
        super().__init__('banked_pure_pursuit_executor')
        
        # Pure Pursuit Parameters
        self.LOOKAHEAD_TIME = 1.5  # seconds - how far ahead to look
        self.MIN_LOOKAHEAD = 2.0   # minimum lookahead distance (m)
        self.MAX_LOOKAHEAD = 8.0   # maximum lookahead distance (m)
        
        # Flight Parameters
        self.CRUISE_SPEED = 4.0     # m/s - normal flight speed
        self.TURN_SPEED = 2.5       # m/s - speed during turns
        self.APPROACH_SPEED = 1.5   # m/s - speed when approaching waypoint
        self.MAX_BANK_ANGLE = 30.0  # degrees - maximum bank angle
        self.GRAVITY = 9.81         # m/s^2
        
        # Waypoint Parameters
        self.WAYPOINT_THRESHOLD = 2.0  # m - distance to consider waypoint reached
        self.TURN_ANTICIPATION = 3.0   # m - start turn this far before waypoint
        self.SMOOTHING_FACTOR = 0.3    # for trajectory smoothing
        
        # Pattern execution state
        self.waypoints = []
        self.current_waypoint_index = 0
        self.pattern_active = False
        self.flight_state = FlightState.IDLE
        
        # Current state
        self.current_pose = None
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.target_altitude = 5.0
        
        # Pure pursuit target
        self.lookahead_point = None
        self.desired_heading = 0.0
        self.desired_bank = 0.0
        
        # Path interpolation
        self.interpolated_path = []
        self.path_distances = []
        
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
        
        # Control timer - 20Hz for smooth control
        self.control_timer = self.create_timer(0.05, self.control_loop)
        
        self.get_logger().info('Banked Pure Pursuit Executor initialized')
        self.get_logger().info('Using dynamic lookahead with banking turns')
    
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
            
            # Convert waypoints from dict format to list format
            self.waypoints = []
            for wp in raw_waypoints:
                if isinstance(wp, dict):
                    # Dictionary format from pattern_generator
                    self.waypoints.append([wp['x'], wp['y'], wp['z']])
                else:
                    # Already in list format
                    self.waypoints.append(wp)
            
            self.current_waypoint_index = 0
            
            if len(self.waypoints) > 0:
                # Generate interpolated path for smooth following
                self.generate_smooth_path()
                
                self.get_logger().info(f'Received {len(self.waypoints)} waypoints')
                self.get_logger().info(f'Generated smooth path with {len(self.interpolated_path)} points')
                
                # Update altitude target
                if self.waypoints:
                    self.target_altitude = self.waypoints[0][2]
        except Exception as e:
            self.get_logger().error(f'Failed to parse waypoints: {e}')
    
    def control_callback(self, msg):
        """Handle pattern control commands"""
        command = msg.data.lower()
        
        if command == 'start':
            if self.waypoints:
                self.pattern_active = True
                self.current_waypoint_index = 0
                self.flight_state = FlightState.APPROACHING
                self.get_logger().info('Starting pattern execution with pure pursuit')
                self.publish_status('executing')
            else:
                self.get_logger().warn('No waypoints loaded')
        
        elif command == 'stop':
            self.pattern_active = False
            self.flight_state = FlightState.IDLE
            self.get_logger().info('Stopping pattern execution')
            self.publish_status('stopped')
        
        elif command == 'pause':
            self.pattern_active = False
            self.flight_state = FlightState.IDLE
            self.get_logger().info('Pausing pattern execution')
            self.publish_status('paused')
        
        elif command == 'resume':
            if self.waypoints:
                self.pattern_active = True
                self.flight_state = FlightState.APPROACHING
                self.get_logger().info('Resuming pattern execution')
                self.publish_status('executing')
    
    def generate_smooth_path(self):
        """Generate interpolated path with smooth corners"""
        if len(self.waypoints) < 2:
            self.interpolated_path = self.waypoints.copy()
            return
        
        self.interpolated_path = []
        
        for i in range(len(self.waypoints)):
            prev_wp = self.waypoints[i-1] if i > 0 else self.waypoints[i]
            curr_wp = self.waypoints[i]
            next_wp = self.waypoints[(i+1) % len(self.waypoints)]
            
            # Add arc transitions at corners
            if i > 0 or (i == 0 and len(self.waypoints) > 2):
                # Calculate corner angle
                v1 = np.array([curr_wp[0], curr_wp[1]]) - np.array([prev_wp[0], prev_wp[1]])
                v2 = np.array([next_wp[0], next_wp[1]]) - np.array([curr_wp[0], curr_wp[1]])
                
                if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                    v1_norm = v1 / np.linalg.norm(v1)
                    v2_norm = v2 / np.linalg.norm(v2)
                    
                    # Calculate turn angle
                    cos_angle = np.clip(np.dot(v1_norm, v2_norm), -1, 1)
                    turn_angle = math.acos(cos_angle)
                    
                    # If significant turn, add arc waypoints
                    if turn_angle > math.radians(30):
                        # Calculate arc radius based on speed and max bank
                        turn_radius = self.calculate_turn_radius(self.TURN_SPEED)
                        
                        # Add approach point
                        approach_dist = min(turn_radius, np.linalg.norm(v1) * 0.3)
                        approach_point = np.array([curr_wp[0], curr_wp[1]]) - v1_norm * approach_dist
                        self.interpolated_path.append([approach_point[0], approach_point[1], curr_wp[2]])
                        
                        # Add arc points for smooth turn
                        num_arc_points = max(3, int(turn_angle * 3))
                        for j in range(num_arc_points):
                            t = (j + 1) / (num_arc_points + 1)
                            # Slerp-like interpolation for smooth arc
                            interp_dir = self.slerp_2d(v1_norm, v2_norm, t)
                            arc_point = np.array([curr_wp[0], curr_wp[1]]) + interp_dir * turn_radius * 0.5
                            self.interpolated_path.append([arc_point[0], arc_point[1], curr_wp[2]])
                        
                        # Add exit point
                        exit_dist = min(turn_radius, np.linalg.norm(v2) * 0.3)
                        exit_point = np.array([curr_wp[0], curr_wp[1]]) + v2_norm * exit_dist
                        self.interpolated_path.append([exit_point[0], exit_point[1], curr_wp[2]])
                    else:
                        # Small turn, just add the waypoint
                        self.interpolated_path.append(curr_wp)
                else:
                    self.interpolated_path.append(curr_wp)
            else:
                self.interpolated_path.append(curr_wp)
        
        # Calculate cumulative distances along path
        self.path_distances = [0.0]
        for i in range(1, len(self.interpolated_path)):
            dist = np.linalg.norm(
                np.array([self.interpolated_path[i][0], self.interpolated_path[i][1]]) - 
                np.array([self.interpolated_path[i-1][0], self.interpolated_path[i-1][1]])
            )
            self.path_distances.append(self.path_distances[-1] + dist)
    
    def slerp_2d(self, v1, v2, t):
        """2D spherical linear interpolation for smooth direction changes"""
        # Convert to angles
        angle1 = math.atan2(v1[1], v1[0])
        angle2 = math.atan2(v2[1], v2[0])
        
        # Handle angle wrap-around
        diff = angle2 - angle1
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
        
        # Interpolate angle
        angle = angle1 + diff * t
        
        # Convert back to unit vector
        return np.array([math.cos(angle), math.sin(angle)])
    
    def calculate_turn_radius(self, velocity):
        """Calculate turn radius for given velocity and max bank angle"""
        # R = V²/(g × tan(φ))
        max_bank_rad = math.radians(self.MAX_BANK_ANGLE)
        return (velocity ** 2) / (self.GRAVITY * math.tan(max_bank_rad))
    
    def calculate_bank_angle(self, velocity, turn_radius):
        """Calculate required bank angle for coordinated turn"""
        if turn_radius <= 0:
            return 0.0
        
        # φ = arctan(V²/(g × R))
        bank_rad = math.atan((velocity ** 2) / (self.GRAVITY * turn_radius))
        bank_deg = math.degrees(bank_rad)
        
        # Limit to max bank angle
        return np.clip(bank_deg, -self.MAX_BANK_ANGLE, self.MAX_BANK_ANGLE)
    
    def find_lookahead_point(self):
        """Find the lookahead point using pure pursuit algorithm"""
        if not self.current_pose or not self.interpolated_path:
            return None
        
        current_pos = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y
        ])
        
        # Calculate dynamic lookahead distance
        speed = np.linalg.norm(self.current_velocity[0:2])
        lookahead_dist = np.clip(
            speed * self.LOOKAHEAD_TIME,
            self.MIN_LOOKAHEAD,
            self.MAX_LOOKAHEAD
        )
        
        # Find closest point on path
        min_dist = float('inf')
        closest_idx = 0
        for i, wp in enumerate(self.interpolated_path):
            dist = np.linalg.norm(current_pos - np.array([wp[0], wp[1]]))
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # Search forward for lookahead point
        for i in range(closest_idx, len(self.interpolated_path)):
            wp = self.interpolated_path[i]
            dist = np.linalg.norm(current_pos - np.array([wp[0], wp[1]]))
            
            if dist >= lookahead_dist:
                # Interpolate between this and previous point
                if i > 0:
                    prev_wp = self.interpolated_path[i-1]
                    prev_dist = np.linalg.norm(current_pos - np.array([prev_wp[0], prev_wp[1]]))
                    
                    # Linear interpolation
                    t = (lookahead_dist - prev_dist) / (dist - prev_dist)
                    t = np.clip(t, 0, 1)
                    
                    lookahead_x = prev_wp[0] + t * (wp[0] - prev_wp[0])
                    lookahead_y = prev_wp[1] + t * (wp[1] - prev_wp[1])
                    lookahead_z = prev_wp[2] + t * (wp[2] - prev_wp[2])
                    
                    return [lookahead_x, lookahead_y, lookahead_z]
                else:
                    return wp
        
        # If we're near the end, return the last waypoint
        if self.interpolated_path:
            return self.interpolated_path[-1]
        
        return None
    
    def calculate_steering_command(self):
        """Calculate velocity and attitude commands using pure pursuit"""
        if not self.current_pose or not self.lookahead_point:
            return None
        
        current_pos = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_pose.position.z
        ])
        
        # Vector to lookahead point
        to_target = np.array(self.lookahead_point) - current_pos
        distance = np.linalg.norm(to_target[0:2])
        
        if distance < 0.1:
            return None
        
        # Calculate desired heading
        self.desired_heading = math.atan2(to_target[1], to_target[0])
        
        # Calculate current heading from velocity or quaternion
        if np.linalg.norm(self.current_velocity[0:2]) > 0.5:
            current_heading = math.atan2(self.current_velocity[1], self.current_velocity[0])
        else:
            # Use pose quaternion
            q = self.current_pose.orientation
            current_heading = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
        
        # Calculate heading error
        heading_error = self.desired_heading - current_heading
        
        # Normalize to [-pi, pi]
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Calculate turn radius from pure pursuit geometry
        if abs(heading_error) > 0.01:
            # Using pure pursuit formula: R = L / (2 * sin(α))
            # where L is lookahead distance and α is heading error
            turn_radius = distance / (2 * math.sin(heading_error))
            turn_radius = abs(turn_radius)
        else:
            turn_radius = 1000.0  # Very large radius for straight flight
        
        # Determine flight state and speed
        speed = self.CRUISE_SPEED
        
        # Check if we're approaching a sharp turn
        if self.is_approaching_turn():
            speed = self.TURN_SPEED
            self.flight_state = FlightState.TURNING
        elif distance < self.WAYPOINT_THRESHOLD * 2:
            speed = self.APPROACH_SPEED
            self.flight_state = FlightState.APPROACHING
        else:
            self.flight_state = FlightState.CRUISING
        
        # Calculate bank angle for coordinated turn
        self.desired_bank = self.calculate_bank_angle(speed, turn_radius)
        if heading_error < 0:
            self.desired_bank = -self.desired_bank
        
        # Calculate velocity components
        velocity_x = speed * math.cos(self.desired_heading)
        velocity_y = speed * math.sin(self.desired_heading)
        velocity_z = (self.lookahead_point[2] - current_pos[2]) * 0.5  # Proportional altitude control
        
        # Create position target message
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
            PositionTarget.IGNORE_YAW
        )
        
        cmd.velocity.x = velocity_x
        cmd.velocity.y = velocity_y
        cmd.velocity.z = velocity_z
        
        # Calculate yaw rate based on heading error and speed
        # Higher speed = lower yaw rate for same turn radius
        max_yaw_rate = 0.5  # rad/s
        cmd.yaw_rate = np.clip(heading_error * 2.0, -max_yaw_rate, max_yaw_rate)
        
        # Add simulated bank by including lateral acceleration
        # This creates coordinated turn feeling
        if abs(self.desired_bank) > 1.0:
            # Lateral acceleration for coordinated turn
            lateral_accel = self.GRAVITY * math.tan(math.radians(self.desired_bank))
            
            # Apply as acceleration perpendicular to velocity
            accel_dir = np.array([-math.sin(self.desired_heading), math.cos(self.desired_heading), 0])
            
            # Note: PX4 may not respond to acceleration commands
            # but this shows the intent for coordinated turns
            cmd.type_mask &= ~(PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY)
            cmd.acceleration_or_force.x = accel_dir[0] * lateral_accel * 0.1  # Scale down
            cmd.acceleration_or_force.y = accel_dir[1] * lateral_accel * 0.1
        
        return cmd
    
    def is_approaching_turn(self):
        """Check if approaching a turn that requires speed reduction"""
        if not self.current_pose or not self.interpolated_path:
            return False
        
        current_pos = np.array([
            self.current_pose.position.x,
            self.current_pose.position.y
        ])
        
        # Look ahead for sharp turns
        check_distance = self.TURN_ANTICIPATION + self.MIN_LOOKAHEAD
        
        for i in range(len(self.interpolated_path) - 2):
            wp1 = np.array([self.interpolated_path[i][0], self.interpolated_path[i][1]])
            wp2 = np.array([self.interpolated_path[i+1][0], self.interpolated_path[i+1][1]])
            wp3 = np.array([self.interpolated_path[i+2][0], self.interpolated_path[i+2][1]])
            
            # Check if this segment is within check distance
            dist_to_wp2 = np.linalg.norm(current_pos - wp2)
            if dist_to_wp2 > check_distance:
                continue
            
            if dist_to_wp2 < check_distance:
                # Calculate turn angle
                v1 = wp2 - wp1
                v2 = wp3 - wp2
                
                if np.linalg.norm(v1) > 0 and np.linalg.norm(v2) > 0:
                    v1_norm = v1 / np.linalg.norm(v1)
                    v2_norm = v2 / np.linalg.norm(v2)
                    
                    cos_angle = np.clip(np.dot(v1_norm, v2_norm), -1, 1)
                    turn_angle = math.acos(cos_angle)
                    
                    # Sharp turn ahead
                    if turn_angle > math.radians(45):
                        return True
        
        return False
    
    def control_loop(self):
        """Main control loop"""
        if not self.pattern_active or not self.current_pose:
            return
        
        # Find lookahead point
        self.lookahead_point = self.find_lookahead_point()
        
        if not self.lookahead_point:
            return
        
        # Calculate and send control command
        cmd = self.calculate_steering_command()
        if cmd:
            self.setpoint_pub.publish(cmd)
            
            # Log state occasionally
            if self.get_clock().now().nanoseconds % 1000000000 < 50000000:  # Once per second
                speed = np.linalg.norm(self.current_velocity[0:2])
                self.get_logger().info(
                    f'State: {self.flight_state.name}, '
                    f'Speed: {speed:.1f}m/s, '
                    f'Bank: {self.desired_bank:.1f}°, '
                    f'Heading err: {math.degrees(self.desired_heading):.1f}°'
                )
        
        # Check if pattern is complete
        if self.lookahead_point == self.interpolated_path[-1]:
            current_pos = np.array([
                self.current_pose.position.x,
                self.current_pose.position.y
            ])
            final_wp = np.array([self.interpolated_path[-1][0], self.interpolated_path[-1][1]])
            
            if np.linalg.norm(current_pos - final_wp) < self.WAYPOINT_THRESHOLD:
                self.pattern_active = False
                self.flight_state = FlightState.IDLE
                self.publish_status('complete')
                self.get_logger().info('Pattern execution complete!')
    
    def publish_status(self, status):
        """Publish pattern execution status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = BankedPurePursuitExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Banked Pure Pursuit Executor')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()