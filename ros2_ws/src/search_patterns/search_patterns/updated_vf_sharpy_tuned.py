#!/usr/bin/env python3
"""
Updated Sharp Corner VF Path Follower - TUNED VERSION
Stop-turn-go with robust corner handling and no yaw while translating.
Performance-tuned parameters for faster execution with smooth corners.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
import numpy as np
import math
import json
from enum import Enum

class FlightState(Enum):
    IDLE = 0
    APPROACH_CORNER = 1  # Slowing down for corner
    AT_CORNER = 2        # Stopped at corner waypoint
    TURNING = 3          # Pure rotation at corner
    STRAIGHT = 4         # Moving along straight segment

class UpdatedVFSharpyTuned(Node):
    def __init__(self):
        super().__init__('updated_vf_sharpy_tuned')

        # QoS for MAVROS-like topics
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/search_pattern/velocity_command', 10)
        self.state_pub = self.create_publisher(String, '/vf_state', 10)
        self.status_pub = self.create_publisher(String, '/pattern_status', 10)

        # Subscribers
        self.pose_sub = self.create_subscription(PoseStamped, '/mavros/local_position/pose', self.pose_callback, qos_profile)
        self.waypoints_sub = self.create_subscription(String, '/pattern_waypoints', self.waypoints_callback, 10)
        self.control_sub = self.create_subscription(String, '/pattern_control', self.control_callback, 10)

        # State variables
        self.current_pose = None
        self.current_yaw = 0.0
        self.waypoints = []
        self.current_waypoint_idx = 0
        self.state = FlightState.IDLE
        self.executing = False

        # ===== TUNED CONTROL PARAMETERS =====
        # Higher cruise speed for efficiency on straights
        self.cruise_speed = 3.0                      # m/s on straights (was 2.0)
        
        # Keep approach speed low for precision
        self.approach_speed = 0.6                    # m/s when near a corner (was 0.5)
        
        # Start slowing earlier for smoother deceleration
        self.corner_threshold = 2.2                  # m: start slowing when this close (was 1.5)
        
        # Slightly larger arrival threshold to prevent oscillation
        self.arrival_threshold = 0.9                 # m: consider waypoint reached (was 0.6)
        
        # Keep corner detection the same
        self.corner_angle_threshold = math.radians(45)   # detect corners if turn > 45 deg
        
        # Same rotation speed
        self.yaw_rate = 1.0                          # rad/s during TURNING
        
        # Slightly more relaxed yaw tolerance for smoother transitions
        self.yaw_tolerance = math.radians(13)        # deg-equivalent: acceptable heading error (was 10)
        
        # Slightly higher threshold to avoid unnecessary turns
        self.yaw_turn_enter = math.radians(22)       # enter TURNING if error exceeds this (was 18)

        # Corner detection cache
        self.corner_angles = []
        self.is_corner = []

        # Control timer
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info("Updated VF Sharpy TUNED - Optimized parameters for speed & smoothness")
        self.get_logger().info(f"  Cruise: {self.cruise_speed} m/s, Approach: {self.approach_speed} m/s")
        self.get_logger().info(f"  Corner threshold: {self.corner_threshold} m, Arrival: {self.arrival_threshold} m")
        self.get_logger().info(f"  Yaw tolerance: {math.degrees(self.yaw_tolerance):.1f}°, Turn enter: {math.degrees(self.yaw_turn_enter):.1f}°")

    # ----------------------- Callbacks -----------------------

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose
        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = float(np.arctan2(siny_cosp, cosy_cosp))

    def waypoints_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            wps = data['waypoints']
            self.waypoints = [(w['x'], w['y'], w['z']) for w in wps]
            self.current_waypoint_idx = 0
            self.detect_corners()
            self.get_logger().info(f"🎯 Received {len(self.waypoints)} waypoints; corners={sum(self.is_corner)}")
            # Debug: Show first few waypoints
            for i in range(min(3, len(self.waypoints))):
                self.get_logger().info(f"   Waypoint {i}: ({self.waypoints[i][0]:.2f}, {self.waypoints[i][1]:.2f})")
        except Exception as e:
            self.get_logger().error(f"Error parsing waypoints JSON: {e}")

    def control_callback(self, msg: String):
        cmd = msg.data.lower().strip()
        if cmd == 'start':
            if self.waypoints:
                self.executing = True
                self.current_waypoint_idx = 0
                self.state = FlightState.STRAIGHT
                self.publish_state("STRAIGHT")
                self.get_logger().info(f"🚀 Pattern execution START with {len(self.waypoints)} waypoints")
                # Publish waypoint 0 status to mark true pattern start
                self.publish_status(f"Waypoint 0/{len(self.waypoints)}")
        elif cmd == 'stop':
            self.executing = False
            self.state = FlightState.IDLE
            self.stop()
            self.publish_state("IDLE")
        elif cmd == 'pause':
            self.executing = False
            self.publish_state("PAUSE")
            self.stop()
        elif cmd == 'resume':
            if self.waypoints:
                self.executing = True
                if self.state == FlightState.IDLE:
                    self.state = FlightState.STRAIGHT
                self.publish_state("RESUME")

    # ----------------------- Corner Detection -----------------------

    def detect_corners(self):
        self.corner_angles = []
        self.is_corner = []
        n = len(self.waypoints)
        if n < 3:
            self.is_corner = [True] if n == 1 else [True, False]  # First waypoint is always a corner
            return

        for i in range(n):
            if i == 0:  # FIRST WAYPOINT IS ALWAYS A CORNER - ensures proper initial orientation
                self.corner_angles.append(0.0)
                self.is_corner.append(True)
                self.get_logger().info("First waypoint marked as corner for initial orientation")
                continue
            elif i == n - 1:
                self.corner_angles.append(0.0)
                self.is_corner.append(False)
                continue

            v1 = np.array([self.waypoints[i][0] - self.waypoints[i-1][0],
                           self.waypoints[i][1] - self.waypoints[i-1][1]])
            v2 = np.array([self.waypoints[i+1][0] - self.waypoints[i][0],
                           self.waypoints[i+1][1] - self.waypoints[i][1]])

            L1 = np.linalg.norm(v1)
            L2 = np.linalg.norm(v2)
            if L1 > 1e-6 and L2 > 1e-6:
                v1 /= L1
                v2 /= L2
                dotp = float(np.clip(np.dot(v1, v2), -1.0, 1.0))
                angle_change = float(np.arccos(dotp))
                self.corner_angles.append(angle_change)
                is_corner = angle_change > self.corner_angle_threshold
                self.is_corner.append(is_corner)
                if is_corner:
                    self.get_logger().info(f"Corner @ wp {i}: {math.degrees(angle_change):.1f}°")
            else:
                self.corner_angles.append(0.0)
                self.is_corner.append(False)

    # ----------------------- Control Loop -----------------------

    def control_loop(self):
        if not self.current_pose or not self.executing or not self.waypoints:
            self.stop()
            return

        if self.current_waypoint_idx >= len(self.waypoints):
            self.executing = False
            self.state = FlightState.IDLE
            self.get_logger().warn(f"⚠️ Pattern complete: idx={self.current_waypoint_idx} >= len={len(self.waypoints)}")
            self.publish_status("Pattern complete")
            self.stop()
            return

        # Current target waypoint
        tx, ty, tz = self.waypoints[self.current_waypoint_idx]

        # Vector to target
        dx = tx - self.current_pose.position.x
        dy = ty - self.current_pose.position.y
        dist = float(np.hypot(dx, dy))

        # Desired yaw to current target
        desired_yaw = float(np.arctan2(dy, dx))
        yaw_error = self.normalize_angle(desired_yaw - self.current_yaw)

        # Is current target a corner?
        is_corner_wp = (self.current_waypoint_idx < len(self.is_corner) and self.is_corner[self.current_waypoint_idx])

        # State machine
        if self.state == FlightState.STRAIGHT:
            if dist < self.arrival_threshold:
                # Reached waypoint
                if is_corner_wp:
                    self.state = FlightState.AT_CORNER
                    self.publish_state("AT_CORNER")
                    self.stop()
                else:
                    self.current_waypoint_idx += 1
                    self.get_logger().info(f"📍 Advanced to waypoint {self.current_waypoint_idx}/{len(self.waypoints)}")
                    self.publish_status(f"Waypoint {self.current_waypoint_idx}/{len(self.waypoints)}")
            elif is_corner_wp and dist < self.corner_threshold:
                self.state = FlightState.APPROACH_CORNER
                self.publish_state("APPROACH_CORNER")
                self.move_slow(dx, dy)   # yaw_rate = 0.0 while translating
            else:
                self.move_straight(dx, dy)  # yaw_rate = 0.0 while translating
                self.publish_state("STRAIGHT")

        elif self.state == FlightState.APPROACH_CORNER:
            if dist < self.arrival_threshold:
                self.state = FlightState.AT_CORNER
                self.publish_state("AT_CORNER")
                self.stop()
            else:
                self.move_slow(dx, dy)   # yaw_rate = 0.0 while translating

        elif self.state == FlightState.AT_CORNER:
            # Decide heading to the NEXT waypoint (idx+1)
            if self.current_waypoint_idx + 1 < len(self.waypoints):
                nx, ny, _ = self.waypoints[self.current_waypoint_idx + 1]
                ndx = nx - self.current_pose.position.x
                ndy = ny - self.current_pose.position.y
                next_yaw = float(np.arctan2(ndy, ndx))
                next_err = self.normalize_angle(next_yaw - self.current_yaw)

                # Enter TURNING only if heading error is clearly large (hysteresis)
                if abs(next_err) > self.yaw_turn_enter:
                    self.state = FlightState.TURNING
                    self.publish_state("TURNING")
                    self.turn(next_err)
                else:
                    # Already sufficiently aligned → advance and go straight
                    self.current_waypoint_idx += 1
                    self.state = FlightState.STRAIGHT
                    self.publish_state("STRAIGHT")
                    self.publish_status(f"Corner complete, waypoint {self.current_waypoint_idx}/{len(self.waypoints)}")
            else:
                # Last waypoint reached
                self.current_waypoint_idx += 1

        elif self.state == FlightState.TURNING:
            # Turn toward NEXT waypoint (idx+1) — crucial fix
            if self.current_waypoint_idx + 1 < len(self.waypoints):
                nx, ny, _ = self.waypoints[self.current_waypoint_idx + 1]
                ndx = nx - self.current_pose.position.x
                ndy = ny - self.current_pose.position.y
                next_yaw = float(np.arctan2(ndy, ndx))
                next_err = self.normalize_angle(next_yaw - self.current_yaw)

                if abs(next_err) < self.yaw_tolerance:
                    # Finished turning — increment index BEFORE moving
                    self.current_waypoint_idx += 1
                    self.state = FlightState.STRAIGHT
                    self.publish_state("STRAIGHT")
                else:
                    self.turn(next_err)
            else:
                # No next waypoint; just stop
                self.state = FlightState.IDLE
                self.stop()
                self.publish_state("IDLE")

    # ----------------------- Motion Primitives -----------------------

    def move_straight(self, dx, dy):
        dist = float(np.hypot(dx, dy))
        if dist < 1e-6:
            self.stop()
            return
        vx = (dx / dist) * self.cruise_speed
        vy = (dy / dist) * self.cruise_speed
        self.send_velocity(vx, vy, 0.0, 0.0)  # yaw_rate = 0.0 while translating

    def move_slow(self, dx, dy):
        dist = float(np.hypot(dx, dy))
        if dist < 1e-6:
            self.stop()
            return
        vx = (dx / dist) * self.approach_speed
        vy = (dy / dist) * self.approach_speed
        self.send_velocity(vx, vy, 0.0, 0.0)  # yaw_rate = 0.0 while translating

    def turn(self, yaw_error):
        yaw_rate_cmd = float(np.clip(yaw_error * 3.0, -self.yaw_rate, self.yaw_rate))
        self.send_velocity(0.0, 0.0, 0.0, yaw_rate_cmd)

    def stop(self):
        self.send_velocity(0.0, 0.0, 0.0, 0.0)

    # ----------------------- Utils -----------------------

    def send_velocity(self, vx, vy, vz, yaw_rate):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = yaw_rate
        self.vel_pub.publish(msg)

    @staticmethod
    def normalize_angle(a):
        while a > np.pi:
            a -= 2 * np.pi
        while a < -np.pi:
            a += 2 * np.pi
        return float(a)

    def publish_state(self, state: str):
        self.state_pub.publish(String(data=state))

    def publish_status(self, status: str):
        self.status_pub.publish(String(data=status))
        self.get_logger().info(status)

def main(args=None):
    rclpy.init(args=args)
    node = UpdatedVFSharpyTuned()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()