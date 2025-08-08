#!/usr/bin/env python3
"""
Pattern Monitor - Terminal-based pattern execution monitor
Simple text output for monitoring pattern execution without GUI dependencies
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
import os
import time

class PatternMonitor(Node):
    def __init__(self):
        super().__init__('pattern_monitor')
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.update_drone_position,
            qos_profile=self.mavros_qos
        )
        
        self.waypoints_sub = self.create_subscription(
            String, '/pattern_waypoints',
            self.update_waypoints, 10
        )
        
        self.status_sub = self.create_subscription(
            String, '/pattern_status',
            self.update_status, 10
        )
        
        # State
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0
        self.waypoints = []
        self.current_waypoint_index = 0
        self.pattern_executing = False
        self.total_waypoints = 0
        self.start_time = time.time()
        self.distance_traveled = 0
        self.last_x = 0
        self.last_y = 0
        
        # Timer for display update
        self.create_timer(0.5, self.update_display)
        
        self.get_logger().info('Pattern Monitor started')
    
    def update_drone_position(self, msg):
        """Update current drone position"""
        self.last_x = self.current_x
        self.last_y = self.current_y
        
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        
        # Calculate distance traveled
        if self.last_x != 0 or self.last_y != 0:
            dx = self.current_x - self.last_x
            dy = self.current_y - self.last_y
            self.distance_traveled += (dx**2 + dy**2)**0.5
    
    def update_waypoints(self, msg):
        """Update waypoint list from pattern generator"""
        try:
            data = json.loads(msg.data)
            self.waypoints = data['waypoints']
            self.total_waypoints = len(self.waypoints)
            self.get_logger().info(f'Loaded {self.total_waypoints} waypoints')
            self.distance_traveled = 0  # Reset distance
            self.start_time = time.time()  # Reset timer
        except Exception as e:
            self.get_logger().error(f'Error parsing waypoints: {e}')
    
    def update_status(self, msg):
        """Update pattern execution status"""
        try:
            data = json.loads(msg.data)
            self.pattern_executing = data.get('executing', False)
            self.current_waypoint_index = data.get('current_waypoint', 0)
        except Exception as e:
            self.get_logger().error(f'Error parsing status: {e}')
    
    def update_display(self):
        """Update terminal display"""
        # Clear screen
        os.system('clear' if os.name == 'posix' else 'cls')
        
        print("╔══════════════════════════════════════════════════════════╗")
        print("║              PATTERN EXECUTION MONITOR                   ║")
        print("╚══════════════════════════════════════════════════════════╝")
        print()
        
        # Current position
        print(f"📍 Current Position:")
        print(f"   X: {self.current_x:8.2f} m")
        print(f"   Y: {self.current_y:8.2f} m")
        print(f"   Z: {self.current_z:8.2f} m")
        print()
        
        # Pattern status
        print(f"🎯 Pattern Status:")
        print(f"   Executing: {'✅ Yes' if self.pattern_executing else '❌ No'}")
        print(f"   Waypoint: {self.current_waypoint_index}/{self.total_waypoints}")
        
        if self.pattern_executing and self.current_waypoint_index > 0 and self.current_waypoint_index <= len(self.waypoints):
            target = self.waypoints[self.current_waypoint_index - 1]
            dx = target['x'] - self.current_x
            dy = target['y'] - self.current_y
            distance_to_target = (dx**2 + dy**2)**0.5
            print(f"   Distance to target: {distance_to_target:.2f} m")
            print(f"   Target: ({target['x']:.1f}, {target['y']:.1f}, {target['z']:.1f})")
        
        # Progress bar
        if self.total_waypoints > 0:
            progress = (self.current_waypoint_index / self.total_waypoints) * 100
            bar_length = 40
            filled = int(bar_length * progress / 100)
            bar = '█' * filled + '░' * (bar_length - filled)
            print(f"\n   Progress: [{bar}] {progress:.1f}%")
        
        print()
        
        # Statistics
        elapsed_time = time.time() - self.start_time
        print(f"📊 Statistics:")
        print(f"   Time elapsed: {elapsed_time:.1f} seconds")
        print(f"   Distance traveled: {self.distance_traveled:.1f} m")
        if elapsed_time > 0:
            print(f"   Average speed: {self.distance_traveled/elapsed_time:.2f} m/s")
        
        print()
        print("Press Ctrl+C to exit")


def main(args=None):
    rclpy.init(args=args)
    node = PatternMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()