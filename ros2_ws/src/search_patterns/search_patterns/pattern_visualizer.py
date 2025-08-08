#!/usr/bin/env python3
"""
Pattern Visualizer - Real-time visualization of drone movement and search patterns
Shows both planned waypoints and actual drone path
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import matplotlib
# Force use of non-interactive backend first to avoid conflicts
matplotlib.use('Agg')
matplotlib.use('TkAgg')  # Then switch to TkAgg for display
import matplotlib.pyplot as plt
import matplotlib.animation as animation
# Suppress the Axes3D warning
import warnings
warnings.filterwarnings("ignore", message="Unable to import Axes3D")
from collections import deque
import json
import numpy as np

class PatternVisualizer(Node):
    def __init__(self):
        super().__init__('pattern_visualizer')
        
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
        
        # Data storage
        self.drone_path = deque(maxlen=1000)  # Store last 1000 positions
        self.waypoints = []
        self.current_waypoint_index = 0
        self.pattern_executing = False
        
        # Current position
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0
        
        # Visualization setup
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.setup_plot()
        
        # Animation setup
        self.ani = None
        
        self.get_logger().info('Pattern Visualizer started - Map window should appear')
    
    def setup_plot(self):
        """Setup the matplotlib figure"""
        self.ax.set_xlabel('X Position (meters)')
        self.ax.set_ylabel('Y Position (meters)')
        self.ax.set_title('Drone Movement Visualization')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # Initialize plot elements
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Actual Path')
        self.waypoint_line, = self.ax.plot([], [], 'g--', linewidth=1, alpha=0.5, label='Planned Path')
        self.waypoint_markers, = self.ax.plot([], [], 'go', markersize=8, label='Waypoints')
        self.current_pos, = self.ax.plot([], [], 'ro', markersize=12, label='Current Position')
        self.current_target, = self.ax.plot([], [], 'yo', markersize=10, label='Current Target')
        
        self.ax.legend(loc='upper right')
        
        # Set initial view
        self.ax.set_xlim(-20, 20)
        self.ax.set_ylim(-20, 20)
    
    def update_drone_position(self, msg):
        """Update current drone position"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        
        # Add to path history
        self.drone_path.append((self.current_x, self.current_y))
    
    def update_waypoints(self, msg):
        """Update waypoint list from pattern generator"""
        try:
            data = json.loads(msg.data)
            self.waypoints = data['waypoints']
            self.get_logger().info(f'Received {len(self.waypoints)} waypoints')
        except Exception as e:
            self.get_logger().error(f'Error parsing waypoints: {e}')
    
    def update_status(self, msg):
        """Update pattern execution status"""
        try:
            data = json.loads(msg.data)
            self.pattern_executing = data.get('executing', False)
            self.current_waypoint_index = data.get('current_waypoint', 0) - 1
        except Exception as e:
            self.get_logger().error(f'Error parsing status: {e}')
    
    def update_plot(self, frame):
        """Update plot with latest data"""
        # Update actual path
        if len(self.drone_path) > 1:
            path_data = list(self.drone_path)
            x_path = [p[0] for p in path_data]
            y_path = [p[1] for p in path_data]
            self.path_line.set_data(x_path, y_path)
        
        # Update waypoints
        if self.waypoints:
            x_waypoints = [w['x'] for w in self.waypoints]
            y_waypoints = [w['y'] for w in self.waypoints]
            
            # Waypoint markers
            self.waypoint_markers.set_data(x_waypoints, y_waypoints)
            
            # Waypoint path
            self.waypoint_line.set_data(x_waypoints, y_waypoints)
            
            # Current target
            if self.pattern_executing and 0 <= self.current_waypoint_index < len(self.waypoints):
                target = self.waypoints[self.current_waypoint_index]
                self.current_target.set_data([target['x']], [target['y']])
            else:
                self.current_target.set_data([], [])
        
        # Update current position
        self.current_pos.set_data([self.current_x], [self.current_y])
        
        # Auto-scale axes
        if len(self.drone_path) > 0 or self.waypoints:
            all_x = [self.current_x]
            all_y = [self.current_y]
            
            if len(self.drone_path) > 0:
                all_x.extend([p[0] for p in self.drone_path])
                all_y.extend([p[1] for p in self.drone_path])
            
            if self.waypoints:
                all_x.extend([w['x'] for w in self.waypoints])
                all_y.extend([w['y'] for w in self.waypoints])
            
            if all_x and all_y:  # Check lists are not empty
                # Add margin
                x_margin = 5
                y_margin = 5
                x_min = min(all_x) - x_margin
                x_max = max(all_x) + x_margin
                y_min = min(all_y) - y_margin
                y_max = max(all_y) + y_margin
                
                # Ensure minimum range
                if x_max - x_min < 10:
                    center_x = (x_max + x_min) / 2
                    x_min = center_x - 10
                    x_max = center_x + 10
                if y_max - y_min < 10:
                    center_y = (y_max + y_min) / 2
                    y_min = center_y - 10
                    y_max = center_y + 10
                
                self.ax.set_xlim(x_min, x_max)
                self.ax.set_ylim(y_min, y_max)
        
        # Update title with altitude
        self.ax.set_title(f'Drone Movement Visualization - Altitude: {self.current_z:.1f}m')
        
        return self.path_line, self.waypoint_line, self.waypoint_markers, self.current_pos, self.current_target
    
    def start_animation(self):
        """Start the matplotlib animation"""
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=100, 
            blit=True, cache_frame_data=False
        )
        plt.show()
    
    def clear_path(self):
        """Clear the recorded path"""
        self.drone_path.clear()
        self.get_logger().info('Path history cleared')


def main(args=None):
    rclpy.init(args=args)
    
    # Create node
    node = PatternVisualizer()
    
    # Create timer to spin the node
    timer = node.create_timer(0.01, lambda: rclpy.spin_once(node, timeout_sec=0))
    
    try:
        # Start animation in main thread
        node.start_animation()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.close('all')

if __name__ == '__main__':
    main()