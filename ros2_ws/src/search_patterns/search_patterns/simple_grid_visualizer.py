#!/usr/bin/env python3
"""
Simple Grid Visualizer - A more reliable real-time drone position plotter
Starts tracking from current drone position (works if drone already flying)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import matplotlib.pyplot as plt
from collections import deque
import json
import time

class SimpleGridVisualizer(Node):
    def __init__(self):
        super().__init__('simple_grid_visualizer')
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribe to drone position
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.update_position,
            qos_profile=self.mavros_qos
        )
        
        # Subscribe to pattern waypoints
        self.waypoints_sub = self.create_subscription(
            String, '/pattern_waypoints',
            self.update_waypoints, 10
        )
        
        # Data storage
        self.path_history = deque(maxlen=500)  # Store last 500 positions
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.waypoints = []
        self.first_position_received = False
        self.start_position = None
        
        # Plot will be created in main thread
        self.fig = None
        self.ax = None
        
        self.get_logger().info('Simple Grid Visualizer started - Waiting for drone position...')
    
    def setup_plot(self):
        """Setup the plot with grid"""
        self.ax.set_xlabel('X Position (meters)', fontsize=12)
        self.ax.set_ylabel('Y Position (meters)', fontsize=12)
        self.ax.set_title('Waiting for drone position...', fontsize=14, fontweight='bold')
        
        # Add grid
        self.ax.grid(True, which='both', linestyle='-', alpha=0.3)
        self.ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
        self.ax.axvline(x=0, color='k', linestyle='-', alpha=0.3)
        
        # Set initial view
        self.ax.set_xlim(-30, 30)
        self.ax.set_ylim(-30, 30)
        self.ax.set_aspect('equal')
        
        # Initialize empty plots
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Drone Path')
        self.current_pos, = self.ax.plot([], [], 'ro', markersize=10, label='Current Position')
        self.start_pos, = self.ax.plot([], [], 'go', markersize=10, label='Start Position')
        self.waypoint_line, = self.ax.plot([], [], 'g--', alpha=0.5, label='Planned Path')
        self.waypoint_markers, = self.ax.plot([], [], 'gs', markersize=8, label='Waypoints')
        
        self.ax.legend(loc='upper right')
        plt.show()
    
    def update_position(self, msg):
        """Update drone position"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        
        # Save start position on first update
        if not self.first_position_received:
            self.start_position = (self.current_x, self.current_y, self.current_z)
            self.first_position_received = True
            self.get_logger().info(f'Started tracking at: X={self.current_x:.2f}, Y={self.current_y:.2f}, Z={self.current_z:.2f}')
            # Add starting position to path
            self.path_history.append((self.current_x, self.current_y))
        
        # Add to path history
        self.path_history.append((self.current_x, self.current_y))
        
        # Log position occasionally
        if len(self.path_history) % 10 == 0:
            self.get_logger().info(f'Position: X={self.current_x:.2f}, Y={self.current_y:.2f}, Z={self.current_z:.2f}')
    
    def update_waypoints(self, msg):
        """Update waypoint list"""
        try:
            data = json.loads(msg.data)
            self.waypoints = data.get('waypoints', [])
            self.get_logger().info(f'Received {len(self.waypoints)} waypoints')
        except:
            pass
    
    def update_plot(self):
        """Update plot - called from main thread"""
        try:
            # Clear and redraw
            self.ax.clear()
            
            # Setup grid again
            self.ax.grid(True, which='both', linestyle='-', alpha=0.3)
            self.ax.axhline(y=0, color='k', linestyle='-', alpha=0.3)
            self.ax.axvline(x=0, color='k', linestyle='-', alpha=0.3)
            
            # Only draw if we have received position data
            if self.first_position_received:
                # Draw start position marker
                if self.start_position:
                    self.ax.plot(self.start_position[0], self.start_position[1], 'g^', 
                               markersize=15, label=f'Start (Alt: {self.start_position[2]:.1f}m)')
                
                # Draw path history
                if len(self.path_history) > 1:
                    path_x = [p[0] for p in self.path_history]
                    path_y = [p[1] for p in self.path_history]
                    self.ax.plot(path_x, path_y, 'b-', linewidth=2, label='Drone Path')
                
                # Draw waypoints
                if self.waypoints:
                    wp_x = [w['x'] for w in self.waypoints]
                    wp_y = [w['y'] for w in self.waypoints]
                    self.ax.plot(wp_x, wp_y, 'g--', alpha=0.5, label='Planned Path')
                    self.ax.plot(wp_x, wp_y, 'gs', markersize=8, label='Waypoints')
                
                # Draw current position
                self.ax.plot(self.current_x, self.current_y, 'ro', markersize=12, label='Current Position')
                
                # Add position text box
                info_text = f'Current: ({self.current_x:.1f}, {self.current_y:.1f}, {self.current_z:.1f})\n'
                if self.start_position:
                    dx = self.current_x - self.start_position[0]
                    dy = self.current_y - self.start_position[1]
                    dz = self.current_z - self.start_position[2]
                    info_text += f'Δ from start: ({dx:.1f}, {dy:.1f}, {dz:.1f})'
                
                self.ax.text(0.02, 0.98, info_text,
                           transform=self.ax.transAxes, fontsize=10,
                           verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))
                
                # Set title with altitude
                self.ax.set_title(f'Drone Tracker - Alt: {self.current_z:.1f}m', fontsize=14, fontweight='bold')
                
                # Dynamic axis limits centered around activity
                if len(self.path_history) > 0:
                    all_x = [p[0] for p in self.path_history] + [self.current_x]
                    all_y = [p[1] for p in self.path_history] + [self.current_y]
                    
                    if self.waypoints:
                        all_x.extend([w['x'] for w in self.waypoints])
                        all_y.extend([w['y'] for w in self.waypoints])
                    
                    margin = 5
                    x_min, x_max = min(all_x) - margin, max(all_x) + margin
                    y_min, y_max = min(all_y) - margin, max(all_y) + margin
                    
                    # Ensure minimum range
                    if x_max - x_min < 20:
                        center = (x_max + x_min) / 2
                        x_min, x_max = center - 10, center + 10
                    if y_max - y_min < 20:
                        center = (y_max + y_min) / 2
                        y_min, y_max = center - 10, center + 10
                    
                    self.ax.set_xlim(x_min, x_max)
                    self.ax.set_ylim(y_min, y_max)
            else:
                # No position received yet
                self.ax.set_title('Waiting for drone position...', fontsize=14, fontweight='bold')
                self.ax.set_xlim(-30, 30)
                self.ax.set_ylim(-30, 30)
            
            # Set labels
            self.ax.set_xlabel('X Position (meters)', fontsize=12)
            self.ax.set_ylabel('Y Position (meters)', fontsize=12)
            self.ax.legend(loc='upper right')
            self.ax.set_aspect('equal')
            
            # Refresh plot
            plt.draw()
            plt.pause(0.01)  # Small pause for update
            
        except Exception as e:
            self.get_logger().error(f'Plot update error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleGridVisualizer()
    
    # Setup plot in main thread
    plt.ion()
    node.fig, node.ax = plt.subplots(figsize=(10, 10))
    node.setup_plot()
    
    try:
        # Main loop - handle both ROS and matplotlib in main thread
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update_plot()
            
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()