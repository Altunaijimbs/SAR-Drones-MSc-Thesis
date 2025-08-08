#!/usr/bin/env python3
"""
Background Visualizer - Non-intrusive drone position plotter
Updates in background without stealing focus
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import matplotlib
matplotlib.use('TkAgg')  # Use TkAgg backend
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import json

class BackgroundVisualizer(Node):
    def __init__(self):
        super().__init__('background_visualizer')
        
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
        self.path_history = deque(maxlen=500)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.waypoints = []
        self.first_position_received = False
        self.start_position = None
        
        # Create figure and axis
        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(111)
        
        # Set up the plot
        self.setup_plot()
        
        # Animation will be started from main
        self.ani = None
        
        self.get_logger().info('Background Visualizer started')
    
    def setup_plot(self):
        """Initial plot setup"""
        self.ax.set_xlabel('X Position (meters)')
        self.ax.set_ylabel('Y Position (meters)')
        self.ax.set_title('Drone Position Tracker')
        self.ax.grid(True)
        self.ax.set_aspect('equal')
        
        # Set initial limits
        self.ax.set_xlim(-30, 30)
        self.ax.set_ylim(-30, 30)
        
        # Create line objects we'll update
        self.path_line, = self.ax.plot([], [], 'b-', linewidth=2, label='Path')
        self.current_dot, = self.ax.plot([], [], 'ro', markersize=10, label='Current')
        self.start_marker, = self.ax.plot([], [], 'g^', markersize=12, label='Start')
        self.waypoint_line, = self.ax.plot([], [], 'g--', alpha=0.5, label='Waypoints')
        
        self.ax.legend()
        
        # Prevent window from stealing focus
        self.fig.canvas.manager.window.attributes('-topmost', False)
    
    def update_position(self, msg):
        """Update drone position from ROS"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        
        if not self.first_position_received:
            self.start_position = (self.current_x, self.current_y, self.current_z)
            self.first_position_received = True
            self.get_logger().info(f'Tracking from: X={self.current_x:.2f}, Y={self.current_y:.2f}, Z={self.current_z:.2f}')
        
        self.path_history.append((self.current_x, self.current_y))
    
    def update_waypoints(self, msg):
        """Update waypoint list"""
        try:
            data = json.loads(msg.data)
            self.waypoints = data.get('waypoints', [])
        except:
            pass
    
    def animate(self, frame):
        """Animation function - called periodically by matplotlib"""
        if not self.first_position_received:
            return self.path_line, self.current_dot, self.start_marker, self.waypoint_line
        
        # Update path line
        if len(self.path_history) > 1:
            path_x = [p[0] for p in self.path_history]
            path_y = [p[1] for p in self.path_history]
            self.path_line.set_data(path_x, path_y)
        
        # Update current position
        self.current_dot.set_data([self.current_x], [self.current_y])
        
        # Update start marker
        if self.start_position:
            self.start_marker.set_data([self.start_position[0]], [self.start_position[1]])
        
        # Update waypoints
        if self.waypoints:
            wp_x = [w['x'] for w in self.waypoints]
            wp_y = [w['y'] for w in self.waypoints]
            self.waypoint_line.set_data(wp_x, wp_y)
        
        # Update title with altitude
        self.ax.set_title(f'Drone Tracker - Alt: {self.current_z:.1f}m')
        
        # Dynamically adjust view if needed (every 10th frame to reduce overhead)
        if frame % 10 == 0 and len(self.path_history) > 0:
            all_x = [p[0] for p in self.path_history]
            all_y = [p[1] for p in self.path_history]
            
            if all_x and all_y:
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
        
        return self.path_line, self.current_dot, self.start_marker, self.waypoint_line
    
    def start_animation(self):
        """Start the animation"""
        # Use blit=True for efficiency, interval=500ms for less aggressive updates
        self.ani = animation.FuncAnimation(
            self.fig, self.animate, interval=500, blit=True, cache_frame_data=False
        )
        plt.show(block=False)  # Non-blocking show

def main(args=None):
    rclpy.init(args=args)
    node = BackgroundVisualizer()
    
    # Start animation in background
    node.start_animation()
    
    try:
        # Spin ROS node
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()