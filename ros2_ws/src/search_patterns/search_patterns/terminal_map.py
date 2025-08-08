#!/usr/bin/env python3
"""
Terminal Map - ASCII-based drone position tracker
No matplotlib required, runs entirely in terminal
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from collections import deque
import json
import os
import time

class TerminalMap(Node):
    def __init__(self):
        super().__init__('terminal_map')
        
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
        self.path_history = deque(maxlen=200)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.waypoints = []
        self.first_position_received = False
        self.start_position = None
        
        # Map settings
        self.map_width = 60
        self.map_height = 30
        self.scale = 1.0  # meters per character
        
        # Create timer for display updates
        self.timer = self.create_timer(0.5, self.update_display)
        
        self.get_logger().info('Terminal Map started - ASCII visualization')
    
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
    
    def world_to_map(self, x, y):
        """Convert world coordinates to map grid"""
        # Center the map around origin
        map_x = int(self.map_width / 2 + x / self.scale)
        map_y = int(self.map_height / 2 - y / self.scale)  # Invert Y for display
        return map_x, map_y
    
    def update_display(self):
        """Update the terminal display"""
        if not self.first_position_received:
            return
        
        # Clear screen
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # Create empty map
        map_grid = [[' ' for _ in range(self.map_width)] for _ in range(self.map_height)]
        
        # Add grid lines
        for i in range(0, self.map_width, 10):
            for j in range(self.map_height):
                if map_grid[j][i] == ' ':
                    map_grid[j][i] = '·'
        for j in range(0, self.map_height, 5):
            for i in range(self.map_width):
                if map_grid[j][i] == ' ':
                    map_grid[j][i] = '·'
        
        # Mark origin
        ox, oy = self.world_to_map(0, 0)
        if 0 <= ox < self.map_width and 0 <= oy < self.map_height:
            map_grid[oy][ox] = '+'
        
        # Draw path history
        for px, py in self.path_history:
            mx, my = self.world_to_map(px, py)
            if 0 <= mx < self.map_width and 0 <= my < self.map_height:
                if map_grid[my][mx] in [' ', '·']:
                    map_grid[my][mx] = '.'
        
        # Draw waypoints
        for wp in self.waypoints:
            wx, wy = self.world_to_map(wp['x'], wp['y'])
            if 0 <= wx < self.map_width and 0 <= wy < self.map_height:
                map_grid[wy][wx] = 'W'
        
        # Draw start position
        if self.start_position:
            sx, sy = self.world_to_map(self.start_position[0], self.start_position[1])
            if 0 <= sx < self.map_width and 0 <= sy < self.map_height:
                map_grid[sy][sx] = 'S'
        
        # Draw current position (overwrites others)
        cx, cy = self.world_to_map(self.current_x, self.current_y)
        if 0 <= cx < self.map_width and 0 <= cy < self.map_height:
            map_grid[cy][cx] = 'D'
        
        # Print header
        print("╔" + "═" * (self.map_width + 20) + "╗")
        print(f"║ DRONE TERMINAL MAP - Alt: {self.current_z:.1f}m" + " " * (self.map_width - 15) + "║")
        print("╠" + "═" * (self.map_width + 20) + "╣")
        
        # Print map with border
        for row in map_grid:
            print("║ " + ''.join(row) + " " * 18 + "║")
        
        print("╠" + "═" * (self.map_width + 20) + "╣")
        
        # Print legend and info
        print(f"║ Position: X={self.current_x:6.2f} Y={self.current_y:6.2f} Z={self.current_z:6.2f}" + " " * (self.map_width - 25) + "║")
        if self.start_position:
            dx = self.current_x - self.start_position[0]
            dy = self.current_y - self.start_position[1]
            print(f"║ Delta:    ΔX={dx:6.2f} ΔY={dy:6.2f}" + " " * (self.map_width - 14) + "║")
        print(f"║ Legend: D=Drone S=Start W=Waypoint .=Path +=Origin" + " " * (self.map_width - 35) + "║")
        print(f"║ Scale: {self.scale:.1f} meter/char | Path points: {len(self.path_history)}" + " " * (self.map_width - 26) + "║")
        print("╚" + "═" * (self.map_width + 20) + "╝")

def main(args=None):
    rclpy.init(args=args)
    node = TerminalMap()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()