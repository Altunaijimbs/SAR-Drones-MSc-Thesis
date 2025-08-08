#!/usr/bin/env python3
"""
Real-time Drone Position Tracker for SimpleFlight
Shows drone position, path history, and planned waypoints
"""

import cosysairsim as airsim
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import threading
import time
import argparse

class DroneTracker:
    def __init__(self, history_length=500, update_rate=10):
        """
        Initialize the drone tracker
        
        Args:
            history_length: Number of position points to keep in history
            update_rate: Updates per second
        """
        self.client = None
        self.history_length = history_length
        self.update_rate = update_rate
        self.update_interval = 1.0 / update_rate
        
        # Position history
        self.x_history = deque(maxlen=history_length)
        self.y_history = deque(maxlen=history_length)
        self.z_history = deque(maxlen=history_length)
        
        # Current position
        self.current_x = 0
        self.current_y = 0
        self.current_z = 0
        
        # Planned waypoints (if any)
        self.waypoints_x = []
        self.waypoints_y = []
        
        # Threading
        self.running = False
        self.update_thread = None
        
        # Plot setup
        self.fig = None
        self.ax_xy = None
        self.ax_alt = None
        
    def connect(self):
        """Connect to AirSim"""
        print("Connecting to AirSim...")
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        print("Connected!")
        
    def update_position(self):
        """Update drone position from AirSim"""
        while self.running:
            try:
                # Get current position
                pose = self.client.simGetVehiclePose()
                
                # Update current position (convert from NED to visualization coords)
                self.current_x = pose.position.x_val
                self.current_y = pose.position.y_val
                self.current_z = -pose.position.z_val  # Convert from NED
                
                # Add to history
                self.x_history.append(self.current_x)
                self.y_history.append(self.current_y)
                self.z_history.append(self.current_z)
                
            except Exception as e:
                print(f"Error updating position: {e}")
                
            time.sleep(self.update_interval)
    
    def set_waypoints(self, waypoints):
        """Set planned waypoints to display"""
        self.waypoints_x = [wp[0] for wp in waypoints]
        self.waypoints_y = [wp[1] for wp in waypoints]
    
    def setup_plot(self):
        """Setup the matplotlib figure and axes"""
        plt.style.use('dark_background')
        self.fig = plt.figure(figsize=(14, 7))
        
        # XY view (top-down)
        self.ax_xy = plt.subplot(121)
        self.ax_xy.set_xlabel('X (meters)', fontsize=10)
        self.ax_xy.set_ylabel('Y (meters)', fontsize=10)
        self.ax_xy.set_title('Top-Down View', fontsize=12, fontweight='bold')
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.set_aspect('equal')
        
        # Altitude view
        self.ax_alt = plt.subplot(122)
        self.ax_alt.set_xlabel('Time (samples)', fontsize=10)
        self.ax_alt.set_ylabel('Altitude (meters)', fontsize=10)
        self.ax_alt.set_title('Altitude History', fontsize=12, fontweight='bold')
        self.ax_alt.grid(True, alpha=0.3)
        
        self.fig.suptitle('SimpleFlight Drone Tracker', fontsize=14, fontweight='bold')
        plt.tight_layout()
        
    def animate(self, frame):
        """Animation update function"""
        # Clear axes
        self.ax_xy.clear()
        self.ax_alt.clear()
        
        # Reapply settings
        self.ax_xy.set_xlabel('X (meters)', fontsize=10)
        self.ax_xy.set_ylabel('Y (meters)', fontsize=10)
        self.ax_xy.set_title('Top-Down View', fontsize=12, fontweight='bold')
        self.ax_xy.grid(True, alpha=0.3)
        self.ax_xy.set_aspect('equal')
        
        self.ax_alt.set_xlabel('Time (samples)', fontsize=10)
        self.ax_alt.set_ylabel('Altitude (meters)', fontsize=10)
        self.ax_alt.set_title('Altitude History', fontsize=12, fontweight='bold')
        self.ax_alt.grid(True, alpha=0.3)
        
        # Plot XY path
        if len(self.x_history) > 1:
            # Plot trail with gradient
            points = len(self.x_history)
            colors = np.linspace(0.3, 1.0, points)
            x_list = list(self.x_history)
            y_list = list(self.y_history)
            for i in range(1, points):
                self.ax_xy.plot(
                    [x_list[i-1], x_list[i]], 
                    [y_list[i-1], y_list[i]],
                    'c-', alpha=colors[i], linewidth=2
                )
        
        # Plot waypoints if any
        if self.waypoints_x:
            self.ax_xy.plot(self.waypoints_x, self.waypoints_y, 
                          'g--', alpha=0.5, linewidth=1, label='Planned Path')
            self.ax_xy.scatter(self.waypoints_x, self.waypoints_y, 
                             c='green', s=30, alpha=0.7, marker='o')
        
        # Plot current position
        if len(self.x_history) > 0:
            self.ax_xy.scatter(self.current_x, self.current_y, 
                             c='red', s=100, marker='o', 
                             edgecolors='white', linewidth=2,
                             label=f'Drone ({self.current_x:.1f}, {self.current_y:.1f})')
            
            # Add direction arrow (from second-to-last to current position)
            if len(self.x_history) > 1:
                x_list = list(self.x_history)
                y_list = list(self.y_history)
                dx = self.current_x - x_list[-2]
                dy = self.current_y - y_list[-2]
                if abs(dx) > 0.01 or abs(dy) > 0.01:  # Only show if moving
                    self.ax_xy.arrow(x_list[-2], y_list[-2], 
                                   dx*0.8, dy*0.8,
                                   head_width=0.5, head_length=0.3, 
                                   fc='yellow', ec='yellow', alpha=0.8)
        
        # Set axis limits with margin
        if len(self.x_history) > 0:
            x_min, x_max = min(self.x_history), max(self.x_history)
            y_min, y_max = min(self.y_history), max(self.y_history)
            
            # Include waypoints in range
            if self.waypoints_x:
                x_min = min(x_min, min(self.waypoints_x))
                x_max = max(x_max, max(self.waypoints_x))
                y_min = min(y_min, min(self.waypoints_y))
                y_max = max(y_max, max(self.waypoints_y))
            
            margin = 5
            self.ax_xy.set_xlim(x_min - margin, x_max + margin)
            self.ax_xy.set_ylim(y_min - margin, y_max + margin)
        
        # Plot altitude history
        if len(self.z_history) > 0:
            time_points = list(range(len(self.z_history)))
            self.ax_alt.plot(time_points, list(self.z_history), 
                           'g-', linewidth=2, label=f'Current: {self.current_z:.1f}m')
            self.ax_alt.fill_between(time_points, 0, list(self.z_history), 
                                    color='green', alpha=0.2)
            
            # Current altitude marker
            self.ax_alt.scatter(len(self.z_history)-1, self.current_z, 
                              c='red', s=100, marker='o', 
                              edgecolors='white', linewidth=2)
            
            # Reference line at 0
            self.ax_alt.axhline(y=0, color='red', linestyle='--', alpha=0.3)
        
        # Add legends
        self.ax_xy.legend(loc='upper right', fontsize=9)
        self.ax_alt.legend(loc='upper right', fontsize=9)
        
        # Add stats
        if len(self.x_history) > 1:
            x_list = list(self.x_history)
            y_list = list(self.y_history)
            total_distance = sum(
                np.sqrt((x_list[i] - x_list[i-1])**2 + 
                       (y_list[i] - y_list[i-1])**2)
                for i in range(1, len(x_list))
            )
            self.ax_xy.text(0.02, 0.98, f'Distance: {total_distance:.1f}m', 
                          transform=self.ax_xy.transAxes,
                          fontsize=9, verticalalignment='top',
                          bbox=dict(boxstyle='round', facecolor='black', alpha=0.5))
    
    def start(self):
        """Start tracking and visualization"""
        self.connect()
        
        # Start position update thread
        self.running = True
        self.update_thread = threading.Thread(target=self.update_position)
        self.update_thread.daemon = True
        self.update_thread.start()
        
        # Setup and start animation
        self.setup_plot()
        ani = animation.FuncAnimation(
            self.fig, self.animate, 
            interval=100,  # Update every 100ms
            blit=False
        )
        
        print("\n" + "="*60)
        print("DRONE TRACKER STARTED")
        print("="*60)
        print("• Real-time position tracking")
        print("• Path history visualization")
        print("• Altitude monitoring")
        print("\nClose the window to stop tracking")
        print("="*60 + "\n")
        
        plt.show()
        
        # Cleanup
        self.running = False
        if self.update_thread:
            self.update_thread.join()

def main():
    parser = argparse.ArgumentParser(description='SimpleFlight Drone Tracker')
    parser.add_argument('--history', type=int, default=500,
                       help='Number of position points to keep in history')
    parser.add_argument('--rate', type=int, default=10,
                       help='Update rate in Hz')
    args = parser.parse_args()
    
    tracker = DroneTracker(history_length=args.history, update_rate=args.rate)
    
    try:
        tracker.start()
    except KeyboardInterrupt:
        print("\nTracking stopped by user")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    main()