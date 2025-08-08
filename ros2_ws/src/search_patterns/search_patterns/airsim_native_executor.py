#!/usr/bin/env python3
"""
AirSim Native Pattern Executor - Uses AirSim's built-in moveOnPath
This provides smooth, realistic drone movement with proper lookahead
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import airsim
import numpy as np
import threading
import time

class AirSimNativeExecutor(Node):
    def __init__(self):
        super().__init__('airsim_native_executor')
        
        # AirSim client
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        
        # Publishers
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
        
        # State
        self.waypoints = []
        self.executing = False
        self.paused = False
        
        # Control parameters (tuned for smooth flight)
        self.velocity = 5.0  # m/s - much faster than our previous attempts!
        self.lookahead_distance = 10.0  # meters to look ahead on path
        self.adaptive_lookahead = 1.0  # adaptive scaling factor
        
        # Execution thread
        self.execution_thread = None
        self.current_task = None
        
        # Status timer
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('AirSim Native Pattern Executor initialized - Smooth flight guaranteed!')
    
    def load_waypoints(self, msg):
        """Load waypoints and convert to AirSim format"""
        try:
            data = json.loads(msg.data)
            self.waypoints = data['waypoints']
            
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints for native execution')
            
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {e}')
    
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
        """Start executing the pattern using AirSim's native path following"""
        if not self.waypoints:
            self.get_logger().warn('No waypoints loaded')
            return
        
        if self.executing:
            self.get_logger().warn('Already executing pattern')
            return
        
        self.executing = True
        self.paused = False
        
        # Start execution in separate thread
        self.execution_thread = threading.Thread(target=self.execute_native_path)
        self.execution_thread.start()
        
        self.get_logger().info('Started AirSim native pattern execution')
    
    def stop_execution(self):
        """Stop pattern execution"""
        self.executing = False
        self.paused = False
        
        # Cancel current AirSim movement
        try:
            if self.current_task:
                self.current_task.cancel()
            self.client.moveByVelocityAsync(0, 0, 0, 0.1).join()  # Stop
            self.client.hoverAsync().join()  # Hover
        except:
            pass
        
        if self.execution_thread and self.execution_thread.is_alive():
            self.execution_thread.join(timeout=2.0)
        
        self.get_logger().info('Stopped pattern execution')
    
    def pause_execution(self):
        """Pause at current position"""
        if self.executing:
            self.paused = True
            try:
                if self.current_task:
                    self.current_task.cancel()
                self.client.hoverAsync().join()
            except:
                pass
            self.get_logger().info('Paused pattern execution')
    
    def resume_execution(self):
        """Resume from current position"""
        if self.executing and self.paused:
            self.paused = False
            # Re-enable API control after hover
            self.client.enableApiControl(True)
            self.get_logger().info('Resumed pattern execution')
    
    def reset_execution(self):
        """Reset execution"""
        self.stop_execution()
        self.get_logger().info('Reset pattern execution')
    
    def execute_native_path(self):
        """Execute path using AirSim's native moveOnPath"""
        try:
            # Convert waypoints to AirSim format
            path = []
            for wp in self.waypoints:
                # AirSim uses NED coordinates (North-East-Down)
                # Our coordinates are ENU (East-North-Up), so we need to convert
                # X (East) -> Y (East in NED)
                # Y (North) -> X (North in NED)  
                # Z (Up) -> -Z (Down in NED)
                path.append(airsim.Vector3r(wp['y'], wp['x'], -wp['z']))
            
            if not path:
                self.get_logger().error('No valid path points')
                return
            
            # Calculate total distance for timeout estimation
            total_distance = 0
            for i in range(1, len(path)):
                dx = path[i].x_val - path[i-1].x_val
                dy = path[i].y_val - path[i-1].y_val
                dz = path[i].z_val - path[i-1].z_val
                total_distance += np.sqrt(dx**2 + dy**2 + dz**2)
            
            # Estimate timeout (add 50% buffer)
            timeout = (total_distance / self.velocity) * 1.5 + 10
            
            self.get_logger().info(
                f'Flying path with {len(path)} waypoints, '
                f'distance: {total_distance:.1f}m, '
                f'velocity: {self.velocity}m/s, '
                f'estimated time: {timeout:.1f}s'
            )
            
            # Dynamic lookahead based on velocity (like in survey.py)
            dynamic_lookahead = self.velocity + (self.velocity / 2)
            
            # Execute the path using AirSim's native path following
            # This provides smooth curves and proper banking!
            self.current_task = self.client.moveOnPathAsync(
                path=path,
                velocity=self.velocity,
                timeout_sec=timeout,
                drivetrain=airsim.DrivetrainType.ForwardOnly,  # Always face forward
                yaw_mode=airsim.YawMode(False, 0),  # Let path determine yaw
                lookahead=dynamic_lookahead,  # Dynamic lookahead for smooth turns
                adaptive_lookahead=self.adaptive_lookahead  # Adaptive scaling
            )
            
            # Wait for completion
            self.current_task.join()
            
            if self.executing and not self.paused:
                self.get_logger().info('Pattern execution completed successfully')
                
                # Return to hover
                self.client.hoverAsync().join()
            
        except Exception as e:
            self.get_logger().error(f'Error during native path execution: {e}')
        
        finally:
            self.executing = False
            self.current_task = None
    
    def publish_status(self):
        """Publish pattern execution status"""
        # Get current position from AirSim
        try:
            state = self.client.getMultirotorState()
            position = state.kinematics_estimated.position
            
            # Convert from NED to ENU for status
            current_pos = {
                'x': position.y_val,
                'y': position.x_val,
                'z': -position.z_val
            }
        except:
            current_pos = {'x': 0, 'y': 0, 'z': 0}
        
        status = {
            'executing': self.executing,
            'paused': self.paused,
            'total_waypoints': len(self.waypoints),
            'current_position': current_pos,
            'velocity': self.velocity,
            'lookahead': self.lookahead_distance
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AirSimNativeExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()