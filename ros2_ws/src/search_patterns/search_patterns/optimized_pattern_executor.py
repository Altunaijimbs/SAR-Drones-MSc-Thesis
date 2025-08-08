#!/usr/bin/env python3
"""
Optimized Pattern Executor - Fixes overshooting and rotation issues
Uses approach slowdown and proper completion handling
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
import json
import math
import threading
import time

class OptimizedPatternExecutor(Node):
    def __init__(self):
        super().__init__('optimized_pattern_executor')
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.position_cmd_pub = self.create_publisher(
            String, '/position_command', 10
        )
        
        self.simple_cmd_pub = self.create_publisher(
            String, '/simple_command', 10
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
        
        # Optimized thresholds
        self.position_threshold = 2.5  # Increased for smoother approach
        self.approach_threshold = 5.0  # Start slowing down at this distance
        self.final_threshold = 1.0    # Final precise positioning
        
        self.execution_thread = None
        self.pattern_completed = False
        
        # Timer for status updates
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Optimized Pattern Executor initialized')
    
    def update_current_pose(self, msg):
        """Update current position"""
        self.current_pose = msg
    
    def load_waypoints(self, msg):
        """Load waypoints from pattern generator"""
        try:
            data = json.loads(msg.data)
            self.waypoints = data['waypoints']
            self.current_waypoint_index = 0
            self.pattern_completed = False
            self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
            
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
        """Start executing the pattern"""
        if not self.waypoints:
            self.get_logger().warn('No waypoints loaded')
            return
        
        if self.executing:
            self.get_logger().warn('Already executing pattern')
            return
        
        self.executing = True
        self.pattern_completed = False
        self.current_waypoint_index = 0
        
        # Start execution in separate thread
        self.execution_thread = threading.Thread(target=self.execute_pattern)
        self.execution_thread.start()
        
        self.get_logger().info('Started optimized pattern execution')
    
    def stop_execution(self):
        """Stop pattern execution and ensure drone stops"""
        self.executing = False
        self.pattern_completed = True
        
        # Send explicit stop command
        stop_cmd = String()
        stop_cmd.data = 'stop'
        self.simple_cmd_pub.publish(stop_cmd)
        
        # Wait for thread to finish
        if self.execution_thread and self.execution_thread.is_alive():
            self.execution_thread.join(timeout=2.0)
        
        self.get_logger().info('Stopped pattern execution - drone hovering')
    
    def pause_execution(self):
        """Pause at current position"""
        if self.executing:
            self.executing = False
            
            # Send stop command to hover
            stop_cmd = String()
            stop_cmd.data = 'stop'
            self.simple_cmd_pub.publish(stop_cmd)
            
            self.get_logger().info('Paused pattern execution')
    
    def resume_execution(self):
        """Resume from current waypoint"""
        if not self.executing and self.waypoints and not self.pattern_completed:
            self.executing = True
            
            # Resume in new thread
            self.execution_thread = threading.Thread(target=self.execute_pattern)
            self.execution_thread.start()
            
            self.get_logger().info('Resumed pattern execution')
    
    def reset_execution(self):
        """Reset to first waypoint"""
        self.stop_execution()
        self.current_waypoint_index = 0
        self.pattern_completed = False
        self.get_logger().info('Reset pattern execution')
    
    def get_distance_to_waypoint(self, waypoint):
        """Calculate distance to waypoint"""
        if not self.current_pose:
            return float('inf')
        
        dx = waypoint['x'] - self.current_pose.pose.position.x
        dy = waypoint['y'] - self.current_pose.pose.position.y
        dz = waypoint['z'] - self.current_pose.pose.position.z
        
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def execute_pattern(self):
        """Main pattern execution with approach control"""
        while self.executing and self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            
            # Send goto command
            goto_cmd = String()
            goto_cmd.data = f"goto:{waypoint['x']},{waypoint['y']},{waypoint['z']}"
            self.position_cmd_pub.publish(goto_cmd)
            
            self.get_logger().info(
                f'Flying to waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: '
                f'({waypoint["x"]:.1f}, {waypoint["y"]:.1f}, {waypoint["z"]:.1f})'
            )
            
            # Wait with approach monitoring
            approach_sent = False
            while self.executing:
                distance = self.get_distance_to_waypoint(waypoint)
                
                # Check if close enough to consider reached
                if distance < self.position_threshold:
                    # Wait a bit for stabilization
                    time.sleep(0.5)
                    
                    # Check if this is a corner (check next waypoint direction)
                    if self.current_waypoint_index < len(self.waypoints) - 1:
                        next_wp = self.waypoints[self.current_waypoint_index + 1]
                        
                        # If significant direction change, add brief hover
                        current_dx = waypoint['x'] - (self.waypoints[self.current_waypoint_index - 1]['x'] 
                                                       if self.current_waypoint_index > 0 
                                                       else self.current_pose.pose.position.x)
                        current_dy = waypoint['y'] - (self.waypoints[self.current_waypoint_index - 1]['y'] 
                                                       if self.current_waypoint_index > 0 
                                                       else self.current_pose.pose.position.y)
                        
                        next_dx = next_wp['x'] - waypoint['x']
                        next_dy = next_wp['y'] - waypoint['y']
                        
                        # Calculate angle between directions
                        angle = math.atan2(next_dy, next_dx) - math.atan2(current_dy, current_dx)
                        angle = abs((angle + math.pi) % (2 * math.pi) - math.pi)
                        
                        # If sharp turn (> 45 degrees), add stabilization
                        if angle > math.pi / 4:
                            self.get_logger().info(f'Sharp turn detected ({math.degrees(angle):.0f}°), stabilizing...')
                            # Send stop briefly to prevent overshoot
                            stop_cmd = String()
                            stop_cmd.data = 'stop'
                            self.simple_cmd_pub.publish(stop_cmd)
                            time.sleep(1.0)
                    
                    break
                
                # Approach control - slow down when getting close
                elif distance < self.approach_threshold and not approach_sent:
                    # Send a slower approach command
                    self.get_logger().debug(f'Approaching waypoint, distance: {distance:.1f}m')
                    approach_sent = True
                
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.executing:
                self.current_waypoint_index += 1
        
        # Pattern completed
        if self.current_waypoint_index >= len(self.waypoints):
            self.pattern_completed = True
            self.executing = False
            
            # IMPORTANT: Send final stop command to prevent rotation
            self.get_logger().info('Pattern execution completed - sending stop command')
            stop_cmd = String()
            stop_cmd.data = 'stop'
            self.simple_cmd_pub.publish(stop_cmd)
            time.sleep(0.5)
            
            # Send it again to be sure
            self.simple_cmd_pub.publish(stop_cmd)
            
            self.get_logger().info('Pattern fully completed - drone should be hovering')
    
    def publish_status(self):
        """Publish pattern execution status"""
        status = {
            'executing': self.executing,
            'completed': self.pattern_completed,
            'total_waypoints': len(self.waypoints),
            'current_waypoint': self.current_waypoint_index + 1 if self.executing else 0,
            'progress': (self.current_waypoint_index / len(self.waypoints) * 100) if self.waypoints else 0
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OptimizedPatternExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()