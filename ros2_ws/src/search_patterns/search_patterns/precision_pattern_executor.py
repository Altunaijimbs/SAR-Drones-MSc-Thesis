#!/usr/bin/env python3
"""
Precision Pattern Executor - Solves jittering and overshooting
Works WITH velocity coordinator, handles sharp turns specially
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String, Int32
import json
import math
import numpy as np
import threading
import time

class PrecisionPatternExecutor(Node):
    def __init__(self):
        super().__init__('precision_pattern_executor')
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers - Work WITH velocity coordinator to avoid jittering
        self.velocity_pub = self.create_publisher(
            Twist, '/velocity_command_2', 10  # Priority 2 for patterns
        )
        
        self.priority_pub = self.create_publisher(
            Int32, '/velocity_priority_2', 10
        )
        
        self.simple_cmd_pub = self.create_publisher(
            String, '/simple_command', 10  # For stop commands
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
        self.enhanced_waypoints = []  # Waypoints with turn handling
        self.current_waypoint_index = 0
        self.executing = False
        self.current_pose = None
        self.current_yaw = 0.0
        
        # Control parameters (INCREASED SPEEDS)
        self.position_threshold = 1.0  # Slightly looser for smooth transitions
        self.sharp_turn_threshold = math.radians(60)  # Only very sharp turns > 60°
        self.cruise_speed = 5.0  # m/s normal speed (DOUBLED!)
        self.turn_speed = 2.0  # m/s for sharp turns (QUADRUPLED!)
        self.approach_distance = 4.0  # Start slowing down
        self.stop_at_sharp_turns = False  # Don't stop, just slow down
        
        # Priority message
        self.priority_msg = Int32()
        self.priority_msg.data = 2
        
        # Control state
        self.control_mode = 'idle'  # idle, cruising, approaching, turning, stopping
        self.turn_state = 'none'  # none, decelerating, stopped, rotating, accelerating
        
        # Execution thread
        self.execution_thread = None
        
        # Control timer at 10Hz (smoother than 20Hz)
        self.create_timer(0.1, self.control_loop)
        
        # Status timer
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info('Precision Pattern Executor initialized - No jittering, no overshooting!')
    
    def update_current_pose(self, msg):
        """Update current position and orientation"""
        self.current_pose = msg
        q = msg.pose.orientation
        self.current_yaw = self.quaternion_to_yaw(q)
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def calculate_angle_change(self, prev_point, curr_point, next_point):
        """Calculate angle change at a waypoint"""
        # Vector from prev to curr
        v1 = np.array([curr_point[0] - prev_point[0], 
                       curr_point[1] - prev_point[1]])
        
        # Vector from curr to next
        v2 = np.array([next_point[0] - curr_point[0],
                       next_point[1] - curr_point[1]])
        
        # Normalize
        if np.linalg.norm(v1) > 0:
            v1 = v1 / np.linalg.norm(v1)
        if np.linalg.norm(v2) > 0:
            v2 = v2 / np.linalg.norm(v2)
        
        # Calculate angle
        cos_angle = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle_change = math.acos(cos_angle)
        
        return angle_change
    
    def enhance_waypoints(self):
        """Add turn information and intermediate points for sharp turns"""
        if not self.waypoints or not self.current_pose:
            return
        
        self.enhanced_waypoints = []
        prev_pos = [self.current_pose.pose.position.x, 
                    self.current_pose.pose.position.y]
        
        for i, waypoint in enumerate(self.waypoints):
            curr_pos = [waypoint['x'], waypoint['y']]
            
            # Check if this is a sharp turn
            if i < len(self.waypoints) - 1:
                next_pos = [self.waypoints[i+1]['x'], self.waypoints[i+1]['y']]
                angle_change = self.calculate_angle_change(prev_pos, curr_pos, next_pos)
                
                if angle_change > self.sharp_turn_threshold:
                    # Mark as sharp turn
                    waypoint['sharp_turn'] = True
                    waypoint['turn_angle'] = angle_change
                    
                    # Add approach point before the turn
                    approach_dist = 1.5  # meters before waypoint
                    direction = np.array(curr_pos) - np.array(prev_pos)
                    if np.linalg.norm(direction) > 0:
                        direction = direction / np.linalg.norm(direction)
                        approach_point = np.array(curr_pos) - direction * approach_dist
                        
                        # Add approach waypoint
                        self.enhanced_waypoints.append({
                            'x': approach_point[0],
                            'y': approach_point[1],
                            'z': waypoint['z'],
                            'type': 'approach',
                            'speed': self.turn_speed,
                            'original_index': i
                        })
                    
                    # Add the actual waypoint with turn flag
                    waypoint['type'] = 'turn'
                    waypoint['speed'] = self.turn_speed
                    self.enhanced_waypoints.append(waypoint)
                    
                    # Add departure point after the turn
                    if i < len(self.waypoints) - 1:
                        next_direction = np.array(next_pos) - np.array(curr_pos)
                        if np.linalg.norm(next_direction) > 0:
                            next_direction = next_direction / np.linalg.norm(next_direction)
                            departure_point = np.array(curr_pos) + next_direction * 1.5
                            
                            self.enhanced_waypoints.append({
                                'x': departure_point[0],
                                'y': departure_point[1],
                                'z': waypoint['z'],
                                'type': 'departure',
                                'speed': self.cruise_speed,
                                'original_index': i
                            })
                else:
                    # Normal waypoint
                    waypoint['sharp_turn'] = False
                    waypoint['type'] = 'normal'
                    waypoint['speed'] = self.cruise_speed
                    self.enhanced_waypoints.append(waypoint)
            else:
                # Last waypoint
                waypoint['sharp_turn'] = False
                waypoint['type'] = 'final'
                waypoint['speed'] = self.turn_speed
                self.enhanced_waypoints.append(waypoint)
            
            prev_pos = curr_pos
        
        self.get_logger().info(f'Enhanced {len(self.waypoints)} waypoints to {len(self.enhanced_waypoints)} with turn handling')
    
    def load_waypoints(self, msg):
        """Load waypoints and enhance them for sharp turns"""
        try:
            data = json.loads(msg.data)
            self.waypoints = data['waypoints']
            self.current_waypoint_index = 0
            
            # Enhance waypoints with turn handling
            self.enhance_waypoints()
            
            self.get_logger().info(f'Loaded and enhanced {len(self.waypoints)} waypoints')
            
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
        if not self.enhanced_waypoints:
            self.get_logger().warn('No waypoints loaded')
            return
        
        if self.executing:
            self.get_logger().warn('Already executing pattern')
            return
        
        self.executing = True
        self.current_waypoint_index = 0
        self.control_mode = 'cruising'
        
        # Start execution in separate thread
        self.execution_thread = threading.Thread(target=self.execute_pattern)
        self.execution_thread.start()
        
        self.get_logger().info('Started precision pattern execution')
    
    def stop_execution(self):
        """Stop pattern execution"""
        self.executing = False
        self.control_mode = 'idle'
        
        # Send stop command
        stop_cmd = String()
        stop_cmd.data = 'stop'
        self.simple_cmd_pub.publish(stop_cmd)
        
        if self.execution_thread and self.execution_thread.is_alive():
            self.execution_thread.join(timeout=2.0)
        
        self.get_logger().info('Stopped pattern execution')
    
    def pause_execution(self):
        """Pause at current position"""
        if self.executing:
            self.executing = False
            self.control_mode = 'idle'
            
            stop_cmd = String()
            stop_cmd.data = 'stop'
            self.simple_cmd_pub.publish(stop_cmd)
            
            self.get_logger().info('Paused pattern execution')
    
    def resume_execution(self):
        """Resume from current waypoint"""
        if not self.executing and self.enhanced_waypoints:
            self.executing = True
            self.control_mode = 'cruising'
            self.execution_thread = threading.Thread(target=self.execute_pattern)
            self.execution_thread.start()
            self.get_logger().info('Resumed pattern execution')
    
    def reset_execution(self):
        """Reset to first waypoint"""
        self.stop_execution()
        self.current_waypoint_index = 0
        self.get_logger().info('Reset pattern execution')
    
    def execute_pattern(self):
        """Main execution loop with precision control"""
        while self.executing and self.current_waypoint_index < len(self.enhanced_waypoints):
            waypoint = self.enhanced_waypoints[self.current_waypoint_index]
            
            # Log waypoint type
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_index + 1}/{len(self.enhanced_waypoints)}: '
                f'type={waypoint.get("type", "normal")}, '
                f'pos=({waypoint["x"]:.1f}, {waypoint["y"]:.1f}, {waypoint["z"]:.1f})'
            )
            
            # Move to waypoint
            reached = self.move_to_waypoint(waypoint)
            
            if reached and self.executing:
                # Special handling for turn waypoints
                if waypoint.get('type') == 'turn' and waypoint.get('sharp_turn') and self.stop_at_sharp_turns:
                    self.get_logger().info('Executing sharp turn')
                    # Brief stop at turn point (only if stop_at_sharp_turns is True)
                    stop_cmd = String()
                    stop_cmd.data = 'stop'
                    self.simple_cmd_pub.publish(stop_cmd)
                    time.sleep(0.5)
                
                self.current_waypoint_index += 1
        
        # Pattern completed
        if self.current_waypoint_index >= len(self.enhanced_waypoints):
            self.executing = False
            self.control_mode = 'idle'
            
            # Send final stop
            stop_cmd = String()
            stop_cmd.data = 'stop'
            self.simple_cmd_pub.publish(stop_cmd)
            
            self.get_logger().info('Pattern execution completed')
    
    def move_to_waypoint(self, waypoint):
        """Move to a single waypoint with appropriate speed"""
        if not self.current_pose:
            return False
        
        max_speed = waypoint.get('speed', self.cruise_speed)
        
        while self.executing:
            # Calculate distance and direction
            dx = waypoint['x'] - self.current_pose.pose.position.x
            dy = waypoint['y'] - self.current_pose.pose.position.y
            dz = waypoint['z'] - self.current_pose.pose.position.z
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # Check if reached
            if distance < self.position_threshold:
                return True
            
            # Calculate velocity
            if distance > self.approach_distance:
                speed = max_speed
                self.control_mode = 'cruising'
            else:
                # Linear deceleration
                speed = max(self.turn_speed, max_speed * (distance / self.approach_distance))
                self.control_mode = 'approaching'
            
            # Normalize direction and apply speed
            if distance > 0:
                vx = (dx / distance) * speed
                vy = (dy / distance) * speed
                vz = (dz / distance) * speed
            else:
                vx = vy = vz = 0
            
            # Send velocity command
            self.send_velocity_command(vx, vy, vz)
            
            # Small delay
            time.sleep(0.1)
        
        return False
    
    def send_velocity_command(self, vx, vy, vz):
        """Send velocity command with priority"""
        # Publish priority
        self.priority_pub.publish(self.priority_msg)
        
        # Send velocity
        vel_cmd = Twist()
        vel_cmd.linear.x = vx
        vel_cmd.linear.y = vy
        vel_cmd.linear.z = vz
        self.velocity_pub.publish(vel_cmd)
    
    def control_loop(self):
        """Periodic control update"""
        # Keep publishing priority if executing
        if self.executing:
            self.priority_pub.publish(self.priority_msg)
    
    def publish_status(self):
        """Publish pattern execution status"""
        status = {
            'executing': self.executing,
            'control_mode': self.control_mode,
            'total_waypoints': len(self.enhanced_waypoints),
            'current_waypoint': self.current_waypoint_index + 1 if self.executing else 0,
            'progress': (self.current_waypoint_index / len(self.enhanced_waypoints) * 100) if self.enhanced_waypoints else 0
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionPatternExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()