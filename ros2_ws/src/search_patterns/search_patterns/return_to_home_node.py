#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, PoseStamped, Point
from mavros_msgs.msg import State
import math


class ReturnToHomeNode(Node):
    """
    Return to Home (RTH) controller that brings drone back to starting position
    """
    
    def __init__(self):
        super().__init__('return_to_home_node')
        
        # Parameters
        self.declare_parameter('return_speed', 2.0)  # m/s
        self.declare_parameter('position_tolerance', 1.0)  # meters
        self.declare_parameter('min_altitude_for_home', 2.0)  # meters - minimum altitude to save home position
        self.declare_parameter('auto_set_home', True)  # automatically set home when conditions are met
        
        self.return_speed = self.get_parameter('return_speed').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.min_altitude = self.get_parameter('min_altitude_for_home').value
        self.auto_set_home = self.get_parameter('auto_set_home').value
        
        # QoS profile to match MAVROS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.vel_pub = self.create_publisher(
            Twist,
            '/rth/velocity_command',
            10
        )
        
        self.emergency_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile
        )
        
        self.rth_cmd_sub = self.create_subscription(
            String,
            '/rth_command',
            self.rth_command_callback,
            10
        )
        
        self.simple_cmd_sub = self.create_subscription(
            String,
            '/simple_command',
            self.simple_command_callback,
            10
        )
        
        # Subscribe to MAVROS state
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10
        )
        
        # Manual home set command
        self.set_home_sub = self.create_subscription(
            String,
            '/set_home_position',
            self.set_home_callback,
            10
        )
        
        # State variables
        self.current_pose = None
        self.home_position = None
        self.initial_position_saved = False
        self.rth_active = False
        self.is_armed = False
        self.mode = None
        
        # Control timer
        self.control_timer = self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('Return to Home Node initialized')
    
    def pose_callback(self, msg: PoseStamped):
        """Update current position and save initial position when conditions are met"""
        self.current_pose = msg.pose
        
        # Auto-save home position when:
        # 1. Not already saved
        # 2. Drone is armed and in OFFBOARD mode
        # 3. Altitude is above minimum threshold
        if (self.auto_set_home and 
            not self.initial_position_saved and 
            self.current_pose is not None and
            self.is_armed and 
            self.mode == 'OFFBOARD' and
            self.current_pose.position.z >= self.min_altitude):
            
            # Add delay to ensure stable position after takeoff
            if not hasattr(self, '_home_save_timer'):
                self._home_save_timer = self.create_timer(2.0, self._delayed_home_save)
                self.get_logger().info(f'Conditions met for auto-home save. Altitude: {self.current_pose.position.z:.2f}m, waiting 2s for stable position...')
    
    def _delayed_home_save(self):
        """Save home after delay to ensure stable position"""
        if hasattr(self, '_home_save_timer'):
            self._home_save_timer.cancel()
            self._home_save_timer.destroy()
            del self._home_save_timer
        
        if not self.initial_position_saved and self.current_pose is not None:
            self.save_home_position()
    
    def state_callback(self, msg: State):
        """Update drone state"""
        prev_armed = self.is_armed
        prev_mode = self.mode
        
        self.is_armed = msg.armed
        self.mode = msg.mode
        
        # Log state changes for debugging
        if self.is_armed != prev_armed or self.mode != prev_mode:
            self.get_logger().info(f'State changed - Armed: {self.is_armed}, Mode: {self.mode}')
    
    def set_home_callback(self, msg: String):
        """Manually set home position"""
        if msg.data.lower() == 'set' and self.current_pose is not None:
            self.save_home_position()
            self.get_logger().info('Home position manually set')
    
    def save_home_position(self):
        """Save current position as home"""
        if self.current_pose is None:
            return
            
        self.home_position = Point()
        self.home_position.x = self.current_pose.position.x
        self.home_position.y = self.current_pose.position.y
        self.home_position.z = self.current_pose.position.z
        self.initial_position_saved = True
        self.get_logger().warn(f'HOME POSITION SAVED: ({self.home_position.x:.2f}, {self.home_position.y:.2f}, {self.home_position.z:.2f})')
    
    def rth_command_callback(self, msg: String):
        """Handle RTH commands"""
        cmd = msg.data.lower()
        
        if 'stop' in cmd or 'return' in cmd or 'home' in cmd or 'rth' in cmd:
            self.activate_rth()
        elif 'cancel' in cmd or 'abort' in cmd:
            self.cancel_rth()
    
    def simple_command_callback(self, msg: String):
        """Handle simple commands"""
        cmd = msg.data.lower()
        
        # Cancel RTH if any movement command is given
        if cmd in ['forward', 'back', 'backward', 'left', 'right', 'up', 'down', 'stop']:
            if self.rth_active:
                self.get_logger().info('RTH cancelled by manual control')
                self.cancel_rth()
        # Only activate RTH for explicit RTH commands
        elif cmd == 'rth' or 'return' in cmd:
            self.activate_rth()
    
    def activate_rth(self):
        """Activate return to home"""
        if self.home_position is None:
            self.get_logger().warn('Cannot return home - no home position saved!')
            return
        
        # Don't send emergency stop - it overrides RTH commands!
        # Just activate RTH
        self.rth_active = True
        self.get_logger().warn(f'RETURN TO HOME ACTIVATED! Target: ({self.home_position.x:.2f}, {self.home_position.y:.2f}, {self.home_position.z:.2f})')
    
    def cancel_rth(self):
        """Cancel return to home"""
        self.rth_active = False
        
        # Send stop velocity
        stop_vel = Twist()
        self.vel_pub.publish(stop_vel)
        
        self.get_logger().info('Return to home cancelled')
    
    def control_loop(self):
        """Main control loop for RTH"""
        if not self.rth_active or self.current_pose is None or self.home_position is None:
            return
        
        # Calculate distance to home
        dx = self.home_position.x - self.current_pose.position.x
        dy = self.home_position.y - self.current_pose.position.y
        dz = self.home_position.z - self.current_pose.position.z
        
        # DEBUG: Log positions and deltas
        self.get_logger().warn(f'[RTH DEBUG] Current: X={self.current_pose.position.x:.2f}, Y={self.current_pose.position.y:.2f}, Z={self.current_pose.position.z:.2f}')
        self.get_logger().warn(f'[RTH DEBUG] Home: X={self.home_position.x:.2f}, Y={self.home_position.y:.2f}, Z={self.home_position.z:.2f}')
        self.get_logger().warn(f'[RTH DEBUG] Delta: dX={dx:.2f}, dY={dy:.2f}, dZ={dz:.2f}')
        
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Check if reached home
        if distance < self.position_tolerance:
            self.get_logger().info('Reached home position!')
            self.rth_active = False
            
            # Send final stop
            stop_vel = Twist()
            self.vel_pub.publish(stop_vel)
            return
        
        # Calculate velocity towards home
        vel_cmd = Twist()
        
        if distance > 0:
            # Normalize and scale by return speed
            vel_cmd.linear.x = (dx / distance) * self.return_speed
            vel_cmd.linear.y = (dy / distance) * self.return_speed
            vel_cmd.linear.z = (dz / distance) * min(self.return_speed, 1.0)
        
        # Transform to drone frame
        # Based on working simple commands:
        # - To go forward (North/+Y in ROS) → vel.y should be positive
        # - To go right (East/+X in ROS) → vel.x should be positive
        # So NO transformation needed! The ROS frame matches the body frame!
        transformed_vel = Twist()
        transformed_vel.linear.x = vel_cmd.linear.x   # Keep X as X
        transformed_vel.linear.y = vel_cmd.linear.y   # Keep Y as Y
        transformed_vel.linear.z = vel_cmd.linear.z   # Keep Z as Z
        
        # DEBUG: Log velocities
        self.get_logger().warn(f'[RTH DEBUG] ROS Vel: X={vel_cmd.linear.x:.2f}, Y={vel_cmd.linear.y:.2f}, Z={vel_cmd.linear.z:.2f}')
        self.get_logger().warn(f'[RTH DEBUG] UE4 Vel: X={transformed_vel.linear.x:.2f}, Y={transformed_vel.linear.y:.2f}, Z={transformed_vel.linear.z:.2f}')
        
        # Publish RTH velocity
        self.vel_pub.publish(transformed_vel)
        
        # Log progress every 10 calls (1 second)
        if hasattr(self, '_log_counter'):
            self._log_counter += 1
        else:
            self._log_counter = 0
            
        if self._log_counter % 10 == 0:
            self.get_logger().info(f'Returning home... Distance: {distance:.1f}m')


def main(args=None):
    rclpy.init(args=args)
    node = ReturnToHomeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()