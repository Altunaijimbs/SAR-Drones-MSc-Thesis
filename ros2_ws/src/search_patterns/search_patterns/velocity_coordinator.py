#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import time


class VelocityCoordinator(Node):
    """
    Coordinates velocity commands from multiple sources to prevent conflicts
    Priority order:
    1. Emergency stop
    2. Search pattern commands
    3. Obstacle avoidance
    4. LLM/manual commands
    5. Keep-alive (lowest priority)
    """
    
    def __init__(self):
        super().__init__('velocity_coordinator')
        
        # Parameters
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('publish_rate', 20.0)  # Hz
        
        # Publishers
        self.cmd_pub = self.create_publisher(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/velocity_coordinator/active_source',
            10
        )
        
        # Subscribers for different command sources
        
        # Keep-alive commands (lowest priority)
        self.keepalive_sub = self.create_subscription(
            Twist,
            '/keepalive/velocity_command',
            lambda msg: self.velocity_callback(msg, 'keepalive', 0),
            10
        )
        
        # LLM/manual commands
        self.llm_sub = self.create_subscription(
            Twist,
            '/llm/velocity_command',
            lambda msg: self.velocity_callback(msg, 'llm', 1),
            10
        )
        
        # Hybrid position controller (higher priority than LLM)
        self.hybrid_sub = self.create_subscription(
            Twist,
            '/hybrid_position/velocity_command',
            lambda msg: self.velocity_callback(msg, 'hybrid', 2),
            10
        )
        
        # Pattern executor commands (priority 2)
        self.pattern_sub = self.create_subscription(
            Twist,
            '/velocity_command_2',
            lambda msg: self.velocity_callback(msg, 'pattern', 2),
            10
        )
        
        # Obstacle avoidance commands
        self.avoidance_sub = self.create_subscription(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped_safe',
            lambda msg: self.velocity_callback(msg, 'avoidance', 3),
            10
        )
        
        # Search pattern commands (high priority)
        self.search_sub = self.create_subscription(
            Twist,
            '/search_pattern/velocity_command',
            lambda msg: self.velocity_callback(msg, 'search', 4),
            10
        )
        
        # Return to home commands (higher priority than search)
        self.rth_sub = self.create_subscription(
            Twist,
            '/rth/velocity_command',
            lambda msg: self.velocity_callback(msg, 'rth', 5),
            10
        )
        
        # Emergency stop (highest priority)
        self.emergency_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            10
        )
        
        # Command storage with priorities
        self.commands = {
            'keepalive': {'vel': Twist(), 'time': 0, 'priority': 0},
            'llm': {'vel': Twist(), 'time': 0, 'priority': 1},
            'hybrid': {'vel': Twist(), 'time': 0, 'priority': 2},
            'pattern': {'vel': Twist(), 'time': 0, 'priority': 2},
            'avoidance': {'vel': Twist(), 'time': 0, 'priority': 3},
            'search': {'vel': Twist(), 'time': 0, 'priority': 4},
            'rth': {'vel': Twist(), 'time': 0, 'priority': 5},
            'emergency': {'vel': Twist(), 'time': 0, 'priority': 99}
        }
        
        self.active_source = 'none'
        self.emergency_stop = False
        
        # Main publishing timer
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0/rate, self.publish_velocity)
        
        self.get_logger().info('Velocity Coordinator initialized')
    
    def velocity_callback(self, msg: Twist, source: str, priority: int):
        """Store velocity command with timestamp"""
        self.commands[source]['vel'] = msg
        self.commands[source]['time'] = time.time()
        self.commands[source]['priority'] = priority
    
    def emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.commands['emergency']['vel'] = Twist()  # All zeros
            self.commands['emergency']['time'] = time.time()
            self.get_logger().error('EMERGENCY STOP ACTIVATED')
    
    def publish_velocity(self):
        """Select and publish the highest priority valid command"""
        timeout = self.get_parameter('command_timeout').value
        current_time = time.time()
        
        # If emergency stop is active, use it
        if self.emergency_stop:
            self.cmd_pub.publish(Twist())
            self.update_status('emergency')
            return
        
        # Find highest priority command that isn't timed out
        best_source = None
        best_priority = -1
        
        for source, data in self.commands.items():
            if source == 'emergency':
                continue
                
            # Check if command is recent enough
            if (current_time - data['time']) < timeout:
                if data['priority'] > best_priority:
                    best_priority = data['priority']
                    best_source = source
        
        # Publish the selected command or hover
        if best_source:
            self.cmd_pub.publish(self.commands[best_source]['vel'])
            self.update_status(best_source)
        else:
            # No valid commands - hover
            self.cmd_pub.publish(Twist())
            self.update_status('hover')
    
    def update_status(self, source: str):
        """Update and publish active source status"""
        if source != self.active_source:
            self.active_source = source
            status_msg = String()
            status_msg.data = f"Active: {source}"
            self.status_pub.publish(status_msg)
            self.get_logger().debug(f'Velocity source: {source}')


def main(args=None):
    rclpy.init(args=args)
    node = VelocityCoordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()