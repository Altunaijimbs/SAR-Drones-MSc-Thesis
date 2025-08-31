#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
import time
import json


class ImprovedVelocityCoordinator(Node):
    """
    Improved Velocity Coordinator with better priority modes
    Fixes issue where simple commands get blocked
    """
    
    def __init__(self):
        super().__init__('improved_velocity_coordinator')
        
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
            '/velocity_coordinator/status',
            10
        )
        
        # Improved priority modes with clearer behavior
        self.priority_modes = {
            'pure_manual': {
                # ONLY manual commands work
                'keepalive': 100,    # Highest priority
                'llm': 0,            # Disabled
                'hybrid': 0,         # Disabled
                'pattern': 0,        # Disabled
                'search': 0,         # Disabled
                'avoidance': 50,     # Still active for safety
                'rth': 0,            # Disabled (manual control only)
                'emergency': 999     # Always highest
            },
            'manual_priority': {
                # Manual commands have priority but others still work
                'keepalive': 80,     # High priority
                'llm': 30,           # Lower
                'hybrid': 20,        # Lower
                'pattern': 10,       # Lowest
                'search': 10,        # Lowest
                'avoidance': 70,     # High for safety
                'rth': 90,           # Very high (safety)
                'emergency': 999     # Always highest
            },
            'llm_only': {
                # ONLY LLM commands work
                'keepalive': 0,      # Disabled
                'llm': 100,          # Highest priority
                'hybrid': 50,        # Support for LLM position commands
                'pattern': 0,        # Disabled
                'search': 0,         # Disabled
                'avoidance': 70,     # Still active for safety
                'rth': 80,           # Safety override
                'emergency': 999     # Always highest
            },
            'hybrid_balanced': {
                # Everything works with balanced priority
                'keepalive': 50,     # Medium
                'llm': 50,           # Medium
                'hybrid': 60,        # Slightly higher for smooth position control
                'pattern': 40,       # Medium-low
                'search': 40,        # Medium-low
                'avoidance': 70,     # High for safety
                'rth': 80,           # Higher for safety
                'emergency': 999     # Always highest
            },
            'pattern_focus': {
                # Patterns have priority but can be overridden
                'keepalive': 20,     # Low
                'llm': 20,           # Low
                'hybrid': 30,        # Low-medium
                'pattern': 80,       # High priority
                'search': 80,        # High priority
                'avoidance': 70,     # Still high for safety
                'rth': 90,           # Override patterns for safety
                'emergency': 999     # Always highest
            },
            'autonomous': {
                # Fully autonomous - patterns and LLM have control
                'keepalive': 10,     # Very low (just for emergency override)
                'llm': 70,           # High
                'hybrid': 70,        # High
                'pattern': 70,       # High
                'search': 70,        # High
                'avoidance': 80,     # Higher for safety
                'rth': 90,           # Highest normal priority
                'emergency': 999     # Always highest
            },
            'emergency': {
                # Emergency mode - only safety commands
                'keepalive': 50,     # Allow manual control
                'llm': 0,            # Disabled
                'hybrid': 0,         # Disabled
                'pattern': 0,        # Disabled
                'search': 0,         # Disabled
                'avoidance': 100,    # Highest
                'rth': 100,          # Highest
                'emergency': 999     # Always highest
            }
        }
        
        # Current priority mode - default to manual_priority for testing
        self.current_mode = 'manual_priority'
        
        # Temporary priority overrides
        self.temp_overrides = {}
        
        # Subscribers for different command sources
        
        # Keep-alive/simple commands
        self.keepalive_sub = self.create_subscription(
            Twist,
            '/keepalive/velocity_command',
            lambda msg: self.velocity_callback(msg, 'keepalive'),
            10
        )
        
        # LLM/manual commands
        self.llm_sub = self.create_subscription(
            Twist,
            '/llm/velocity_command',
            lambda msg: self.velocity_callback(msg, 'llm'),
            10
        )
        
        # Hybrid position controller
        self.hybrid_sub = self.create_subscription(
            Twist,
            '/hybrid_position/velocity_command',
            lambda msg: self.velocity_callback(msg, 'hybrid'),
            10
        )
        
        # Pattern executor commands
        self.pattern_sub = self.create_subscription(
            Twist,
            '/velocity_command_2',
            lambda msg: self.velocity_callback(msg, 'pattern'),
            10
        )
        
        # Search pattern commands (from tuned sharpy, etc)
        self.search_sub = self.create_subscription(
            Twist,
            '/search_pattern/velocity_command',
            lambda msg: self.velocity_callback(msg, 'search'),
            10
        )
        
        # Obstacle avoidance commands
        self.avoidance_sub = self.create_subscription(
            Twist,
            '/mavros/setpoint_velocity/cmd_vel_unstamped_safe',
            lambda msg: self.velocity_callback(msg, 'avoidance'),
            10
        )
        
        # Return to home commands
        self.rth_sub = self.create_subscription(
            Twist,
            '/rth/velocity_command',
            lambda msg: self.velocity_callback(msg, 'rth'),
            10
        )
        
        # Emergency stop
        self.emergency_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.emergency_stop_callback,
            10
        )
        
        # Priority mode control subscriber
        self.priority_mode_sub = self.create_subscription(
            String,
            '/priority_mode',
            self.priority_mode_callback,
            10
        )
        
        # Web interface priority control
        self.web_priority_sub = self.create_subscription(
            String,
            '/velocity_coordinator/priority',
            self.web_priority_callback,
            10
        )
        
        # Command storage with dynamic priorities
        self.commands = {
            'keepalive': {'vel': Twist(), 'time': 0},
            'llm': {'vel': Twist(), 'time': 0},
            'hybrid': {'vel': Twist(), 'time': 0},
            'pattern': {'vel': Twist(), 'time': 0},
            'search': {'vel': Twist(), 'time': 0},
            'avoidance': {'vel': Twist(), 'time': 0},
            'rth': {'vel': Twist(), 'time': 0},
            'emergency': {'vel': Twist(), 'time': 0}
        }
        
        self.active_source = 'none'
        self.emergency_stop = False
        
        # Stats tracking
        self.command_counts = {source: 0 for source in self.commands.keys()}
        self.last_status_time = 0
        
        # Main publishing timer
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0/rate, self.publish_velocity)
        
        # Status publishing timer (slower rate)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info(f'Improved Velocity Coordinator initialized in {self.current_mode} mode')
        self.log_priority_mode()
    
    def log_priority_mode(self):
        """Log current priority settings"""
        self.get_logger().info(f'Priority mode: {self.current_mode}')
        priorities = self.priority_modes[self.current_mode]
        active_sources = [f"{src}:{pri}" for src, pri in priorities.items() if pri > 0]
        self.get_logger().info(f'Active sources: {", ".join(active_sources)}')
    
    def get_priority(self, source):
        """Get current priority for a source"""
        # Check for temporary override
        if source in self.temp_overrides:
            return self.temp_overrides[source]
        
        # Use mode-based priority
        return self.priority_modes[self.current_mode].get(source, 0)
    
    def velocity_callback(self, msg: Twist, source: str):
        """Store velocity command with timestamp"""
        self.commands[source]['vel'] = msg
        self.commands[source]['time'] = time.time()
        self.command_counts[source] += 1
    
    def emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.get_logger().warn('EMERGENCY STOP ACTIVATED')
            # Create stop command
            stop = Twist()
            self.commands['emergency']['vel'] = stop
            self.commands['emergency']['time'] = time.time()
    
    def priority_mode_callback(self, msg: String):
        """Change priority mode"""
        if msg.data in self.priority_modes:
            old_mode = self.current_mode
            self.current_mode = msg.data
            self.get_logger().info(f'Priority mode changed: {old_mode} -> {self.current_mode}')
            self.log_priority_mode()
        else:
            self.get_logger().warn(f'Unknown priority mode: {msg.data}')
            self.get_logger().info(f'Available modes: {list(self.priority_modes.keys())}')
    
    def web_priority_callback(self, msg: String):
        """Handle priority changes from web interface"""
        # Map web interface names to our modes
        web_to_mode = {
            'manual': 'manual_priority',
            'pattern': 'pattern_focus',
            'balanced': 'hybrid_balanced',
            'emergency': 'emergency',
            'pure_manual': 'pure_manual',
            'llm_only': 'llm_only',
            'autonomous': 'autonomous'
        }
        
        if msg.data in web_to_mode:
            new_mode = web_to_mode[msg.data]
            if new_mode != self.current_mode:
                old_mode = self.current_mode
                self.current_mode = new_mode
                self.get_logger().info(f'Web priority change: {old_mode} -> {self.current_mode}')
                self.log_priority_mode()
    
    def publish_velocity(self):
        """Publish highest priority velocity command"""
        timeout = self.get_parameter('command_timeout').value
        current_time = time.time()
        
        # Find highest priority valid command
        best_source = None
        best_priority = -1
        
        for source, data in self.commands.items():
            # Check if command is recent enough
            if (current_time - data['time']) < timeout:
                priority = self.get_priority(source)
                if priority > best_priority:
                    best_priority = priority
                    best_source = source
        
        # Publish the best command
        if best_source:
            cmd = self.commands[best_source]['vel']
            self.cmd_pub.publish(cmd)
            
            if self.active_source != best_source:
                self.active_source = best_source
                self.get_logger().debug(f'Active source: {best_source} (priority: {best_priority})')
        else:
            # No valid commands, publish zero
            self.cmd_pub.publish(Twist())
            if self.active_source != 'none':
                self.active_source = 'none'
                self.get_logger().debug('No active command sources')
    
    def publish_status(self):
        """Publish detailed status for monitoring"""
        status = {
            'priority_mode': self.current_mode,
            'active_source': self.active_source,
            'command_counts': self.command_counts,
            'priorities': self.priority_modes[self.current_mode],
            'timestamp': time.time()
        }
        
        self.status_pub.publish(String(data=json.dumps(status)))
        
        # Log periodically if there's activity
        if self.active_source != 'none':
            total_commands = sum(self.command_counts.values())
            if total_commands % 100 == 0:  # Every 100 commands
                self.get_logger().info(
                    f'Mode: {self.current_mode} | Active: {self.active_source} | '
                    f'Commands: {total_commands}'
                )

def main(args=None):
    rclpy.init(args=args)
    node = ImprovedVelocityCoordinator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()