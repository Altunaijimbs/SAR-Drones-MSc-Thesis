#!/usr/bin/env python3
"""
Advanced LLM Controller with Custom Search Pattern Support
Supports custom patterns like expanding squares, spirals, and more
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point, PoseStamped
from std_msgs.msg import String, Bool
from mavros_msgs.msg import State
from openai import OpenAI
import json
import re
import math
import os
import threading
import time

class AdvancedLLMController(Node):
    def __init__(self):
        super().__init__('advanced_llm_controller')
        
        # OpenAI setup
        self.declare_parameter('openai_api_key', '')
        self.api_key = self.get_parameter('openai_api_key').value or os.getenv('OPENAI_API_KEY')
        self.client = OpenAI(api_key=self.api_key)
        
        # Publishers
        self.search_pub = self.create_publisher(String, '/search_command', 10)
        self.simple_pub = self.create_publisher(String, '/simple_command', 10)
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # Subscribers
        self.llm_input_sub = self.create_subscription(
            String, '/llm/command_input', 
            self.process_natural_command, 10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.update_position, 10
        )
        
        self.detection_sub = self.create_subscription(
            String, '/drone/scene_description',
            self.handle_detections, 10
        )
        
        # State
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.executing_pattern = False
        self.pattern_thread = None
        self.stop_pattern = False
        self.hover_on_detection = False
        self.pattern_waypoints = []
        self.current_waypoint_index = 0
        
        self.get_logger().info('Advanced LLM Controller ready with custom pattern support')
    
    def update_position(self, msg):
        """Update current drone position"""
        self.current_position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
    
    def process_natural_command(self, msg):
        """Process natural language commands with GPT"""
        command = msg.data
        self.get_logger().info(f'Processing: "{command}"')
        
        # Enhanced prompt for custom patterns
        system_prompt = """
        You are an advanced drone command parser. Parse natural language into structured commands.
        
        Supported patterns:
        - grid: Standard grid search (existing)
        - expanding_square: Start small, expand outward in squares
        - spiral: Spiral outward from center
        - circle: Circular pattern
        - zigzag: Back and forth pattern
        - custom_waypoints: Specific waypoints provided
        
        Extract:
        - action: (search/hover/return/move/pattern)
        - pattern_type: (grid/expanding_square/spiral/circle/zigzag/custom)
        - pattern_params: {
            - size: initial size or radius in meters
            - expand_factor: how much to expand each iteration (for expanding patterns)
            - spacing: distance between lines/passes
            - iterations: number of expansions or circles
            - waypoints: list of [x,y,z] for custom patterns
        }
        - hover_on_detection: true/false
        - target_object: what to search for (person, vehicle, etc)
        
        Examples:
        "Do an expanding square search starting at 10 meters" ->
        {
            "action": "pattern",
            "pattern_type": "expanding_square",
            "pattern_params": {
                "size": 10,
                "expand_factor": 10,
                "iterations": 4
            },
            "hover_on_detection": true,
            "target_object": "person"
        }
        
        Return as JSON.
        """
        
        try:
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": command}
                ],
                temperature=0
            )
            
            parsed = json.loads(response.choices[0].message.content)
            self.get_logger().info(f"Parsed command: {json.dumps(parsed, indent=2)}")
            
            # Execute based on parsed command
            if parsed['action'] == 'pattern':
                self.execute_custom_pattern(parsed)
            elif parsed['action'] == 'search':
                # Default grid search
                self.execute_search()
            elif parsed['action'] == 'hover':
                self.execute_hover()
            elif parsed['action'] == 'return':
                self.execute_return()
                
        except Exception as e:
            self.get_logger().error(f"LLM parsing error: {e}")
            # Fallback to simple parsing
            self.simple_parse_and_execute(command)
    
    def simple_parse_and_execute(self, command):
        """Simple fallback parser"""
        command_lower = command.lower()
        
        if 'expanding square' in command_lower:
            # Extract size if mentioned
            size_match = re.search(r'(\d+)\s*meter', command_lower)
            size = int(size_match.group(1)) if size_match else 10
            
            self.execute_custom_pattern({
                'pattern_type': 'expanding_square',
                'pattern_params': {
                    'size': size,
                    'expand_factor': 10,
                    'iterations': 4
                },
                'hover_on_detection': True
            })
        elif 'spiral' in command_lower:
            self.execute_custom_pattern({
                'pattern_type': 'spiral',
                'pattern_params': {
                    'radius': 20,
                    'spacing': 5
                },
                'hover_on_detection': True
            })
        elif 'search' in command_lower or 'find' in command_lower:
            self.execute_search()
        elif 'hover' in command_lower:
            self.execute_hover()
        elif 'return' in command_lower or 'home' in command_lower:
            self.execute_return()
    
    def execute_custom_pattern(self, params):
        """Execute custom search patterns"""
        pattern_type = params.get('pattern_type', 'grid')
        pattern_params = params.get('pattern_params', {})
        self.hover_on_detection = params.get('hover_on_detection', True)
        
        self.get_logger().info(f"Executing {pattern_type} pattern with params: {pattern_params}")
        
        # Stop any existing pattern
        if self.executing_pattern:
            self.stop_pattern = True
            if self.pattern_thread:
                self.pattern_thread.join()
        
        # Generate waypoints based on pattern type
        if pattern_type == 'expanding_square':
            self.pattern_waypoints = self.generate_expanding_square_waypoints(pattern_params)
        elif pattern_type == 'spiral':
            self.pattern_waypoints = self.generate_spiral_waypoints(pattern_params)
        elif pattern_type == 'circle':
            self.pattern_waypoints = self.generate_circle_waypoints(pattern_params)
        elif pattern_type == 'zigzag':
            self.pattern_waypoints = self.generate_zigzag_waypoints(pattern_params)
        elif pattern_type == 'custom':
            self.pattern_waypoints = pattern_params.get('waypoints', [])
        else:
            # Default to standard grid search
            self.execute_search()
            return
        
        # Start pattern execution in separate thread
        self.stop_pattern = False
        self.executing_pattern = True
        self.current_waypoint_index = 0
        self.pattern_thread = threading.Thread(target=self.execute_waypoint_pattern)
        self.pattern_thread.start()
    
    def generate_expanding_square_waypoints(self, params):
        """Generate waypoints for expanding square pattern"""
        size = params.get('size', 10)
        expand_factor = params.get('expand_factor', 10)
        iterations = params.get('iterations', 4)
        
        waypoints = []
        current_pos = self.current_position
        base_x, base_y, z = current_pos['x'], current_pos['y'], current_pos['z']
        
        for i in range(iterations):
            current_size = size + (i * expand_factor)
            half_size = current_size / 2
            
            # Square corners (clockwise)
            corners = [
                (base_x + half_size, base_y + half_size, z),  # Top-right
                (base_x + half_size, base_y - half_size, z),  # Bottom-right
                (base_x - half_size, base_y - half_size, z),  # Bottom-left
                (base_x - half_size, base_y + half_size, z),  # Top-left
                (base_x + half_size, base_y + half_size, z),  # Back to start
            ]
            
            waypoints.extend(corners)
        
        return waypoints
    
    def generate_spiral_waypoints(self, params):
        """Generate waypoints for spiral pattern"""
        max_radius = params.get('radius', 30)
        spacing = params.get('spacing', 5)
        
        waypoints = []
        current_pos = self.current_position
        center_x, center_y, z = current_pos['x'], current_pos['y'], current_pos['z']
        
        # Archimedean spiral
        angle = 0
        while True:
            radius = spacing * angle / (2 * math.pi)
            if radius > max_radius:
                break
                
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            waypoints.append((x, y, z))
            
            angle += 0.5  # Increment angle for next point
        
        return waypoints
    
    def generate_circle_waypoints(self, params):
        """Generate waypoints for circular pattern"""
        radius = params.get('radius', 20)
        num_points = params.get('points', 16)
        
        waypoints = []
        current_pos = self.current_position
        center_x, center_y, z = current_pos['x'], current_pos['y'], current_pos['z']
        
        for i in range(num_points + 1):  # +1 to close the circle
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            waypoints.append((x, y, z))
        
        return waypoints
    
    def generate_zigzag_waypoints(self, params):
        """Generate waypoints for zigzag pattern"""
        width = params.get('width', 30)
        length = params.get('length', 30)
        spacing = params.get('spacing', 5)
        
        waypoints = []
        current_pos = self.current_position
        start_x, start_y, z = current_pos['x'], current_pos['y'], current_pos['z']
        
        num_lines = int(width / spacing)
        
        for i in range(num_lines):
            y_offset = i * spacing
            
            if i % 2 == 0:
                # Move forward
                waypoints.append((start_x, start_y + y_offset, z))
                waypoints.append((start_x + length, start_y + y_offset, z))
            else:
                # Move backward
                waypoints.append((start_x + length, start_y + y_offset, z))
                waypoints.append((start_x, start_y + y_offset, z))
        
        return waypoints
    
    def execute_waypoint_pattern(self):
        """Execute the generated waypoint pattern"""
        self.get_logger().info(f"Starting pattern with {len(self.pattern_waypoints)} waypoints")
        
        for i, waypoint in enumerate(self.pattern_waypoints):
            if self.stop_pattern:
                self.get_logger().info("Pattern execution stopped")
                break
            
            self.current_waypoint_index = i
            self.fly_to_waypoint(waypoint)
            
            # Small pause at each waypoint
            time.sleep(0.5)
        
        self.executing_pattern = False
        self.get_logger().info("Pattern execution complete")
    
    def fly_to_waypoint(self, waypoint):
        """Fly to a specific waypoint using velocity control"""
        target_x, target_y, target_z = waypoint
        
        self.get_logger().info(f"Flying to waypoint: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})")
        
        while not self.stop_pattern:
            # Calculate distance to target
            dx = target_x - self.current_position['x']
            dy = target_y - self.current_position['y']
            dz = target_z - self.current_position['z']
            
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            if distance < 0.5:  # Within 0.5m of waypoint
                break
            
            # Calculate velocity commands (simple P controller)
            max_vel = 2.0  # m/s
            vel_x = max(-max_vel, min(max_vel, dx * 0.5))
            vel_y = max(-max_vel, min(max_vel, dy * 0.5))
            vel_z = max(-max_vel, min(max_vel, dz * 0.5))
            
            # Publish velocity command
            vel_cmd = Twist()
            vel_cmd.linear.x = vel_x
            vel_cmd.linear.y = vel_y
            vel_cmd.linear.z = vel_z
            self.vel_pub.publish(vel_cmd)
            
            time.sleep(0.1)  # 10Hz control loop
        
        # Stop at waypoint
        stop_cmd = Twist()
        self.vel_pub.publish(stop_cmd)
    
    def handle_detections(self, msg):
        """Handle detections during pattern execution"""
        if not self.executing_pattern or not self.hover_on_detection:
            return
        
        try:
            data = json.loads(msg.data)
            detections = data.get('detections', [])
            
            # Check for person detections
            people = [d for d in detections if d.get('label') == 'person']
            
            if people:
                self.get_logger().warn(f"DETECTION: Found {len(people)} person(s)!")
                
                # Pause pattern execution
                self.stop_pattern = True
                
                # Hover for 5 seconds
                stop_cmd = String()
                stop_cmd.data = 'stop'
                self.simple_pub.publish(stop_cmd)
                
                # Resume after delay
                threading.Timer(5.0, self.resume_pattern).start()
                
        except Exception as e:
            self.get_logger().error(f"Detection processing error: {e}")
    
    def resume_pattern(self):
        """Resume pattern execution after hovering"""
        self.get_logger().info("Resuming pattern execution")
        self.stop_pattern = False
        
        # Continue from current waypoint
        remaining_waypoints = self.pattern_waypoints[self.current_waypoint_index:]
        self.pattern_waypoints = remaining_waypoints
        self.current_waypoint_index = 0
        
        # Restart pattern thread
        self.pattern_thread = threading.Thread(target=self.execute_waypoint_pattern)
        self.pattern_thread.start()
    
    def execute_search(self):
        """Execute standard grid search"""
        search_cmd = String()
        search_cmd.data = 'search'
        self.search_pub.publish(search_cmd)
    
    def execute_hover(self):
        """Execute hover command"""
        cmd = String()
        cmd.data = 'stop'
        self.simple_pub.publish(cmd)
    
    def execute_return(self):
        """Return to home"""
        # First stop any pattern
        if self.executing_pattern:
            self.stop_pattern = True
        
        # Send RTH command
        cmd = String()
        cmd.data = 'rth'
        self.simple_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AdvancedLLMController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()