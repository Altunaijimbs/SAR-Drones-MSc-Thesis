#!/usr/bin/env python3
"""
Enhanced LLM Controller for Natural Language SAR Commands
Example: "Search for 3 missing people last seen at position (10, 20, 5), 
         do a grid search and hover when found"
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import String, Bool
from openai import OpenAI
import json
import re
import math
import os

class EnhancedLLMController(Node):
    def __init__(self):
        super().__init__('enhanced_llm_controller')
        
        # OpenAI setup
        self.declare_parameter('openai_api_key', '')
        self.api_key = self.get_parameter('openai_api_key').value or os.getenv('OPENAI_API_KEY')
        self.client = OpenAI(api_key=self.api_key)
        
        # Publishers
        self.search_pub = self.create_publisher(String, '/search_command', 10)
        self.simple_pub = self.create_publisher(String, '/simple_command', 10)
        
        # Subscribers
        self.llm_input_sub = self.create_subscription(
            String, '/llm/command_input', 
            self.process_natural_command, 10
        )
        
        self.detection_sub = self.create_subscription(
            String, '/drone/scene_description',
            self.handle_detections, 10
        )
        
        # State
        self.target_count = 0
        self.found_count = 0
        self.search_active = False
        self.hovering = False
        
        self.get_logger().info('Enhanced LLM Controller ready')
    
    def process_natural_command(self, msg):
        """Process natural language commands"""
        command = msg.data
        self.get_logger().info(f'Processing: "{command}"')
        
        # Use LLM to parse intent and parameters
        parsed = self.parse_with_llm(command)
        
        if parsed['action'] == 'search':
            self.execute_search(parsed)
        elif parsed['action'] == 'hover':
            self.execute_hover()
        elif parsed['action'] == 'return':
            self.execute_return()
    
    def parse_with_llm(self, command):
        """Use GPT to parse natural language into structured commands"""
        
        system_prompt = """
        You are a drone command parser. Extract the following from natural language:
        - action: (search/hover/return/move)
        - target_count: number of people to find
        - position: (x, y, z) coordinates if mentioned
        - search_radius: radius for grid search (default 20m)
        
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
            
            return json.loads(response.choices[0].message.content)
            
        except Exception as e:
            # Fallback to regex parsing
            return self.parse_with_regex(command)
    
    def parse_with_regex(self, command):
        """Fallback regex parser"""
        parsed = {
            'action': 'search',
            'target_count': 1,
            'position': None,
            'search_radius': 20
        }
        
        # Extract numbers for people count
        people_match = re.search(r'(\d+)\s*(?:missing\s*)?(?:people|person)', command)
        if people_match:
            parsed['target_count'] = int(people_match.group(1))
        
        # Extract coordinates
        coord_match = re.search(r'\(?\s*(\d+\.?\d*)\s*,\s*(\d+\.?\d*)\s*,?\s*(\d+\.?\d*)?\s*\)?', command)
        if coord_match:
            x, y, z = coord_match.groups()
            parsed['position'] = {
                'x': float(x),
                'y': float(y),
                'z': float(z) if z else 5.0  # Default altitude
            }
        
        # Determine action
        if 'search' in command.lower() or 'find' in command.lower():
            parsed['action'] = 'search'
        elif 'hover' in command.lower() or 'stop' in command.lower():
            parsed['action'] = 'hover'
        elif 'return' in command.lower() or 'home' in command.lower():
            parsed['action'] = 'return'
        
        return parsed
    
    def execute_search(self, params):
        """Execute grid search based on parsed parameters"""
        self.target_count = params['target_count']
        self.found_count = 0
        self.search_active = True
        
        # If position specified, move there first
        if params['position']:
            # This would integrate with your grid search node
            # For now, just start search
            self.get_logger().info(f"Starting search for {self.target_count} people at {params['position']}")
        
        # Trigger grid search
        search_cmd = String()
        search_cmd.data = 'search'
        self.search_pub.publish(search_cmd)
    
    def handle_detections(self, msg):
        """Monitor detections and hover when person found"""
        if not self.search_active:
            return
            
        try:
            data = json.loads(msg.data)
            detections = data.get('detections', [])
            
            # Check for person detections
            people = [d for d in detections if d.get('label') == 'person']
            
            if people and not self.hovering:
                self.found_count += len(people)
                self.get_logger().warn(f"FOUND {len(people)} person(s)! Total: {self.found_count}/{self.target_count}")
                
                # Stop and hover
                stop_cmd = String()
                stop_cmd.data = 'stop'
                self.simple_pub.publish(stop_cmd)
                self.hovering = True
                
                # After 5 seconds, continue search if more to find
                if self.found_count < self.target_count:
                    self.create_timer(5.0, self.resume_search)
                else:
                    self.get_logger().info("All targets found! Mission complete.")
                    self.search_active = False
                    
        except Exception as e:
            self.get_logger().error(f"Detection processing error: {e}")
    
    def resume_search(self):
        """Resume search after hovering"""
        self.hovering = False
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
        cmd = String()
        cmd.data = 'rth'
        self.simple_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = EnhancedLLMController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()