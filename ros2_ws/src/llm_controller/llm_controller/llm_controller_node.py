#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from drone_interfaces.msg import LLMCommand, SceneDescription
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String
import openai
import os
import json

class LLMControllerNode(Node):
    def __init__(self):
        super().__init__('llm_controller_node')
        
        # Parameters
        self.declare_parameter('use_openai', True)
        self.declare_parameter('openai_model', 'gpt-4')
        self.declare_parameter('openai_api_key', '')
        
        self.use_openai = self.get_parameter('use_openai').value
        self.model = self.get_parameter('openai_model').value
        self.api_key = self.get_parameter('openai_api_key').value
        
        # Initialize OpenAI
        if self.use_openai:
            if not self.api_key:
                self.api_key = os.getenv('OPENAI_API_KEY', '')
            if self.api_key:
                openai.api_key = self.api_key
                self.get_logger().info(f'OpenAI initialized with model: {self.model}')
            else:
                self.get_logger().error('No OpenAI API key provided!')
                
        # Subscribers
        self.command_sub = self.create_subscription(
            String,
            '/llm/command_input',
            self.command_callback,
            10
        )
        
        self.scene_sub = self.create_subscription(
            SceneDescription,
            '/drone/scene_description',
            self.scene_callback,
            10
        )
        
        # Publishers
        self.velocity_pub = self.create_publisher(
            Twist,
            '/llm/velocity_command',
            10
        )
        
        self.llm_response_pub = self.create_publisher(
            String,
            '/llm/response',
            10
        )
        
        # State
        self.current_scene = None
        self.command_history = []
        
        self.get_logger().info('LLM Controller initialized')
        
    def scene_callback(self, msg):
        """Store latest scene information"""
        self.current_scene = msg
        
    def command_callback(self, msg):
        """Process natural language command"""
        command = msg.data
        self.get_logger().info(f'Received command: "{command}"')
        
        # Add to history
        self.command_history.append(command)
        
        # Process with LLM
        if self.use_openai and self.api_key:
            self.process_with_openai(command)
        else:
            self.process_with_rules(command)
            
    def process_with_openai(self, command):
        """Process command using OpenAI"""
        try:
            # Build context
            context = self.build_context()
            
            # Create prompt
            prompt = f"""You are a drone flight controller. Convert natural language commands to drone movements.
            
Current scene: {context}
Command: {command}

Respond with a JSON object containing:
- action: "move", "hover", "land", "search", or "stop"
- velocity: object with x, y, z values (-1 to 1)
- duration: seconds to execute (0-10)
- response: brief confirmation message

Example: {{"action": "move", "velocity": {{"x": 0.5, "y": 0, "z": 0}}, "duration": 2, "response": "Moving forward"}}
"""
            
            # Call OpenAI
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": "You are a helpful drone controller that converts natural language to precise drone commands."},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.3
            )
            
            # Parse response
            result = json.loads(response.choices[0].message.content)
            self.execute_command(result)
            
        except Exception as e:
            self.get_logger().error(f'OpenAI error: {e}')
            self.process_with_rules(command)
            
    def process_with_rules(self, command):
        """Fallback rule-based processing"""
        cmd = command.lower()
        result = {"action": "hover", "velocity": {"x": 0, "y": 0, "z": 0}, "duration": 1}
        
        # Simple keyword matching
        if "forward" in cmd or "go straight" in cmd:
            result = {"action": "move", "velocity": {"x": 0.5, "y": 0, "z": 0}, "duration": 2, "response": "Moving forward"}
        elif "back" in cmd or "reverse" in cmd:
            result = {"action": "move", "velocity": {"x": -0.5, "y": 0, "z": 0}, "duration": 2, "response": "Moving backward"}
        elif "left" in cmd:
            result = {"action": "move", "velocity": {"x": 0, "y": 0.5, "z": 0}, "duration": 2, "response": "Moving left"}
        elif "right" in cmd:
            result = {"action": "move", "velocity": {"x": 0, "y": -0.5, "z": 0}, "duration": 2, "response": "Moving right"}
        elif "up" in cmd or "ascend" in cmd:
            result = {"action": "move", "velocity": {"x": 0, "y": 0, "z": 0.5}, "duration": 2, "response": "Ascending"}
        elif "down" in cmd or "descend" in cmd:
            result = {"action": "move", "velocity": {"x": 0, "y": 0, "z": -0.5}, "duration": 2, "response": "Descending"}
        elif "stop" in cmd or "hover" in cmd:
            result = {"action": "hover", "velocity": {"x": 0, "y": 0, "z": 0}, "duration": 1, "response": "Hovering"}
        elif "land" in cmd:
            result = {"action": "land", "velocity": {"x": 0, "y": 0, "z": -0.3}, "duration": 5, "response": "Landing"}
        elif "search" in cmd or "look for" in cmd:
            result = {"action": "search", "velocity": {"x": 0, "y": 0, "z": 0}, "duration": 5, "response": "Searching area"}
        else:
            result["response"] = "Command not understood, hovering"
            
        self.execute_command(result)
        
    def execute_command(self, command_dict):
        """Execute the parsed command"""
        # Log response
        response = command_dict.get('response', 'Executing command')
        self.get_logger().info(response)
        
        # Publish response
        resp_msg = String()
        resp_msg.data = response
        self.llm_response_pub.publish(resp_msg)
        
        # Create velocity command with coordinate transformation
        vel_msg = Twist()
        vel = command_dict.get('velocity', {})
        
        # Transform from intuitive commands to UE coordinates
        # Command "forward" → UE needs negative Y
        # Command "right" → UE needs positive X  
        # Command "up" → UE Z is correct
        vel_msg.linear.x = float(vel.get('y', 0)) * -1  # Right/left (inverted)
        vel_msg.linear.y = float(vel.get('x', 0))       # Forward/back
        vel_msg.linear.z = float(vel.get('z', 0))       # Up/down (same)
        
        # Publish velocity
        self.velocity_pub.publish(vel_msg)
        # TODO: Implement duration-based execution
        
    def build_context(self):
        """Build context string from current scene"""
        if not self.current_scene:
            return "No scene information available"
            
        context = f"Scene: {self.current_scene.general_description}"
        if self.current_scene.detected_objects:
            context += f" Objects: {len(self.current_scene.detected_objects)} detected."
        if self.current_scene.safety_assessment:
            context += f" Safety: {self.current_scene.safety_assessment}"
            
        return context

def main(args=None):
    rclpy.init(args=args)
    node = LLMControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LLM controller...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
