#!/usr/bin/env python3
"""
Practical LLM Controller that works with existing system capabilities
Uses position control for precise movements and integrates with current features
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from openai import OpenAI
import json
import os
import time
import threading

class PracticalLLMController(Node):
    def __init__(self):
        super().__init__('practical_llm_controller')
        
        # Define QoS profile for MAVROS topics
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # OpenAI setup
        self.declare_parameter('openai_api_key', '')
        self.api_key = self.get_parameter('openai_api_key').value or os.getenv('OPENAI_API_KEY')
        
        if not self.api_key:
            self.get_logger().error('No OpenAI API key found!')
        else:
            self.client = OpenAI(api_key=self.api_key)
        
        # Publishers
        self.position_pub = self.create_publisher(String, '/position_command', 10)
        self.simple_pub = self.create_publisher(String, '/simple_command', 10)
        self.search_pub = self.create_publisher(String, '/search_command', 10)
        self.rth_pub = self.create_publisher(String, '/rth_command', 10)
        
        # Subscribers
        self.llm_input_sub = self.create_subscription(
            String, '/llm/command_input', 
            self.process_natural_command, 10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.update_position, 
            qos_profile=self.mavros_qos
        )
        
        self.detection_sub = self.create_subscription(
            String, '/drone/scene_description',
            self.handle_detections, 10
        )
        
        # State
        self.current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.executing_mission = False
        self.mission_steps = []
        self.current_step = 0
        self.hover_on_detection = False
        
        self.get_logger().info('Practical LLM Controller ready')
    
    def update_position(self, msg):
        """Track current position"""
        self.current_position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
    
    def process_natural_command(self, msg):
        """Process natural language with realistic capabilities"""
        command = msg.data
        self.get_logger().info(f'Processing: "{command}"')
        
        # Parse with GPT
        parsed = self.parse_command(command)
        
        # Execute based on parsed intent
        self.execute_command(parsed)
    
    def parse_command(self, command):
        """Use GPT to understand command intent"""
        
        system_prompt = """You are a drone command parser. Parse natural language into structured commands.
        
The drone can ONLY do these things:
1. Move in directions with specific distances: forward/backward/left/right/up/down X meters
2. Go to absolute positions: goto X,Y,Z
3. Execute pre-programmed grid search pattern
4. Return to home (RTH)
5. Stop/hover
6. Rotate/yaw left or right

The drone CANNOT do custom patterns like spirals or expanding squares.

Extract and return as JSON:
{
    "intent": "move/goto/search/rth/stop/rotate/unsupported",
    "params": {
        "direction": "forward/backward/left/right/up/down",
        "distance": float (meters),
        "position": {"x": float, "y": float, "z": float},
        "hover_on_detection": boolean
    },
    "original_request": "user's original text",
    "explanation": "brief explanation of what will be done"
}

If user asks for something not supported (like spiral search), set intent as "unsupported" and explain in the explanation field.
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
            
            result = json.loads(response.choices[0].message.content)
            self.get_logger().info(f"Parsed: {json.dumps(result, indent=2)}")
            return result
            
        except Exception as e:
            self.get_logger().error(f"LLM parsing error: {e}")
            # Fallback parsing
            return self.simple_parse(command)
    
    def simple_parse(self, command):
        """Simple fallback parser"""
        cmd_lower = command.lower()
        
        # Basic movement detection
        if 'forward' in cmd_lower and 'meter' in cmd_lower:
            import re
            match = re.search(r'(\d+\.?\d*)\s*meter', cmd_lower)
            if match:
                return {
                    'intent': 'move',
                    'params': {
                        'direction': 'forward',
                        'distance': float(match.group(1))
                    }
                }
        
        # Search detection
        if 'search' in cmd_lower or 'find' in cmd_lower:
            return {
                'intent': 'search',
                'params': {'hover_on_detection': True}
            }
        
        # RTH detection
        if 'return' in cmd_lower or 'home' in cmd_lower:
            return {'intent': 'rth', 'params': {}}
        
        # Stop/hover
        if 'stop' in cmd_lower or 'hover' in cmd_lower:
            return {'intent': 'stop', 'params': {}}
        
        return {'intent': 'unsupported', 'params': {}}
    
    def execute_command(self, parsed):
        """Execute the parsed command"""
        intent = parsed.get('intent', 'unsupported')
        params = parsed.get('params', {})
        explanation = parsed.get('explanation', '')
        
        if explanation:
            self.get_logger().info(f"Execution plan: {explanation}")
        
        if intent == 'move':
            # Precise movement with distance
            direction = params.get('direction', 'forward')
            distance = params.get('distance', 1.0)
            
            cmd = f"{direction}:{distance}"
            self.position_pub.publish(String(data=cmd))
            self.get_logger().info(f"Moving {direction} {distance} meters")
            
        elif intent == 'goto':
            # Go to absolute position
            pos = params.get('position', {})
            if all(k in pos for k in ['x', 'y', 'z']):
                cmd = f"goto:{pos['x']},{pos['y']},{pos['z']}"
                self.position_pub.publish(String(data=cmd))
                self.get_logger().info(f"Going to position ({pos['x']}, {pos['y']}, {pos['z']})")
            
        elif intent == 'search':
            # Trigger grid search
            self.hover_on_detection = params.get('hover_on_detection', True)
            self.search_pub.publish(String(data='search'))
            self.get_logger().info("Starting grid search pattern")
            
        elif intent == 'rth':
            # Return to home
            self.rth_pub.publish(String(data='rth'))
            self.get_logger().info("Returning to home")
            
        elif intent == 'stop':
            # Stop/hover
            self.simple_pub.publish(String(data='stop'))
            self.get_logger().info("Stopping/hovering")
            
        elif intent == 'rotate':
            # Rotation
            direction = params.get('direction', 'left')
            if direction == 'left':
                self.simple_pub.publish(String(data='yaw_left'))
            else:
                self.simple_pub.publish(String(data='yaw_right'))
            
            # Stop rotation after 3 seconds (about 90 degrees)
            threading.Timer(3.0, lambda: self.simple_pub.publish(String(data='stop'))).start()
            
        elif intent == 'unsupported':
            self.get_logger().warn(
                f"Unsupported command. Available features:\n"
                f"- Move with distance (e.g., 'go forward 5 meters')\n"
                f"- Grid search pattern\n"
                f"- Return to home\n"
                f"- Stop/hover\n"
                f"Explanation: {explanation}"
            )
    
    def handle_detections(self, msg):
        """Handle vision detections during search"""
        if not self.hover_on_detection:
            return
        
        try:
            data = json.loads(msg.data)
            detections = data.get('detections', [])
            
            # Check for person detections
            people = [d for d in detections if d.get('label') == 'person']
            
            if people:
                self.get_logger().warn(f"DETECTION: Found {len(people)} person(s)!")
                
                # Stop and hover
                self.simple_pub.publish(String(data='stop'))
                
                # Log detection details
                for i, person in enumerate(people):
                    confidence = person.get('confidence', 0.0)
                    bbox = person.get('bbox', {})
                    self.get_logger().info(
                        f"Person {i+1}: Confidence {confidence:.2f}, "
                        f"Position in image: {bbox}"
                    )
                
        except Exception as e:
            self.get_logger().error(f"Detection processing error: {e}")
    
    def execute_multi_step_mission(self, steps):
        """Execute a series of commands (for future enhancement)"""
        self.mission_steps = steps
        self.current_step = 0
        self.executing_mission = True
        
        # This would need position feedback to know when each step is complete
        # For now, just log the plan
        self.get_logger().info(f"Mission plan with {len(steps)} steps:")
        for i, step in enumerate(steps):
            self.get_logger().info(f"  Step {i+1}: {step}")


def main(args=None):
    rclpy.init(args=args)
    node = PracticalLLMController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()