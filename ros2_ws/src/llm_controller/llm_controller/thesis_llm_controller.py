#!/usr/bin/env python3
"""
Thesis LLM Controller - Demonstrates TRUE LLM understanding for MSc Thesis
Parses complex natural language into autonomous SAR behaviors
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from openai import OpenAI
import json
import os
import time

class ThesisLLMController(Node):
    def __init__(self):
        super().__init__('thesis_llm_controller')
        
        # ESSENTIAL: OpenAI API for thesis demonstration
        self.api_key = os.getenv('OPENAI_API_KEY', '')
        if not self.api_key:
            self.get_logger().error('='*60)
            self.get_logger().error('❌ CRITICAL: OPENAI_API_KEY not set!')
            self.get_logger().error('   MSc Thesis REQUIRES LLM for natural language understanding')
            self.get_logger().error('   Set with: export OPENAI_API_KEY="your-key-here"')
            self.get_logger().error('='*60)
            raise ValueError('OpenAI API is ESSENTIAL for LLM-based drone control thesis')
        
        self.client = OpenAI(api_key=self.api_key)
        
        # Publishers
        self.mission_pub = self.create_publisher(String, '/autonomous/mission_command', 10)
        self.simple_pub = self.create_publisher(String, '/simple_command', 10)
        self.position_pub = self.create_publisher(String, '/position_command', 10)  # CRITICAL: For distance-based movements
        self.priority_pub = self.create_publisher(String, '/priority_mode', 10)
        self.status_pub = self.create_publisher(String, '/llm/interpretation', 10)
        
        # Subscriber
        self.command_sub = self.create_subscription(
            String, '/llm/command_input',
            self.process_natural_language, 10
        )
        
        self.get_logger().info('🎓 MSc Thesis LLM Controller initialized')
        self.get_logger().info('   ✅ Using OpenAI GPT for natural language understanding')
        self.get_logger().info('   ✅ Demonstrates autonomous SAR from complex commands')
    
    def process_natural_language(self, msg):
        """Process natural language using OpenAI GPT"""
        command = msg.data
        self.get_logger().info(f'Natural language input: "{command}"')
        
        start_time = time.time()
        
        try:
            # Use GPT to understand complex commands
            response = self.client.chat.completions.create(
                model="gpt-3.5-turbo",
                messages=[
                    {"role": "system", "content": """You are an AI system for an MSc thesis on LLM-based autonomous drones.
                    Parse natural language commands for search and rescue operations.
                    
                    Understand complex mission descriptions like:
                    - "Do a zigzag search and find people"
                    - "Search the area for survivors using a spiral pattern"
                    - "Look for injured people with an expanding square search"
                    - "Patrol the area and inspect any persons found"
                    
                    Extract:
                    1. search_pattern: zigzag, spiral, expanding_square, or grid
                    2. search_size: numeric size in meters (e.g., 20, 50, 100)
                    3. iterations: number of iterations/loops for the pattern (e.g., 3, 4, 5)
                    4. target: what to search for (people, survivors, persons, injured)
                    5. behavior: full (complete SAR sequence) or simple (just search)
                    6. urgency: normal or urgent
                    
                    Also understand simple commands:
                    - Movement: forward, backward, left, right, up, down
                    - Control: stop, hover, land, return
                    
                    Output JSON with:
                    {
                        "type": "mission" or "simple",
                        "action": "forward/backward/left/right/up/down/stop" (REQUIRED for simple commands),
                        "pattern": "zigzag/spiral/expanding_square" (for missions),
                        "size": number (search area size in meters, default 30 if not specified),
                        "iterations": number (pattern iterations, default 3 if not specified),
                        "target": "people" (for missions),
                        "parameters": {"distance": number} (if specified),
                        "interpretation": "human-readable explanation",
                        "confidence": 0.0-1.0
                    }
                    
                    Examples:
                    "move ahead 3m" → {"type":"simple", "action":"forward", "parameters":{"distance":3}, "interpretation":"Move forward 3 meters"}
                    "go up" → {"type":"simple", "action":"up", "interpretation":"Ascend"}
                    "search for people" → {"type":"mission", "pattern":"expanding_square", "size":30, "iterations":3, "target":"people", "interpretation":"SAR mission with 30m area, 3 iterations"}
                    "do a 50 meter expanding square" → {"type":"mission", "pattern":"expanding_square", "size":50, "iterations":3, "target":"people", "interpretation":"50m expanding square search"}
                    "expanding square 10m 4 iterations" → {"type":"mission", "pattern":"expanding_square", "size":10, "iterations":4, "target":"people", "interpretation":"10m expanding square with 4 iterations"}
                    "search 100m area 5 times" → {"type":"mission", "pattern":"expanding_square", "size":100, "iterations":5, "target":"people", "interpretation":"Search 100m area, 5 iterations"}
                    """},
                    {"role": "user", "content": command}
                ],
                temperature=0.3,
                max_tokens=200
            )
            
            # Parse GPT response
            gpt_output = response.choices[0].message.content
            parsed = json.loads(gpt_output)
            
            elapsed = time.time() - start_time
            
            # Log interpretation
            self.get_logger().info('='*50)
            self.get_logger().info(f'🧠 LLM Understanding (took {elapsed:.2f}s):')
            self.get_logger().info(f'   Interpretation: {parsed.get("interpretation")}')
            self.get_logger().info(f'   Type: {parsed.get("type")}')
            if parsed.get("type") == "mission":
                self.get_logger().info(f'   Pattern: {parsed.get("pattern", "N/A")}')
                self.get_logger().info(f'   Size: {parsed.get("size", 30)}m')
                self.get_logger().info(f'   Iterations: {parsed.get("iterations", 3)}')
                self.get_logger().info(f'   Target: {parsed.get("target", "N/A")}')
            self.get_logger().info(f'   Confidence: {parsed.get("confidence", 0)*100:.0f}%')
            self.get_logger().info('='*50)
            
            # Execute based on understanding
            if parsed.get('type') == 'mission':
                # Complex SAR mission
                self.execute_sar_mission(parsed)
            else:
                # Simple command
                self.execute_simple_command(parsed)
            
            # Publish interpretation for monitoring
            self.status_pub.publish(String(data=json.dumps({
                'original': command,
                'interpretation': parsed.get('interpretation'),
                'executed': True,
                'llm_time': elapsed
            })))
            
        except Exception as e:
            self.get_logger().error(f'LLM processing error: {e}')
            self.get_logger().error('Cannot demonstrate LLM capability without API!')
    
    def execute_sar_mission(self, parsed):
        """Execute complex SAR mission"""
        # IMPORTANT: Don't set priority here - let the SAR executor handle it
        # The SAR executor will switch between pattern_focus for search
        # and autonomous for detection/approach behaviors
        
        # Build mission command with size and iteration information
        pattern = parsed.get('pattern', 'expanding_square')
        size = parsed.get('size', 30)  # Get size from GPT parsing
        iterations = parsed.get('iterations', 3)  # Get iterations from GPT parsing
        target = parsed.get('target', 'people')
        
        # Include size and iterations in the mission command string
        mission_cmd = f"{size} meter {pattern} {iterations} iterations search for {target}"
        
        self.get_logger().info(f'🚁 Executing SAR mission: {mission_cmd}')
        self.get_logger().info('   SAR executor will manage priorities:')
        self.get_logger().info('   • pattern_focus during search')
        self.get_logger().info('   • autonomous during detection/approach')
        self.mission_pub.publish(String(data=mission_cmd))
        
        # Log expected behavior
        self.get_logger().info('Expected autonomous behavior:')
        self.get_logger().info('  1. Start search pattern')
        self.get_logger().info('  2. Detect person → pause pattern')
        self.get_logger().info('  3. Descend to 10m')
        self.get_logger().info('  4. Approach to 5m')
        self.get_logger().info('  5. Orbit 2x')
        self.get_logger().info('  6. Return to altitude')
        self.get_logger().info('  7. Resume search')
    
    def execute_simple_command(self, parsed):
        """Execute simple movement command - FIXED to use position commands like V4"""
        action = parsed.get('action')
        
        if not action:
            self.get_logger().error('No action specified in GPT response!')
            self.get_logger().error(f'Parsed data: {parsed}')
            return
        
        # Check if distance was specified
        params = parsed.get('parameters', {})
        distance = params.get('distance')
        
        # Handle different command types like V4 does
        if action in ['yaw_left', 'yaw_right', 'stop']:
            # Rotation and stop commands need manual_priority
            self.priority_pub.publish(String(data='manual_priority'))
            time.sleep(0.1)
            self.simple_pub.publish(String(data=action))
            self.get_logger().info(f'🎮 Executing simple command: {action}')
            
        elif action in ['forward', 'backward', 'left', 'right', 'up', 'down']:
            # Movement commands use position controller with distance
            # CRITICAL: Set llm_only priority for position commands
            self.priority_pub.publish(String(data='llm_only'))
            time.sleep(0.1)
            
            # Use default distance if not specified
            if not distance:
                distance = 5  # Default 5 meters
            
            # FIXED: Use position command format like V4!
            position_cmd = f'{action}:{distance}'
            self.get_logger().info(f'📏 Executing position command: {action} for {distance}m')
            self.get_logger().info(f'   Publishing to /position_command: "{position_cmd}"')
            self.position_pub.publish(String(data=position_cmd))
            
        else:
            # Unknown command, just publish as simple
            self.priority_pub.publish(String(data='manual_priority'))
            time.sleep(0.1)
            self.simple_pub.publish(String(data=action))
            self.get_logger().info(f'Executing fallback command: {action}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ThesisLLMController()
        rclpy.spin(node)
    except ValueError as e:
        print(f"\n{'='*60}")
        print(f"THESIS REQUIREMENT: {e}")
        print(f"{'='*60}\n")
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()