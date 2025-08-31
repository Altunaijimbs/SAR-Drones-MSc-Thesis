#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from flask import Flask, render_template, jsonify, request, Response
from flask_cors import CORS
import threading
import json
import cv2
import base64
import numpy as np
from datetime import datetime
import subprocess
import os
import signal

# ROS2 message imports
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import Image
from mavros_msgs.msg import State
from cv_bridge import CvBridge

class EnhancedSARWebPlatform(Node):
    def __init__(self):
        super().__init__('enhanced_sar_web_platform')
        
        # Initialize Flask app
        import ament_index_python
        try:
            package_share = ament_index_python.get_package_share_directory('sar_web_platform')
            template_folder = os.path.join(package_share, 'templates')
            static_folder = os.path.join(package_share, 'static')
        except:
            # Fallback to relative paths during development
            template_folder = os.path.join(os.path.dirname(__file__), '../templates')
            static_folder = os.path.join(os.path.dirname(__file__), '../static')
        
        self.app = Flask(__name__, 
                        template_folder=template_folder,
                        static_folder=static_folder)
        CORS(self.app)
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # System state
        self.drone_state = {
            'armed': False,
            'mode': 'UNKNOWN',
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'home_position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'home_set': False,
            'battery': 100,  # Simulated for now
            'gps_fix': True,  # Simulated
            'search_active': False,
            'mission_status': 'IDLE',
            'velocity_source': 'none',
            'latest_image': None,
            'detections': [],
            'has_detections': False,
            # NEW: Priority control state
            'priority_mode': 'manual_priority',  # Default to manual_priority for V3
            'pattern_paused': False,
            'pattern_status': 'idle',
            'current_controller': 'none'
        }
        
        # Priority mode definitions - MATCHES improved_velocity_coordinator
        self.priority_modes = {
            'pure_manual': {
                'description': 'ONLY manual commands work',
                'manual': 100,
                'pattern': 0,
                'llm': 0,
                'rth': 0
            },
            'manual_priority': {
                'description': 'Manual has priority (DEFAULT)',
                'manual': 80,
                'pattern': 10,
                'llm': 30,
                'rth': 90
            },
            'llm_only': {
                'description': 'Only LLM commands',
                'manual': 0,
                'pattern': 0,
                'llm': 100,
                'rth': 80
            },
            'hybrid_balanced': {
                'description': 'Everything balanced',
                'manual': 50,
                'pattern': 40,
                'llm': 50,
                'rth': 80
            },
            'pattern_focus': {
                'description': 'Patterns have priority',
                'manual': 20,
                'pattern': 80,
                'llm': 20,
                'rth': 90
            },
            'autonomous': {
                'description': 'Fully autonomous',
                'manual': 10,
                'pattern': 70,
                'llm': 70,
                'rth': 90
            },
            'emergency': {
                'description': 'Safety only',
                'manual': 50,
                'pattern': 0,
                'llm': 0,
                'rth': 100
            }
        }
        
        # Process management
        self.processes = {}
        
        # QoS profile for MAVROS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.search_cmd_pub = self.create_publisher(String, '/search_command', 10)
        self.stop_cmd_pub = self.create_publisher(String, '/stop_command', 10)
        self.simple_cmd_pub = self.create_publisher(String, '/simple_command', 10)
        self.rth_cmd_pub = self.create_publisher(String, '/rth_command', 10)
        self.llm_cmd_pub = self.create_publisher(String, '/llm/command_input', 10)
        self.emergency_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        self.set_home_pub = self.create_publisher(String, '/set_home_position', 10)
        
        # NEW: Priority control publishers
        self.priority_mode_pub = self.create_publisher(String, '/priority_mode', 10)
        self.pattern_control_pub = self.create_publisher(String, '/pattern_control', 10)
        self.pattern_cmd_pub = self.create_publisher(String, '/pattern_command', 10)
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', 
            self.state_callback, qos_profile
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.pose_callback, qos_profile
        )
        
        self.vel_source_sub = self.create_subscription(
            String, '/velocity_coordinator/active_source',
            self.vel_source_callback, 10
        )
        
        # NEW: Pattern status subscriber
        self.pattern_status_sub = self.create_subscription(
            String, '/pattern_status',
            self.pattern_status_callback, 10
        )
        
        # Subscribe to debug image with bounding boxes if available
        self.camera_sub = self.create_subscription(
            Image, '/drone/vision/debug_image',
            self.image_callback, 10
        )
        
        # Fallback to raw camera feed
        self.raw_camera_sub = self.create_subscription(
            Image, '/airsim_node/PX4/front_center_Scene/image',
            self.raw_image_callback, 10
        )
        
        self.detection_sub = self.create_subscription(
            String, '/drone/scene_description',
            self.detection_callback, 10
        )
        
        # Pattern waypoints subscriber for map visualization
        self.pattern_waypoints_sub = self.create_subscription(
            String, '/pattern_waypoints',
            self.pattern_waypoints_callback, 10
        )
        
        # Setup Flask routes
        self.setup_routes()
        
        # Initialize priority mode
        self.set_priority_mode('manual_priority')
        
        self.get_logger().info('Enhanced SAR Web Platform with Priority Control initialized')
    
    def state_callback(self, msg):
        self.drone_state['armed'] = msg.armed
        self.drone_state['mode'] = msg.mode
    
    def pose_callback(self, msg):
        self.drone_state['position'] = {
            'x': round(msg.pose.position.x, 2),
            'y': round(msg.pose.position.y, 2),
            'z': round(msg.pose.position.z, 2)
        }
    
    def vel_source_callback(self, msg):
        self.drone_state['velocity_source'] = msg.data.replace('Active: ', '')
        self.drone_state['current_controller'] = msg.data.replace('Active: ', '')
        self.drone_state['search_active'] = 'search' in msg.data.lower()
    
    def pattern_status_callback(self, msg):
        self.drone_state['pattern_status'] = msg.data
        # Try to extract waypoint progress from status
        if 'waypoint' in msg.data.lower():
            try:
                # Extract waypoint number from status like "Executing waypoint 3/10"
                import re
                match = re.search(r'(\d+)/(\d+)', msg.data)
                if match:
                    current = int(match.group(1))
                    total = int(match.group(2))
                    self.drone_state['pattern_progress'] = {'current': current, 'total': total}
            except:
                pass
    
    def pattern_waypoints_callback(self, msg):
        """Handle pattern waypoints for map visualization"""
        try:
            data = json.loads(msg.data)
            waypoints = data.get('waypoints', [])
            # Convert waypoints to list of dicts for JavaScript
            self.drone_state['pattern_waypoints'] = [
                {'x': wp[0], 'y': wp[1], 'z': wp[2] if len(wp) > 2 else 10}
                for wp in waypoints
            ]
            self.drone_state['pattern_type'] = data.get('type', 'Custom')
            # Initialize progress tracking
            if 'pattern_progress' not in self.drone_state:
                self.drone_state['pattern_progress'] = {'current': 0, 'total': len(waypoints)}
            self.drone_state['pattern_progress']['total'] = len(waypoints)
        except Exception as e:
            self.get_logger().debug(f'Pattern waypoints parsing: {e}')
    
    def image_callback(self, msg):
        # This receives the debug image with bounding boxes
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # Higher resolution for better quality
            cv_image = cv2.resize(cv_image, (1280, 720))
            # Higher quality JPEG encoding
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, 65]
            _, buffer = cv2.imencode('.jpg', cv_image, encode_params)
            self.drone_state['latest_image'] = base64.b64encode(buffer).decode('utf-8')
            self.drone_state['has_detections'] = True
        except Exception as e:
            self.get_logger().error(f'Debug image conversion error: {e}')
    
    def raw_image_callback(self, msg):
        # Fallback: only use raw image if no debug image available
        if not self.drone_state.get('has_detections', False):
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                # Higher resolution for better quality
                cv_image = cv2.resize(cv_image, (1280, 720))
                # Higher quality JPEG encoding
                encode_params = [cv2.IMWRITE_JPEG_QUALITY, 65]
                _, buffer = cv2.imencode('.jpg', cv_image, encode_params)
                self.drone_state['latest_image'] = base64.b64encode(buffer).decode('utf-8')
            except Exception as e:
                self.get_logger().error(f'Raw image conversion error: {e}')
    
    def detection_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.drone_state['detections'] = data.get('detections', [])
        except:
            pass
    
    def set_priority_mode(self, mode):
        """Set the priority mode and notify velocity coordinator"""
        if mode in self.priority_modes:
            self.drone_state['priority_mode'] = mode
            # Send just the mode name to velocity coordinator
            msg = String()
            msg.data = mode  # improved_velocity_coordinator expects just the mode name
            self.priority_mode_pub.publish(msg)
            self.get_logger().info(f'Priority mode set to: {mode} - {self.priority_modes[mode].get("description", "")}')
    
    def toggle_pattern_pause(self):
        """Toggle pattern pause state"""
        self.drone_state['pattern_paused'] = not self.drone_state['pattern_paused']
        
        if self.drone_state['pattern_paused']:
            # Pause pattern
            msg = String()
            msg.data = 'pause'
            self.pattern_control_pub.publish(msg)
            
            # Temporarily set pattern priority to 0
            temp_mode = String()
            temp_mode.data = json.dumps({
                'temporary': True,
                'pattern_priority': 0
            })
            self.priority_mode_pub.publish(temp_mode)
        else:
            # Resume pattern
            msg = String()
            msg.data = 'resume'
            self.pattern_control_pub.publish(msg)
            
            # Restore normal priority mode
            self.set_priority_mode(self.drone_state['priority_mode'])
        
        return self.drone_state['pattern_paused']
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('enhanced_index.html')
        
        @self.app.route('/api/status')
        def get_status():
            return jsonify(self.drone_state)
        
        @self.app.route('/api/command', methods=['POST'])
        def send_command():
            data = request.json
            command = data.get('command')
            
            if command == 'arm':
                result = self.arm_drone()
            elif command == 'disarm':
                result = self.disarm_drone()
            elif command == 'takeoff':
                result = self.takeoff()
            elif command == 'land':
                result = self.land()
            elif command == 'start_search':
                result = self.start_search()
            elif command == 'stop_search':
                result = self.stop_search()
            elif command == 'return_home':
                result = self.return_home()
            elif command == 'emergency_stop':
                result = self.emergency_stop()
            elif command == 'set_home':
                result = self.set_home()
            elif command in ['forward', 'backward', 'left', 'right', 'up', 'down', 'stop']:
                result = self.send_simple_command(command)
            elif command in ['yaw_left', 'yaw_right']:
                result = self.send_simple_command(command)
            else:
                result = {'success': False, 'message': 'Unknown command'}
            
            return jsonify(result)
        
        @self.app.route('/api/priority', methods=['POST'])
        def set_priority():
            """Set priority mode"""
            data = request.json
            mode = data.get('mode', 'balanced')
            self.set_priority_mode(mode)
            return jsonify({'success': True, 'mode': mode})
        
        @self.app.route('/api/pattern/pause', methods=['POST'])
        def pause_pattern():
            """Toggle pattern pause"""
            paused = self.toggle_pattern_pause()
            return jsonify({'success': True, 'paused': paused})
        
        @self.app.route('/api/pattern/command', methods=['POST'])
        def send_pattern():
            """Send pattern command"""
            data = request.json
            pattern_type = data.get('type', 'expanding_square')
            params = data.get('params', {})
            
            # Create pattern command string
            if pattern_type == 'expanding_square':
                size = params.get('size', 10)
                iterations = params.get('iterations', 3)
                cmd = f'expanding_square:{size},{iterations}'
            elif pattern_type == 'zigzag':
                width = params.get('width', 20)
                length = params.get('length', 20)
                spacing = params.get('spacing', 5)
                cmd = f'zigzag:{width},{length},{spacing}'
            else:
                cmd = pattern_type
            
            # Send pattern command
            msg = String()
            msg.data = cmd
            self.pattern_cmd_pub.publish(msg)
            
            # Start pattern after short delay
            threading.Timer(0.5, lambda: self.pattern_control_pub.publish(String(data='start'))).start()
            
            return jsonify({'success': True, 'pattern': pattern_type})
        
        @self.app.route('/api/llm', methods=['POST'])
        def send_llm_command():
            data = request.json
            command = data.get('command')
            msg = String()
            msg.data = command
            self.llm_cmd_pub.publish(msg)
            return jsonify({'success': True})
        
        @self.app.route('/api/video_feed')
        def video_feed():
            def generate():
                while True:
                    if self.drone_state['latest_image']:
                        frame_data = self.drone_state['latest_image']
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + 
                               base64.b64decode(frame_data) + b'\r\n')
                    else:
                        # Send a placeholder if no image
                        import time
                        time.sleep(0.1)
            
            return Response(generate(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
    
    def arm_drone(self):
        """Arm the drone using MAVROS services"""
        try:
            # First set OFFBOARD mode
            subprocess.run([
                'ros2', 'service', 'call', '/mavros/set_mode',
                'mavros_msgs/srv/SetMode', '{"custom_mode": "OFFBOARD"}'
            ])
            # Then arm the drone
            subprocess.run([
                'ros2', 'service', 'call', '/mavros/cmd/arming',
                'mavros_msgs/srv/CommandBool', '{"value": true}'
            ])
            self.get_logger().info('Drone armed successfully')
            return {'success': True, 'message': 'Drone armed'}
        except Exception as e:
            self.get_logger().error(f'Failed to arm drone: {e}')
            return {'success': False, 'message': str(e)}
    
    def disarm_drone(self):
        """Disarm the drone using MAVROS service"""
        try:
            subprocess.run([
                'ros2', 'service', 'call', '/mavros/cmd/arming',
                'mavros_msgs/srv/CommandBool', '{"value": false}'
            ])
            self.get_logger().info('Drone disarmed successfully')
            return {'success': True, 'message': 'Drone disarmed'}
        except Exception as e:
            self.get_logger().error(f'Failed to disarm drone: {e}')
            return {'success': False, 'message': str(e)}
    
    def takeoff(self):
        """Proper takeoff using MAVROS service"""
        try:
            # Use MAVROS takeoff service (altitude in meters)
            result = subprocess.run([
                'ros2', 'service', 'call', '/mavros/cmd/takeoff',
                'mavros_msgs/srv/CommandTOL',
                '{"altitude": 5.0, "latitude": 0, "longitude": 0, "min_pitch": 0, "yaw": 0}'
            ], capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                self.get_logger().info('Takeoff command sent (5m altitude)')
                return {'success': True, 'message': 'Takeoff to 5m initiated'}
            else:
                # Fallback to velocity command if service fails
                msg = String()
                msg.data = 'up'
                self.simple_cmd_pub.publish(msg)
                return {'success': True, 'message': 'Takeoff initiated (velocity mode)'}
        except Exception as e:
            self.get_logger().error(f'Takeoff failed: {e}')
            return {'success': False, 'message': str(e)}
    
    def land(self):
        """Proper landing using MAVROS service"""
        try:
            # Use MAVROS land service
            result = subprocess.run([
                'ros2', 'service', 'call', '/mavros/cmd/land',
                'mavros_msgs/srv/CommandTOL',
                '{"altitude": 0, "latitude": 0, "longitude": 0, "min_pitch": 0, "yaw": 0}'
            ], capture_output=True, text=True, timeout=5)
            
            if result.returncode == 0:
                self.get_logger().info('Land command sent')
                return {'success': True, 'message': 'Landing initiated'}
            else:
                # Fallback to velocity command if service fails
                msg = String()
                msg.data = 'down'
                self.simple_cmd_pub.publish(msg)
                return {'success': True, 'message': 'Landing initiated (velocity mode)'}
        except Exception as e:
            self.get_logger().error(f'Landing failed: {e}')
            return {'success': False, 'message': str(e)}
    
    def start_search(self):
        msg = String()
        msg.data = 'search'
        self.search_cmd_pub.publish(msg)
        return {'success': True, 'message': 'Search pattern started'}
    
    def stop_search(self):
        msg = String()
        msg.data = 'stop'
        self.stop_cmd_pub.publish(msg)
        return {'success': True, 'message': 'Search stopped'}
    
    def return_home(self):
        msg = String()
        msg.data = 'rth'
        self.rth_cmd_pub.publish(msg)
        return {'success': True, 'message': 'Returning to home'}
    
    def emergency_stop(self):
        msg = Bool()
        msg.data = True
        self.emergency_pub.publish(msg)
        return {'success': True, 'message': 'Emergency stop activated'}
    
    def set_home(self):
        msg = String()
        msg.data = 'set'
        self.set_home_pub.publish(msg)
        return {'success': True, 'message': 'Home position set'}
    
    def send_simple_command(self, command):
        msg = String()
        msg.data = command
        self.simple_cmd_pub.publish(msg)
        return {'success': True, 'message': f'{command} command sent'}

def main(args=None):
    rclpy.init(args=args)
    
    platform = EnhancedSARWebPlatform()
    
    # Start Flask in a separate thread
    flask_thread = threading.Thread(
        target=lambda: platform.app.run(host='0.0.0.0', port=5000, debug=False)
    )
    flask_thread.daemon = True
    flask_thread.start()
    
    try:
        rclpy.spin(platform)
    except KeyboardInterrupt:
        platform.get_logger().info('Shutting down web platform')
    finally:
        platform.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()