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
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from mavros_msgs.msg import State
from cv_bridge import CvBridge

class SARWebPlatform(Node):
    def __init__(self):
        super().__init__('sar_web_platform')
        
        # Initialize Flask app
        # Get the package share directory
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
            'has_detections': False
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
        
        # Setup Flask routes
        self.setup_routes()
        
        self.get_logger().info('SAR Web Platform initialized')
    
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
        self.drone_state['search_active'] = 'search' in msg.data.lower()
    
    def image_callback(self, msg):
        # This receives the debug image with bounding boxes
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            # Higher resolution for better quality
            cv_image = cv2.resize(cv_image, (1280, 720))
            # Higher quality JPEG encoding
            encode_params = [cv2.IMWRITE_JPEG_QUALITY, 85]
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
                encode_params = [cv2.IMWRITE_JPEG_QUALITY, 85]
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
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('index.html')
        
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
            elif command.startswith('move_'):
                direction = command.replace('move_', '')
                if direction == 'stop':
                    # Special case for stop
                    result = self.move_drone('stop')
                elif direction == 'backward':
                    # Fix backward command
                    result = self.move_drone('back')
                else:
                    result = self.move_drone(direction)
            elif command.startswith('rotate_'):
                direction = command.replace('rotate_', '')
                result = self.rotate_drone(direction)
            elif command.startswith('nlp:'):
                nlp_cmd = command.replace('nlp:', '')
                result = self.send_nlp_command(nlp_cmd)
            else:
                result = {'success': False, 'message': 'Unknown command'}
            
            return jsonify(result)
        
        @self.app.route('/api/launch', methods=['POST'])
        def launch_system():
            data = request.json
            component = data.get('component')
            
            if component == 'all':
                return jsonify(self.launch_all_systems())
            elif component == 'stop_all':
                return jsonify(self.stop_all_systems())
            else:
                return jsonify({'success': False, 'message': 'Unknown component'})
        
        @self.app.route('/api/video_feed')
        def video_feed():
            def generate():
                while True:
                    if self.drone_state['latest_image']:
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + 
                               base64.b64decode(self.drone_state['latest_image']) + 
                               b'\r\n')
                    else:
                        # Send blank frame if no image
                        blank = np.zeros((720, 1280, 3), dtype=np.uint8)
                        cv2.putText(blank, "Waiting for camera feed...", 
                                   (440, 360), cv2.FONT_HERSHEY_SIMPLEX, 
                                   1, (100, 100, 100), 2)
                        _, buffer = cv2.imencode('.jpg', blank)
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + 
                               buffer.tobytes() + 
                               b'\r\n')
                    # Small delay for smoother streaming (~30 FPS)
                    cv2.waitKey(33)
            
            return Response(generate(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
    
    def arm_drone(self):
        try:
            subprocess.run([
                'ros2', 'service', 'call', '/mavros/set_mode',
                'mavros_msgs/srv/SetMode', '{"custom_mode": "OFFBOARD"}'
            ])
            subprocess.run([
                'ros2', 'service', 'call', '/mavros/cmd/arming',
                'mavros_msgs/srv/CommandBool', '{"value": true}'
            ])
            return {'success': True, 'message': 'Drone armed'}
        except Exception as e:
            return {'success': False, 'message': str(e)}
    
    def disarm_drone(self):
        try:
            subprocess.run([
                'ros2', 'service', 'call', '/mavros/cmd/arming',
                'mavros_msgs/srv/CommandBool', '{"value": false}'
            ])
            return {'success': True, 'message': 'Drone disarmed'}
        except Exception as e:
            return {'success': False, 'message': str(e)}
    
    def takeoff(self):
        msg = String()
        msg.data = 'up'
        self.simple_cmd_pub.publish(msg)
        
        def stop_climb():
            stop_msg = String()
            stop_msg.data = 'stop'
            self.simple_cmd_pub.publish(stop_msg)
        
        # Stop climbing after 5 seconds
        threading.Timer(5.0, stop_climb).start()
        return {'success': True, 'message': 'Taking off'}
    
    def land(self):
        msg = String()
        msg.data = 'down'
        self.simple_cmd_pub.publish(msg)
        return {'success': True, 'message': 'Landing'}
    
    def start_search(self):
        msg = String()
        msg.data = 'search'
        self.search_cmd_pub.publish(msg)
        self.drone_state['mission_status'] = 'SEARCHING'
        return {'success': True, 'message': 'Search started'}
    
    def stop_search(self):
        msg = String()
        msg.data = 'stop'
        self.stop_cmd_pub.publish(msg)
        self.drone_state['mission_status'] = 'IDLE'
        return {'success': True, 'message': 'Search stopped'}
    
    def return_home(self):
        msg = String()
        msg.data = 'rth'
        self.rth_cmd_pub.publish(msg)
        self.drone_state['mission_status'] = 'RETURNING'
        return {'success': True, 'message': 'Returning to home'}
    
    def emergency_stop(self):
        msg = Bool()
        msg.data = True
        self.emergency_pub.publish(msg)
        self.drone_state['mission_status'] = 'EMERGENCY'
        return {'success': True, 'message': 'EMERGENCY STOP'}
    
    def set_home(self):
        msg = String()
        msg.data = 'set'
        self.set_home_pub.publish(msg)
        self.drone_state['home_position'] = self.drone_state['position'].copy()
        self.drone_state['home_set'] = True
        return {'success': True, 'message': 'Home position set'}
    
    def move_drone(self, direction):
        msg = String()
        msg.data = direction
        self.simple_cmd_pub.publish(msg)
        return {'success': True, 'message': f'Moving {direction}'}
    
    def rotate_drone(self, direction):
        msg = String()
        if direction == 'left':
            msg.data = 'yaw_left'
        elif direction == 'right':
            msg.data = 'yaw_right'
        else:
            return {'success': False, 'message': 'Invalid rotation direction'}
        
        self.simple_cmd_pub.publish(msg)
        return {'success': True, 'message': f'Rotating {direction}'}
    
    def send_nlp_command(self, command):
        msg = String()
        msg.data = command
        self.llm_cmd_pub.publish(msg)
        return {'success': True, 'message': f'NLP command sent: {command}'}
    
    def launch_all_systems(self):
        # This would launch all ROS nodes
        # For now, return a message
        return {'success': True, 'message': 'Use launch script for full system start'}
    
    def stop_all_systems(self):
        # Stop all processes
        for name, proc in self.processes.items():
            proc.terminate()
        self.processes.clear()
        return {'success': True, 'message': 'All systems stopped'}
    
    def run_server(self):
        self.app.run(host='0.0.0.0', port=5000, debug=False)


def main(args=None):
    rclpy.init(args=args)
    
    platform = SARWebPlatform()
    
    # Run Flask in separate thread
    flask_thread = threading.Thread(target=platform.run_server)
    flask_thread.daemon = True
    flask_thread.start()
    
    # Run ROS2 node
    rclpy.spin(platform)
    
    platform.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()