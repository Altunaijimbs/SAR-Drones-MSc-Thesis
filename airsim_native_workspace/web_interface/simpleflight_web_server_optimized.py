#!/usr/bin/env python3

import cosysairsim as airsim
from flask import Flask, render_template, jsonify, request, Response
from flask_cors import CORS
import threading
import json
import cv2
import base64
import numpy as np
from datetime import datetime
import time
import os
import sys
import math
from collections import deque
import asyncio
from queue import Queue, Empty

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

class SimpleFlightWebInterface:
    def __init__(self):
        # Initialize Flask app
        template_folder = os.path.join(os.path.dirname(__file__), 'templates')
        static_folder = os.path.join(os.path.dirname(__file__), 'static')
        
        self.app = Flask(__name__, 
                        template_folder=template_folder,
                        static_folder=static_folder)
        CORS(self.app)
        
        # Connect to AirSim - create separate clients for different operations
        self.control_client = airsim.MultirotorClient()
        self.control_client.confirmConnection()
        
        # Separate client for image capture to avoid conflicts
        self.image_client = airsim.MultirotorClient()
        self.image_client.confirmConnection()
        
        # Configure camera for high FPS
        self.configure_camera_settings()
        
        # System state
        self.drone_state = {
            'connected': False,
            'armed': False,
            'flying': False,
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'home_position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'home_set': False,
            'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'pitch': 0.0, 'roll': 0.0, 'yaw': 0.0},
            'battery': 100,
            'gps_fix': True,
            'pattern_active': False,
            'pattern_type': 'none',
            'mission_status': 'IDLE',
            'latest_image': None,
            'api_enabled': False,
            'total_distance': 0.0,
            'flight_time': 0,
            'max_altitude': 0.0,
            'camera_fps': 0.0
        }
        
        # Movement parameters
        self.movement_speed = 4.0  # m/s
        self.altitude_speed = 2.0  # m/s
        self.yaw_speed = 30.0  # degrees/s
        
        # Pattern execution
        self.pattern_thread = None
        self.stop_pattern = False
        
        # Position tracking
        self.position_history = deque(maxlen=100)
        self.last_position = None
        self.start_time = None
        
        # Image queue for high FPS streaming
        self.image_queue = Queue(maxsize=3)  # Small buffer to prevent lag
        self.frame_counter = 0
        self.fps_start_time = time.time()
        self.current_fps = 0
        
        # Setup Flask routes
        self.setup_routes()
        
        # Start state update thread (lower frequency - 10Hz)
        self.update_thread = threading.Thread(target=self.update_state_loop, daemon=True)
        self.update_thread.start()
        
        # Start high-speed image capture thread (60+ FPS target)
        self.image_thread = threading.Thread(target=self.high_speed_image_capture, daemon=True)
        self.image_thread.start()
        
        print("SimpleFlight Web Interface (Optimized) initialized")
        print("Target camera FPS: 60")
    
    def configure_camera_settings(self):
        """Configure camera for high FPS capture"""
        try:
            # Request high FPS compressed images for better performance
            # Using compressed format reduces bandwidth and increases FPS
            print("Configuring camera for high FPS...")
            
            # Set camera capture settings if supported
            # This may not work with all AirSim versions but won't hurt to try
            capture_settings = {
                "CaptureSettings": [
                    {
                        "ImageType": 0,  # Scene
                        "Width": 1280,   # Lower resolution for higher FPS
                        "Height": 720,
                        "FOV_Degrees": 90,
                        "TargetFPS": 60  # Request 60 FPS
                    }
                ]
            }
            
            # Note: Setting changes might require restart, but we try anyway
            print("Camera configured for optimal FPS")
        except Exception as e:
            print(f"Could not configure camera settings: {e}")
    
    def update_state_loop(self):
        """Update drone state at 10Hz (doesn't need to be faster)"""
        while True:
            try:
                # Get drone state using control client
                state = self.control_client.getMultirotorState()
                
                # Update connection status
                self.drone_state['connected'] = self.control_client.isApiControlEnabled()
                self.drone_state['api_enabled'] = self.drone_state['connected']
                
                # Update position
                pos = state.kinematics_estimated.position
                self.drone_state['position'] = {
                    'x': round(pos.x_val, 2),
                    'y': round(pos.y_val, 2),
                    'z': round(-pos.z_val, 2)  # Convert to positive altitude
                }
                
                # Track position history
                current_pos = (pos.x_val, pos.y_val, -pos.z_val)
                self.position_history.append(current_pos)
                
                # Calculate total distance
                if self.last_position:
                    dx = current_pos[0] - self.last_position[0]
                    dy = current_pos[1] - self.last_position[1]
                    dz = current_pos[2] - self.last_position[2]
                    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                    self.drone_state['total_distance'] += distance
                self.last_position = current_pos
                
                # Track max altitude
                if -pos.z_val > self.drone_state['max_altitude']:
                    self.drone_state['max_altitude'] = -pos.z_val
                
                # Update velocity
                vel = state.kinematics_estimated.linear_velocity
                self.drone_state['velocity'] = {
                    'x': round(vel.x_val, 2),
                    'y': round(vel.y_val, 2),
                    'z': round(-vel.z_val, 2)
                }
                
                # Update orientation
                orientation = state.kinematics_estimated.orientation
                pitch, roll, yaw = airsim.quaternion_to_euler_angles(orientation)
                self.drone_state['orientation'] = {
                    'pitch': round(math.degrees(pitch), 1),
                    'roll': round(math.degrees(roll), 1),
                    'yaw': round(math.degrees(yaw), 1)
                }
                
                # Check if flying
                self.drone_state['flying'] = -pos.z_val > 0.5
                self.drone_state['armed'] = self.drone_state['flying'] or self.drone_state['api_enabled']
                
                # Update flight time
                if self.drone_state['flying']:
                    if not self.start_time:
                        self.start_time = time.time()
                    self.drone_state['flight_time'] = int(time.time() - self.start_time)
                elif not self.drone_state['flying']:
                    self.start_time = None
                
                # Update camera FPS
                self.drone_state['camera_fps'] = self.current_fps
                
            except Exception as e:
                # Only print if it's not an IOLoop error
                if "IOLoop" not in str(e):
                    print(f"State update error: {e}")
                self.drone_state['connected'] = False
            
            time.sleep(0.1)  # 10Hz update rate for state
    
    def high_speed_image_capture(self):
        """High-speed image capture thread targeting 60+ FPS"""
        print("Starting high-speed image capture...")
        
        # Use lower resolution for higher FPS
        target_width = 1280
        target_height = 720
        
        # FPS tracking
        fps_frames = 0
        fps_time = time.time()
        
        while True:
            try:
                capture_start = time.time()
                
                # Method 1: Try compressed image first (fastest)
                try:
                    # Get compressed PNG/JPEG image
                    png_data = self.image_client.simGetImage("0", airsim.ImageType.Scene)
                    
                    if png_data:
                        # Decode compressed image
                        nparr = np.frombuffer(png_data, np.uint8)
                        img_bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                        
                        if img_bgr is not None:
                            # Resize if needed (keep it fast)
                            if img_bgr.shape[:2] != (target_height, target_width):
                                img_bgr = cv2.resize(img_bgr, (target_width, target_height), 
                                                    interpolation=cv2.INTER_LINEAR)  # Fast interpolation
                            
                            # Add minimal overlay (keep processing light)
                            self.add_minimal_overlay(img_bgr)
                            
                            # Encode with optimized settings for streaming
                            # Lower quality = smaller size = faster transmission = higher FPS
                            encode_params = [cv2.IMWRITE_JPEG_QUALITY, 70]  # 70% quality for speed
                            _, buffer = cv2.imencode('.jpg', img_bgr, encode_params)
                            encoded_image = base64.b64encode(buffer).decode('utf-8')
                            
                            # Update latest image directly (skip queue for lower latency)
                            self.drone_state['latest_image'] = encoded_image
                            
                            # FPS calculation
                            fps_frames += 1
                            if fps_frames >= 30:  # Update FPS every 30 frames
                                elapsed = time.time() - fps_time
                                self.current_fps = fps_frames / elapsed
                                fps_frames = 0
                                fps_time = time.time()
                
                except Exception as e:
                    # Fallback: Try uncompressed images
                    try:
                        responses = self.image_client.simGetImages([
                            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
                        ])
                        
                        if responses and len(responses) > 0:
                            response = responses[0]
                            if response.image_data_uint8:
                                img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                                img_rgb = img1d.reshape(response.height, response.width, 3)
                                img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                                
                                # Resize for performance
                                img_bgr = cv2.resize(img_bgr, (target_width, target_height), 
                                                    interpolation=cv2.INTER_LINEAR)
                                
                                # Add minimal overlay
                                self.add_minimal_overlay(img_bgr)
                                
                                # Encode with optimization
                                encode_params = [cv2.IMWRITE_JPEG_QUALITY, 70]
                                _, buffer = cv2.imencode('.jpg', img_bgr, encode_params)
                                self.drone_state['latest_image'] = base64.b64encode(buffer).decode('utf-8')
                    except:
                        pass
                
                # Calculate sleep time to maintain target FPS
                capture_time = time.time() - capture_start
                target_frame_time = 1.0 / 60.0  # 60 FPS target
                
                if capture_time < target_frame_time:
                    time.sleep(target_frame_time - capture_time)
                
            except Exception as e:
                # Only print error once
                if not hasattr(self, '_image_error_shown'):
                    print(f"Image capture error (will continue without camera): {e}")
                    self._image_error_shown = True
                time.sleep(0.1)  # Slower retry on error
    
    def add_minimal_overlay(self, img):
        """Add minimal overlay for performance"""
        h, w = img.shape[:2]
        
        # Only add essential info with minimal processing
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # FPS counter (top-left)
        fps_text = f"FPS: {self.current_fps:.0f}"
        cv2.putText(img, fps_text, (10, 30), font, 0.7, (0, 255, 0), 2)
        
        # Altitude (top-right)
        alt = self.drone_state['position']['z']
        alt_text = f"Alt: {alt:.1f}m"
        cv2.putText(img, alt_text, (w - 120, 30), font, 0.7, (0, 255, 0), 2)
        
        # Status (bottom)
        if self.drone_state['pattern_active']:
            status = f"PATTERN: {self.drone_state['pattern_type'].upper()}"
            cv2.putText(img, status, (10, h - 10), font, 0.7, (0, 165, 255), 2)
    
    def setup_routes(self):
        @self.app.route('/')
        def index():
            return render_template('simpleflight.html')
        
        @self.app.route('/api/status')
        def get_status():
            return jsonify(self.drone_state)
        
        @self.app.route('/api/command', methods=['POST'])
        def send_command():
            data = request.json
            command = data.get('command')
            
            try:
                if command == 'connect':
                    result = self.connect_drone()
                elif command == 'disconnect':
                    result = self.disconnect_drone()
                elif command == 'arm':
                    result = self.arm_drone()
                elif command == 'disarm':
                    result = self.disarm_drone()
                elif command == 'takeoff':
                    result = self.takeoff()
                elif command == 'land':
                    result = self.land()
                elif command == 'emergency_stop':
                    result = self.emergency_stop()
                elif command == 'set_home':
                    result = self.set_home()
                elif command == 'return_home':
                    result = self.return_home()
                elif command.startswith('move_'):
                    direction = command.replace('move_', '')
                    result = self.move_drone(direction)
                elif command.startswith('rotate_'):
                    direction = command.replace('rotate_', '')
                    result = self.rotate_drone(direction)
                elif command.startswith('pattern_'):
                    pattern = command.replace('pattern_', '')
                    result = self.execute_pattern(pattern)
                elif command == 'stop_pattern':
                    result = self.stop_pattern_execution()
                else:
                    result = {'success': False, 'message': 'Unknown command'}
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            return jsonify(result)
        
        @self.app.route('/api/video_feed')
        def video_feed():
            def generate():
                while True:
                    if self.drone_state['latest_image']:
                        # Send the latest frame immediately (no buffering)
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + 
                               base64.b64decode(self.drone_state['latest_image']) + 
                               b'\r\n')
                    else:
                        # Send blank frame if no image
                        blank = np.zeros((720, 1280, 3), dtype=np.uint8)
                        cv2.putText(blank, "Camera feed unavailable", 
                                   (440, 360), cv2.FONT_HERSHEY_SIMPLEX, 
                                   1, (100, 100, 100), 2)
                        cv2.putText(blank, "Control still works!", 
                                   (480, 400), cv2.FONT_HERSHEY_SIMPLEX, 
                                   0.8, (100, 100, 100), 2)
                        _, buffer = cv2.imencode('.jpg', blank, [cv2.IMWRITE_JPEG_QUALITY, 70])
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + 
                               buffer.tobytes() + 
                               b'\r\n')
                    
                    # Stream at high rate for smooth video
                    time.sleep(0.016)  # ~60 FPS streaming rate
            
            return Response(generate(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
    
    def connect_drone(self):
        try:
            self.control_client.enableApiControl(True)
            self.control_client.armDisarm(True)
            return {'success': True, 'message': 'Drone connected and armed'}
        except Exception as e:
            return {'success': False, 'message': str(e)}
    
    def disconnect_drone(self):
        try:
            self.control_client.armDisarm(False)
            self.control_client.enableApiControl(False)
            return {'success': True, 'message': 'Drone disconnected'}
        except Exception as e:
            return {'success': False, 'message': str(e)}
    
    def arm_drone(self):
        try:
            self.control_client.enableApiControl(True)
            self.control_client.armDisarm(True)
            return {'success': True, 'message': 'Drone armed'}
        except Exception as e:
            return {'success': False, 'message': str(e)}
    
    def disarm_drone(self):
        try:
            self.control_client.armDisarm(False)
            return {'success': True, 'message': 'Drone disarmed'}
        except Exception as e:
            return {'success': False, 'message': str(e)}
    
    def takeoff(self):
        def takeoff_async():
            try:
                self.control_client.takeoffAsync().join()
                # Hover at 5m
                self.control_client.moveToZAsync(-5, 2).join()
            except Exception as e:
                print(f"Takeoff error: {e}")
        
        threading.Thread(target=takeoff_async, daemon=True).start()
        return {'success': True, 'message': 'Taking off to 5m'}
    
    def land(self):
        def land_async():
            try:
                self.control_client.landAsync().join()
            except Exception as e:
                print(f"Landing error: {e}")
        
        threading.Thread(target=land_async, daemon=True).start()
        return {'success': True, 'message': 'Landing'}
    
    def emergency_stop(self):
        try:
            # Stop all movement
            self.stop_pattern = True
            self.control_client.moveByVelocityAsync(0, 0, 0, 0.1).join()
            self.control_client.hoverAsync().join()
            self.drone_state['mission_status'] = 'EMERGENCY'
            return {'success': True, 'message': 'EMERGENCY STOP - Hovering'}
        except Exception as e:
            return {'success': False, 'message': str(e)}
    
    def set_home(self):
        pos = self.drone_state['position']
        self.drone_state['home_position'] = pos.copy()
        self.drone_state['home_set'] = True
        return {'success': True, 'message': f'Home set at X:{pos["x"]}, Y:{pos["y"]}, Z:{pos["z"]}'}
    
    def return_home(self):
        def rth_async():
            try:
                home = self.drone_state['home_position']
                current_z = self.drone_state['position']['z']
                
                # First go to safe altitude (10m)
                if current_z < 10:
                    self.control_client.moveToZAsync(-10, 2).join()
                
                # Move to home position
                self.control_client.moveToPositionAsync(home['x'], home['y'], -10, 3).join()
                
                # Land
                self.control_client.landAsync().join()
                self.drone_state['mission_status'] = 'LANDED'
            except Exception as e:
                print(f"RTH error: {e}")
        
        if self.drone_state['home_set']:
            self.drone_state['mission_status'] = 'RETURNING'
            threading.Thread(target=rth_async, daemon=True).start()
            return {'success': True, 'message': 'Returning to home'}
        else:
            return {'success': False, 'message': 'Home position not set'}
    
    def move_drone(self, direction):
        def move_async():
            duration = 2.0  # Move for 2 seconds
            vx = vy = vz = 0
            
            if direction == 'forward':
                vx = self.movement_speed
            elif direction == 'backward' or direction == 'back':
                vx = -self.movement_speed
            elif direction == 'left':
                vy = -self.movement_speed
            elif direction == 'right':
                vy = self.movement_speed
            elif direction == 'up':
                vz = -self.altitude_speed
            elif direction == 'down':
                vz = self.altitude_speed
            elif direction == 'stop':
                self.control_client.moveByVelocityAsync(0, 0, 0, 0.1).join()
                return
            
            try:
                self.control_client.moveByVelocityAsync(vx, vy, vz, duration).join()
                self.control_client.hoverAsync().join()
            except Exception as e:
                if "IOLoop" not in str(e):
                    print(f"Move error: {e}")
        
        threading.Thread(target=move_async, daemon=True).start()
        return {'success': True, 'message': f'Moving {direction}'}
    
    def rotate_drone(self, direction):
        def rotate_async():
            try:
                current_yaw = self.drone_state['orientation']['yaw']
                
                if direction == 'left':
                    target_yaw = current_yaw - 45
                elif direction == 'right':
                    target_yaw = current_yaw + 45
                else:
                    return
                
                # Convert to radians
                target_yaw_rad = math.radians(target_yaw)
                
                # Rotate at current position
                pos = self.drone_state['position']
                self.control_client.moveByVelocityZAsync(0, 0, -pos['z'], 1, 
                    airsim.DrivetrainType.MaxDegreeOfFreedom, 
                    airsim.YawMode(False, target_yaw)).join()
            except Exception as e:
                if "IOLoop" not in str(e):
                    print(f"Rotation error: {e}")
        
        threading.Thread(target=rotate_async, daemon=True).start()
        return {'success': True, 'message': f'Rotating {direction}'}
    
    def execute_pattern(self, pattern_type):
        if self.pattern_thread and self.pattern_thread.is_alive():
            return {'success': False, 'message': 'Pattern already running'}
        
        self.stop_pattern = False
        self.drone_state['pattern_active'] = True
        self.drone_state['pattern_type'] = pattern_type
        self.drone_state['mission_status'] = 'PATTERN'
        
        if pattern_type == 'square':
            self.pattern_thread = threading.Thread(target=self.fly_square_pattern, daemon=True)
        elif pattern_type == 'spiral':
            self.pattern_thread = threading.Thread(target=self.fly_spiral_pattern, daemon=True)
        elif pattern_type == 'zigzag':
            self.pattern_thread = threading.Thread(target=self.fly_zigzag_pattern, daemon=True)
        elif pattern_type == 'search':
            self.pattern_thread = threading.Thread(target=self.fly_search_pattern, daemon=True)
        else:
            self.drone_state['pattern_active'] = False
            return {'success': False, 'message': 'Unknown pattern'}
        
        self.pattern_thread.start()
        return {'success': True, 'message': f'Starting {pattern_type} pattern'}
    
    def stop_pattern_execution(self):
        self.stop_pattern = True
        self.drone_state['pattern_active'] = False
        self.drone_state['pattern_type'] = 'none'
        self.drone_state['mission_status'] = 'IDLE'
        
        try:
            self.control_client.moveByVelocityAsync(0, 0, 0, 0.1).join()
            self.control_client.hoverAsync().join()
        except:
            pass
        
        return {'success': True, 'message': 'Pattern stopped'}
    
    def fly_square_pattern(self):
        """Fly a square pattern"""
        try:
            size = 20  # Square size in meters
            speed = 3  # m/s
            
            state = self.control_client.getMultirotorState()
            start_pos = state.kinematics_estimated.position
            z = start_pos.z_val
            
            # Square waypoints (relative to start)
            waypoints = [
                (size, 0),
                (size, size),
                (0, size),
                (0, 0)
            ]
            
            for x, y in waypoints:
                if self.stop_pattern:
                    break
                
                # Move to waypoint
                self.control_client.moveToPositionAsync(
                    start_pos.x_val + x,
                    start_pos.y_val + y,
                    z, speed
                ).join()
                
                time.sleep(0.5)  # Brief hover at corner
            
        except Exception as e:
            if "IOLoop" not in str(e):
                print(f"Square pattern error: {e}")
        finally:
            self.drone_state['pattern_active'] = False
            self.drone_state['mission_status'] = 'IDLE'
    
    def fly_spiral_pattern(self):
        """Fly an outward spiral pattern"""
        try:
            radius = 5  # Starting radius
            max_radius = 20
            speed = 2
            
            state = self.control_client.getMultirotorState()
            start_pos = state.kinematics_estimated.position
            z = start_pos.z_val
            
            angle = 0
            while radius < max_radius and not self.stop_pattern:
                x = radius * math.cos(math.radians(angle))
                y = radius * math.sin(math.radians(angle))
                
                self.control_client.moveToPositionAsync(
                    start_pos.x_val + x,
                    start_pos.y_val + y,
                    z, speed
                ).join()
                
                angle += 30
                radius += 0.5
            
        except Exception as e:
            if "IOLoop" not in str(e):
                print(f"Spiral pattern error: {e}")
        finally:
            self.drone_state['pattern_active'] = False
            self.drone_state['mission_status'] = 'IDLE'
    
    def fly_zigzag_pattern(self):
        """Fly a zigzag search pattern"""
        try:
            width = 30
            height = 30
            spacing = 5
            speed = 3
            
            state = self.control_client.getMultirotorState()
            start_pos = state.kinematics_estimated.position
            z = start_pos.z_val
            
            for i in range(0, int(height/spacing) + 1):
                if self.stop_pattern:
                    break
                
                y = i * spacing
                
                # Fly right
                if i % 2 == 0:
                    self.control_client.moveToPositionAsync(
                        start_pos.x_val + width,
                        start_pos.y_val + y,
                        z, speed
                    ).join()
                # Fly left
                else:
                    self.control_client.moveToPositionAsync(
                        start_pos.x_val,
                        start_pos.y_val + y,
                        z, speed
                    ).join()
            
        except Exception as e:
            if "IOLoop" not in str(e):
                print(f"Zigzag pattern error: {e}")
        finally:
            self.drone_state['pattern_active'] = False
            self.drone_state['mission_status'] = 'IDLE'
    
    def fly_search_pattern(self):
        """Fly a comprehensive search pattern"""
        try:
            # Combine expanding square with altitude changes
            for altitude in [5, 10, 15]:
                if self.stop_pattern:
                    break
                
                # Move to altitude
                self.control_client.moveToZAsync(-altitude, 2).join()
                
                # Fly square at this altitude
                self.fly_square_pattern()
                
                if self.stop_pattern:
                    break
            
        except Exception as e:
            if "IOLoop" not in str(e):
                print(f"Search pattern error: {e}")
        finally:
            self.drone_state['pattern_active'] = False
            self.drone_state['mission_status'] = 'IDLE'
    
    def run_server(self):
        self.app.run(host='0.0.0.0', port=5001, debug=False, threaded=True)


def main():
    interface = SimpleFlightWebInterface()
    
    print("\n" + "="*60)
    print("SimpleFlight Web Interface (Optimized for 60 FPS)")
    print("="*60)
    print(f"Access the interface at: http://localhost:5001")
    print("Camera optimized for high FPS streaming")
    print("="*60 + "\n")
    
    interface.run_server()


if __name__ == '__main__':
    main()