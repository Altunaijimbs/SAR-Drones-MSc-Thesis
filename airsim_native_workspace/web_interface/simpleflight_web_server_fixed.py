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
            'max_altitude': 0.0
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
        
        # Setup Flask routes
        self.setup_routes()
        
        # Start state update thread
        self.update_thread = threading.Thread(target=self.update_state_loop, daemon=True)
        self.update_thread.start()
        
        # Start separate image capture thread
        self.image_thread = threading.Thread(target=self.image_capture_loop, daemon=True)
        self.image_thread.start()
        
        print("SimpleFlight Web Interface initialized")
    
    def update_state_loop(self):
        """Continuously update drone state (without images)"""
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
                
            except Exception as e:
                # Only print if it's not an IOLoop error
                if "IOLoop" not in str(e):
                    print(f"State update error: {e}")
                self.drone_state['connected'] = False
            
            time.sleep(0.1)  # 10Hz update rate
    
    def image_capture_loop(self):
        """Separate thread for image capture"""
        while True:
            try:
                # Get camera image using separate client
                response = self.image_client.simGetImage("0", airsim.ImageType.Scene)
                
                if response:
                    # Convert raw image to numpy array
                    img1d = np.frombuffer(response, dtype=np.uint8)
                    
                    # Decode JPEG/PNG image
                    img_rgb = cv2.imdecode(img1d, cv2.IMREAD_COLOR)
                    
                    if img_rgb is not None:
                        # Resize for web streaming
                        img_rgb = cv2.resize(img_rgb, (1280, 720))
                        
                        # Add overlay information
                        self.add_overlay(img_rgb)
                        
                        # Encode to JPEG
                        encode_params = [cv2.IMWRITE_JPEG_QUALITY, 85]
                        _, buffer = cv2.imencode('.jpg', img_rgb, encode_params)
                        self.drone_state['latest_image'] = base64.b64encode(buffer).decode('utf-8')
                
            except Exception as e:
                # Only print error once
                if not hasattr(self, '_image_error_shown'):
                    print(f"Image capture error (will continue without camera): {e}")
                    self._image_error_shown = True
            
            time.sleep(0.033)  # ~30 FPS
    
    def add_overlay(self, img):
        """Add HUD overlay to image"""
        h, w = img.shape[:2]
        
        # Add semi-transparent overlay for text
        overlay = img.copy()
        cv2.rectangle(overlay, (10, 10), (350, 120), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, img, 0.7, 0, img)
        
        # Add text info
        font = cv2.FONT_HERSHEY_SIMPLEX
        color = (0, 255, 0)  # Green
        
        pos = self.drone_state['position']
        vel = self.drone_state['velocity']
        
        cv2.putText(img, f"Alt: {pos['z']:.1f}m", (20, 35), font, 0.7, color, 2)
        cv2.putText(img, f"Pos: X:{pos['x']:.1f} Y:{pos['y']:.1f}", (20, 65), font, 0.7, color, 2)
        cv2.putText(img, f"Vel: {math.sqrt(vel['x']**2 + vel['y']**2):.1f}m/s", (20, 95), font, 0.7, color, 2)
        
        # Add crosshair
        cv2.line(img, (w//2 - 20, h//2), (w//2 - 5, h//2), (0, 255, 0), 2)
        cv2.line(img, (w//2 + 5, h//2), (w//2 + 20, h//2), (0, 255, 0), 2)
        cv2.line(img, (w//2, h//2 - 20), (w//2, h//2 - 5), (0, 255, 0), 2)
        cv2.line(img, (w//2, h//2 + 5), (w//2, h//2 + 20), (0, 255, 0), 2)
        
        # Add status indicator
        status = "PATTERN" if self.drone_state['pattern_active'] else "MANUAL"
        status_color = (0, 165, 255) if self.drone_state['pattern_active'] else (0, 255, 0)
        cv2.putText(img, status, (w - 150, 35), font, 0.8, status_color, 2)
    
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
                        _, buffer = cv2.imencode('.jpg', blank)
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + 
                               buffer.tobytes() + 
                               b'\r\n')
                    time.sleep(0.033)  # ~30 FPS
            
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
        self.app.run(host='0.0.0.0', port=5001, debug=False)


def main():
    interface = SimpleFlightWebInterface()
    
    print("\n" + "="*60)
    print("SimpleFlight Web Interface (Fixed)")
    print("="*60)
    print(f"Access the interface at: http://localhost:5001")
    print("Make sure UE5 is running with SimpleFlight mode enabled")
    print("Camera feed may not work with older AirSim versions")
    print("But drone control will still function!")
    print("="*60 + "\n")
    
    interface.run_server()


if __name__ == '__main__':
    main()