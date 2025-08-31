#!/usr/bin/env python3
"""
Autonomous SAR Executor - POSITION ONLY VERSION
Uses only position commands throughout - no velocity mixing
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Twist, Point
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import json
import time
import threading
import math
from enum import Enum
from openai import OpenAI
import os
import csv
from datetime import datetime

class MissionState(Enum):
    IDLE = "idle"
    SEARCHING = "searching"
    TARGET_FOUND = "target_found"
    CONFIRMING = "confirming"  # New state for verification
    PREPARING = "preparing"  # New state for system preparation after confirmation
    APPROACHING = "approaching"  # New state for approach before orbit
    CENTERING = "centering"  # V6.5: Center target in camera view
    LOGGING = "logging"      # V6.5: Log GPS coordinates
    RETURNING_TO_YAW = "returning_to_yaw"  # V6.5: Return to original yaw
    RETURNING_TO_ALTITUDE = "returning_to_altitude"  # V6.5: Return to search altitude
    HOVERING_INSPECTION = "hovering_inspection"
    MISSION_COMPLETE = "mission_complete"
    PAUSED = "paused"
    ABORTED = "aborted"

class AutonomousSARExecutor(Node):
    def __init__(self):
        super().__init__('autonomous_sar_executor')  # V6.5 - Same node name for compatibility
        
        # OpenAI setup
        self.api_key = os.getenv('OPENAI_API_KEY', '')
        self.client = OpenAI(api_key=self.api_key) if self.api_key else None
        
        # CV Bridge for depth processing
        self.bridge = CvBridge()
        
        # MAVROS QoS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers - ONLY position commands!
        self.position_pub = self.create_publisher(String, '/autonomous/position_command', 10)
        self.pattern_pub = self.create_publisher(String, '/pattern_command', 10)
        self.pattern_ctrl_pub = self.create_publisher(String, '/pattern_control', 10)
        self.priority_pub = self.create_publisher(String, '/priority_mode', 10)
        self.status_pub = self.create_publisher(String, '/autonomous/status', 10)
        self.mission_pub = self.create_publisher(String, '/autonomous/mission', 10)
        
        # Subscribers
        self.mission_sub = self.create_subscription(
            String, '/autonomous/mission_command',
            self.parse_mission, 10
        )
        
        self.mission_ctrl_sub = self.create_subscription(
            String, '/autonomous/mission_control',
            self.handle_mission_control, 10
        )
        
        # Vision subscribers
        self.detection_sub = self.create_subscription(
            String, '/yolo/detections',
            self.handle_detections, 10
        )
        
        # Depth camera subscriber - FIXED WITH CORRECT TOPIC!
        # Using DepthPerspective (ImageType 2) for better accuracy
        self.depth_sub = self.create_subscription(
            Image, '/airsim_node/PX4/depth_camera_DepthPerspective/image',
            self.handle_depth, 10
        )
        self.get_logger().info('✅ Connected to depth camera (DepthPerspective) for accurate distance')
        
        # Position tracking
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.update_position,
            qos_profile=self.mavros_qos
        )
        
        # Pattern status
        self.pattern_status_sub = self.create_subscription(
            String, '/pattern_status',
            self.update_pattern_status, 10
        )
        
        # State variables
        self.state = MissionState.IDLE
        self.mission = None
        self.current_position = {'x': 0, 'y': 0, 'z': 0}
        self.current_yaw = 0
        self.target_position = None
        self.target_distance = None
        self.depth_image = None
        self.latest_detection = None
        self.pattern_active = False
        self.pattern_started = False
        
        # V6.5: Simplified SAR behavior - no orbiting
        self.centering_tolerance = 150  # pixels from center (increased from 100 for less precision required)
        self.centering_attempts = 0
        self.max_centering_attempts = 10  # Reduced from 20 - if can't center in 10 attempts, move on
        self.image_center_x = 640  # Assuming 1280x720
        self.target_pixel_x = 640
        self.centering_yaw_speed = 0.3  # rad/s for yaw adjustment
        self.last_detection_update = 0  # Track when detection was last updated
        
        # Temporary orbit variables to prevent errors (not actually used)
        self.orbit_radius = 5.0
        self.max_orbits = 0
        self.orbit_index = 0
        self.orbit_waypoints = []
        self.waypoints_per_orbit = 8
        self.hover_distance = 2.0
        self.hover_duration = 10.0
        
        # Survivor logging
        self.survivor_log = []
        self.survivor_positions = []  # Track positions to avoid duplicates
        self.duplicate_distance_threshold = 50.0  # meters - increased to 50m to account for groups of people close together
        self.log_file = f'/home/mbs/SAR-Drones-MSc-Thesis/survivor_log_v6_5_{datetime.now().strftime("%Y%m%d_%H%M%S")}.csv'
        self.init_log_file()
        
        # Logging state
        self.logging_start_time = None
        self.logging_duration = 3.0  # seconds to log and confirm
        
        # Altitude tracking for return to search altitude
        self.search_altitude = None  # Store altitude when detection happens
        self.returning_to_altitude = False
        self.altitude_reached = False
        
        # Yaw tracking for return to original heading
        self.original_yaw = None  # Store yaw before centering
        self.yaw_restored = False
        self.yaw_return_attempts = 0
        
        # Approach state tracking
        self.approach_position = None
        self.approach_command_time = 0
        self.approach_command_count = 0
        
        # Detection confirmation parameters (adjustable for sensitivity)
        self.detection_count = 0
        # ADJUST THESE FOR SENSITIVITY:
        self.required_detections = 5  # Increased from 3 for more thorough confirmation
        self.max_detection_gap = 3.0  # Increased from 2.0 to allow for detection fluctuations
        self.confirmation_timeout = 15.0  # Increased from 10.0 for more time
        self.position_variance_threshold = 5.0  # Max position variance allowed
        
        # Transition timing parameters
        self.confirmation_hold_time = 2.0  # Hold after confirmation before approach
        self.confirmation_complete_time = None  # When confirmation finished
        
        # Tracking variables
        self.last_detection_time = 0
        self.confirmation_start_time = None
        self.detection_positions = []  # Track detection position stability
        
        self.get_logger().info(f'🎯 Detection sensitivity settings:')
        self.get_logger().info(f'   • {self.required_detections} detections needed')
        self.get_logger().info(f'   • Max {self.max_detection_gap}s between detections')
        self.get_logger().info(f'   • {self.confirmation_hold_time}s preparation after confirmation')
        
        # Position tracking for waypoint reached
        self.last_waypoint_time = 0
        self.waypoint_threshold = 2.5  # meters (more forgiving for position control)
        self.waypoint_timeout = 15.0  # seconds (allow more time to reach waypoint)
        
        # Pattern timing
        self.pattern_start_time = None
        self.min_pattern_duration = 30.0  # Increased minimum seconds before pattern can complete (was 10.0)
        
        # Temporal filtering for detections
        self.last_sar_completion_time = None
        self.detection_cooldown_period = 5.0  # Ignore detections for 5s after completing SAR
        
        # Control thread
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        
        self.get_logger().info('🚁 Autonomous SAR Executor V6.5 initialized')
        self.get_logger().info('  • Simplified behavior: Approach → Center → Log → Resume')
        self.get_logger().info('  • GPS logging to CSV file')
        self.get_logger().info(f'  • Log file: {self.log_file}')
        self.get_logger().info(f'  • Duplicate prevention: {self.duplicate_distance_threshold}m radius')
        self.get_logger().info('  • Each survivor logged only once!')
    
    def init_log_file(self):
        """Initialize CSV file for survivor logging"""
        try:
            with open(self.log_file, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['Timestamp', 'GPS_Lat', 'GPS_Lon', 'Local_X', 'Local_Y', 
                               'Altitude', 'Confidence', 'Distance'])
            self.get_logger().info(f'Created survivor log: {self.log_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to create log file: {e}')
    
    def handle_depth(self, msg):
        """Process depth image for distance estimation"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            self.get_logger().debug(f'Depth image received: {self.depth_image.shape}')
        except Exception as e:
            self.get_logger().error(f'Depth processing error: {e}')
    
    def estimate_target_distance(self, detection):
        """Estimate distance using depth camera and bounding box"""
        distance_estimates = []
        
        # Method 1: Depth camera
        if self.depth_image is not None and detection:
            try:
                bbox = detection.get('bbox', {})
                height, width = self.depth_image.shape
                center_x = int(bbox.get('x', 0.5) * width)
                center_y = int(bbox.get('y', 0.5) * height)
                
                roi_size = 10
                x1 = max(0, center_x - roi_size)
                x2 = min(width, center_x + roi_size)
                y1 = max(0, center_y - roi_size)
                y2 = min(height, center_y + roi_size)
                
                depth_roi = self.depth_image[y1:y2, x1:x2]
                depth_value = np.median(depth_roi[depth_roi > 0])
                
                if not np.isnan(depth_value) and depth_value > 0:
                    distance_estimates.append(depth_value)
                    self.get_logger().info(f'📏 Depth camera distance: {depth_value:.2f}m')
            except Exception as e:
                self.get_logger().error(f'Depth estimation error: {e}')
        
        # Method 2: Bounding box size
        if detection:
            bbox = detection.get('bbox', {})
            bbox_height = bbox.get('height', 0)
            
            if bbox_height > 0:
                # CALIBRATED PARAMETERS - Tested with real person at multiple distances
                # Accuracy: <10% error at 10-20m, ~20% at 30m+
                human_height = 1.8  # Calibrated (was 1.7) - 1.8m works better
                image_height = 1080  # AirSim camera resolution
                fov_rad = math.radians(80)  # Calibrated FOV (was 90) - 80° is more accurate
                
                pixel_height = bbox_height * image_height
                distance_bbox = (human_height * image_height) / (pixel_height * 2 * math.tan(fov_rad/2))
                
                distance_estimates.append(distance_bbox)
                self.get_logger().info(f'📐 Bbox size distance: {distance_bbox:.2f}m')
        
        # Method 3: Altitude-based
        if abs(self.current_position['z']) > 2:
            altitude = abs(self.current_position['z'])
            camera_pitch = math.radians(10)
            ground_distance = altitude / math.cos(camera_pitch)
            distance_estimates.append(ground_distance)
            self.get_logger().info(f'📍 Altitude-based distance: {ground_distance:.2f}m')
        
        if distance_estimates:
            # Depth camera gets 3x weight when available (very accurate at all ranges)
            # Bbox gets 1x weight (backup method, less accurate at long range)
            weights = [3.0 if i == 0 and self.depth_image is not None else 1.0 
                      for i in range(len(distance_estimates))]
            weighted_distance = np.average(distance_estimates, weights=weights[:len(distance_estimates)])
            
            self.get_logger().info(f'🎯 Final estimated distance: {weighted_distance:.2f}m')
            
            # Confidence assessment based on available methods
            if self.depth_image is not None:
                self.get_logger().info('   📊 High confidence (depth camera available)')
            else:
                self.get_logger().info('   ⚠️  Medium confidence (bbox only, best at 10-20m)')
            
            return weighted_distance
        else:
            self.get_logger().warn('⚠️  No distance estimation available, using default 10m')
            return 10.0
    
    def calculate_target_position(self, detection, distance):
        """Calculate target's 3D position from detection and distance"""
        bbox = detection.get('bbox', {})
        img_x = bbox.get('x', 0.5) - 0.5  # -0.5 to 0.5 (left to right)
        img_y = bbox.get('y', 0.5) - 0.5  # -0.5 to 0.5 (top to bottom)
        
        # Log image coordinates for debugging
        self.get_logger().info(f'🎯 Detection in image: x={bbox.get("x", 0.5):.2f} ({"right" if img_x > 0 else "left"}), y={bbox.get("y", 0.5):.2f}')
        
        fov_h = math.radians(90)
        fov_v = math.radians(67.5)
        
        angle_h = img_x * fov_h  # Positive = right, Negative = left
        angle_v = img_y * fov_v
        
        camera_pitch = math.radians(10)
        total_pitch = camera_pitch + angle_v
        
        forward_distance = distance * math.cos(total_pitch) * math.cos(angle_h)
        lateral_distance = distance * math.cos(total_pitch) * math.sin(angle_h)  # Positive = right
        vertical_distance = -distance * math.sin(total_pitch)
        
        self.get_logger().info(f'📐 Relative to drone: forward={forward_distance:.1f}m, lateral={lateral_distance:.1f}m ({"right" if lateral_distance > 0 else "left"})')
        self.get_logger().info(f'🧭 Drone yaw: {math.degrees(self.current_yaw):.1f}°')
        
        # Fixed coordinate transformation - lateral should ADD when target is on right
        world_x = self.current_position['x'] + forward_distance * math.cos(self.current_yaw) + lateral_distance * math.sin(self.current_yaw)
        world_y = self.current_position['y'] + forward_distance * math.sin(self.current_yaw) - lateral_distance * math.cos(self.current_yaw)
        world_z = self.current_position['z'] + vertical_distance
        
        target_pos = {
            'x': world_x,
            'y': world_y,
            'z': max(0, world_z)
        }
        
        self.get_logger().info(f'📍 Target world position: ({world_x:.2f}, {world_y:.2f}, {world_z:.2f})')
        self.get_logger().info(f'🚁 Drone position: ({self.current_position["x"]:.2f}, {self.current_position["y"]:.2f}, {self.current_position["z"]:.2f})')
        return target_pos
    
    def handle_detections(self, msg):
        """Process detections"""
        # V6.5: Also process during CENTERING to update pixel position
        if self.state not in [MissionState.SEARCHING, MissionState.CONFIRMING, MissionState.CENTERING]:
            return
        
        # Ignore detections for a few seconds after completing SAR to avoid immediate re-detection
        if self.last_sar_completion_time:
            time_since_sar = time.time() - self.last_sar_completion_time
            if time_since_sar < self.detection_cooldown_period:
                self.get_logger().debug(f'Ignoring detections for {self.detection_cooldown_period - time_since_sar:.1f}s more (cooldown after SAR)')
                return
        
        try:
            data = json.loads(msg.data)
            detections = data.get('detections', [])
            
            # Log all detections for debugging
            self.get_logger().info(f'Received {len(detections)} detections')
            for d in detections:
                self.get_logger().debug(f"  - {d.get('class', 'unknown')} (conf: {d.get('confidence', 0):.2f})")
            
            for detection in detections:
                obj_class = detection.get('class', '').lower()
                confidence = detection.get('confidence', 0)
                
                # STRICT PERSON-ONLY DETECTION
                # Only react to 'person' class specifically
                if obj_class == 'person' and confidence > 0.5:  # Higher threshold
                    # First check if we're already doing SAR on someone
                    if self.state in [MissionState.PREPARING, MissionState.APPROACHING, 
                                      MissionState.CENTERING, MissionState.LOGGING,
                                      MissionState.RETURNING_TO_YAW, MissionState.RETURNING_TO_ALTITUDE]:
                        self.get_logger().debug(f'Ignoring detection - already in SAR sequence (state: {self.state.value})')
                        continue  # Skip this detection, we're busy with another survivor
                    # V6.5: Always update pixel position for centering
                    bbox = detection.get('bbox', {})
                    self.target_pixel_x = bbox.get('x', 0.5) * 1280  # Convert normalized to pixels
                    
                    # During centering, update detection for fresh data
                    if self.state == MissionState.CENTERING:
                        self.latest_detection = detection  # Update the detection object!
                        self.last_detection_update = time.time()  # Track update time
                        self.get_logger().info(f'📍 Centering update: target at pixel {self.target_pixel_x:.0f}')
                        continue  # Don't process further, just update pixel position
                    
                    self.get_logger().info(f'👁️ Person detected: {obj_class} (conf: {confidence:.2f})')
                else:
                    # Skip non-person detections
                    self.get_logger().debug(f'Ignoring non-person: {obj_class}')
                    continue
                
                # Estimate distance and position
                distance = self.estimate_target_distance(detection)
                position = self.calculate_target_position(detection, distance)
                
                # ALWAYS check for duplicates, regardless of state
                if self.is_duplicate_survivor(position):
                    if self.state == MissionState.SEARCHING:
                        self.get_logger().info(f'⚠️ Ignoring duplicate survivor at ({position["x"]:.1f}, {position["y"]:.1f})')
                        self.get_logger().info(f'   Already logged {len(self.survivor_positions)} survivor(s)')
                    # Don't process this detection further
                    continue
                
                # Check if we're already confirming
                if self.state == MissionState.CONFIRMING:
                    # Update confirmation tracking
                    self.handle_confirmation_detection(detection, distance, position)
                else:
                    # First detection - enter confirmation mode
                    self.enter_confirmation_mode(detection, distance, position)
                    
                break
                    
        except Exception as e:
            self.get_logger().error(f'Detection handling error: {e}')
    
    def enter_confirmation_mode(self, detection, distance, position):
        """Enter confirmation mode for first detection"""
        
        # Double-check for duplicates (redundant safety check)
        if self.is_duplicate_survivor(position):
            self.get_logger().info(f'⚠️ Ignoring duplicate survivor at ({position["x"]:.1f}, {position["y"]:.1f})')
            self.get_logger().info(f'   Already found {len(self.survivor_positions)} survivor(s)')
            return  # Don't enter confirmation mode for duplicates
        
        self.get_logger().info('='*50)
        self.get_logger().info('🔍 ENTERING CONFIRMATION MODE')
        self.get_logger().info(f'  Need {self.required_detections} consecutive detections')
        self.get_logger().info(f'  Max gap between detections: {self.max_detection_gap}s')
        self.get_logger().info(f'  Timeout: {self.confirmation_timeout}s')
        self.get_logger().info('='*50)
        
        # PAUSE pattern (not stop) to preserve state for resumption
        self.get_logger().info('⏸️ PAUSING pattern and holding position for confirmation')
        
        # 1. Pause pattern to preserve waypoint state
        self.pattern_ctrl_pub.publish(String(data='pause'))
        time.sleep(0.05)
        
        # 2. Set priority to autonomous for position control
        self.priority_pub.publish(String(data='autonomous'))
        time.sleep(0.05)
        
        # 3. Send ONE FINAL position hold command, then let PX4 maintain it
        hold_cmd = f"goto:{self.current_position['x']:.2f},{self.current_position['y']:.2f},{self.current_position['z']:.2f}"
        self.position_pub.publish(String(data=hold_cmd))
        self.get_logger().info('🎯 Sent final hold command - PX4 will maintain position')
        self.get_logger().info('📡 Keep-alive node maintains OFFBOARD mode')
        
        # 4. Send stop velocity command to ensure immediate halt
        self.position_pub.publish(String(data='stop'))
        
        self.get_logger().info(f'📍 Holding at ({self.current_position["x"]:.2f}, {self.current_position["y"]:.2f}, {self.current_position["z"]:.2f})')
        
        # Initialize confirmation tracking
        self.state = MissionState.CONFIRMING
        self.detection_count = 1
        self.last_detection_time = time.time()
        self.confirmation_start_time = time.time()
        self.confirmation_complete_time = None
        self.detection_positions = [position]
        
        # Store the FIXED hold position to prevent swaying
        self.hold_position = {
            'x': self.current_position['x'],
            'y': self.current_position['y'],
            'z': self.current_position['z']
        }
        self.get_logger().info(f'📌 Fixed hold position: ({self.hold_position["x"]:.2f}, {self.hold_position["y"]:.2f}, {self.hold_position["z"]:.2f})')
        self.latest_detection = detection
        self.target_distance = distance
        self.target_position = position
        
        self.publish_status(f'Confirming detection (1/{self.required_detections}) 👁️')
    
    def handle_confirmation_detection(self, detection, distance, position):
        """Handle subsequent detections during confirmation"""
        current_time = time.time()
        time_since_last = current_time - self.last_detection_time
        
        # Check if detection is timely
        if time_since_last > self.max_detection_gap:
            self.get_logger().warn(f'⚠️ Detection gap too large ({time_since_last:.1f}s > {self.max_detection_gap}s)')
            self.get_logger().warn('   Resetting confirmation process')
            self.reset_confirmation()
            return
        
        # Update tracking
        self.detection_count += 1
        self.last_detection_time = current_time
        self.detection_positions.append(position)
        self.latest_detection = detection
        self.target_distance = distance
        self.target_position = position
        
        # Show progress with visual indicator
        progress_bar = '█' * self.detection_count + '░' * (self.required_detections - self.detection_count)
        self.get_logger().info(f'✓ Detection {self.detection_count}/{self.required_detections} [{progress_bar}]')
        self.publish_status(f'Confirming [{progress_bar}] ({self.detection_count}/{self.required_detections})')
        
        # Check if we have enough detections
        if self.detection_count >= self.required_detections:
            # Check position stability
            if self.check_position_stability():
                self.confirm_target()
            else:
                self.get_logger().warn('⚠️ Position variance too high, need more stable detections')
                self.detection_count = max(self.required_detections - 2, self.detection_count - 1)
    
    def is_duplicate_survivor(self, position):
        """Check if this position is near a previously found survivor"""
        if not position or not self.survivor_positions:
            return False
        
        for prev_pos in self.survivor_positions:
            dx = position['x'] - prev_pos['x']
            dy = position['y'] - prev_pos['y']
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < self.duplicate_distance_threshold:
                self.get_logger().debug(f'Duplicate check: {distance:.1f}m from previous survivor (threshold: {self.duplicate_distance_threshold}m)')
                return True
        
        return False
    
    def check_position_stability(self):
        """Check if detected positions are stable (not jumping around)"""
        if len(self.detection_positions) < 2:
            return True
        
        # Calculate variance in positions
        positions = self.detection_positions[-3:]  # Check last 3 positions
        x_coords = [p['x'] for p in positions if p]
        y_coords = [p['y'] for p in positions if p]
        
        if not x_coords or not y_coords:
            return True
        
        x_variance = max(x_coords) - min(x_coords)
        y_variance = max(y_coords) - min(y_coords)
        total_variance = math.sqrt(x_variance**2 + y_variance**2)
        
        stable = total_variance < self.position_variance_threshold
        if not stable:
            self.get_logger().warn(f'Position variance: {total_variance:.1f}m (threshold: {self.position_variance_threshold}m)')
        
        return stable
    
    def reset_confirmation(self):
        """Reset confirmation tracking and resume search"""
        self.get_logger().info('🔄 Confirmation reset - resuming search')
        self.detection_count = 0
        self.detection_positions = []
        self.confirmation_start_time = None
        self.confirmation_complete_time = None
        self.last_detection_time = 0
        self.state = MissionState.SEARCHING
        
        # Reset priority to pattern focus for searching
        self.priority_pub.publish(String(data='pattern_focus'))
        time.sleep(0.05)
        
        # Resume the pattern from where it was paused
        self.pattern_ctrl_pub.publish(String(data='resume'))
        self.publish_status('Resuming search (false alarm)')
    
    def confirm_target(self):
        """Target confirmed - prepare for approach"""
        self.get_logger().info('='*50)
        self.get_logger().info('✅ TARGET CONFIRMED!')
        self.get_logger().info(f'  Position: ({self.target_position["x"]:.1f}, {self.target_position["y"]:.1f})')
        self.get_logger().info(f'  Distance: {self.target_distance:.1f}m')
        self.get_logger().info(f'  Preparing system for approach...')
        self.get_logger().info('='*50)
        
        # CRITICAL FIX: Add position to survivor list IMMEDIATELY when confirmed
        # This prevents re-detection during the SAR sequence
        if self.target_position and not self.is_duplicate_survivor(self.target_position):
            self.survivor_positions.append({
                'x': self.target_position['x'],
                'y': self.target_position['y'],
                'z': self.target_position['z']
            })
            self.get_logger().info(f'🚩 Marked survivor #{len(self.survivor_positions)} at ({self.target_position["x"]:.1f}, {self.target_position["y"]:.1f})')
            self.get_logger().info(f'   Position tracked to prevent re-detection during SAR sequence')
        
        # V6.5: Record current altitude before approaching
        if self.current_position and self.search_altitude is None:
            self.search_altitude = self.current_position.get('z', 20.0)
            self.get_logger().info(f'📏 Recording search altitude: {self.search_altitude:.1f}m')
        
        # Update hold position for PREPARING state (in case drone moved slightly during confirmation)
        if not hasattr(self, 'hold_position'):
            self.hold_position = {
                'x': self.current_position['x'],
                'y': self.current_position['y'],
                'z': self.current_position['z']
            }
        
        # Transition to PREPARING state
        self.state = MissionState.PREPARING
        self.confirmation_complete_time = time.time()
        self.publish_status('🚁 Target confirmed - preparing for approach...')
        
        # Keep pattern paused (don't stop it completely to preserve state)
        self.get_logger().info('⏸️ Keeping pattern paused during approach')
        self.pattern_ctrl_pub.publish(String(data='pause'))
        time.sleep(0.05)
        # Note: pattern_active remains true so we can resume later
        
        # Set priority to autonomous BEFORE clearing movement
        self.get_logger().info('🎯 Setting priority to autonomous')
        self.priority_pub.publish(String(data='autonomous'))
        time.sleep(0.05)
        
        # NO MORE POSITION COMMANDS - PX4 is already holding from confirmation
        self.get_logger().info('📡 Maintaining position via PX4 internal hold (no commands needed)')
        self.get_logger().info(f'⏳ Waiting {self.confirmation_hold_time}s before approach...')
    
    def start_approach_sequence(self):
        """Approach the target to optimal distance before orbiting"""
        if not self.target_position:
            self.get_logger().error('❌ No target position for approach!')
            return
        
        self.get_logger().info('='*50)
        self.get_logger().info('🚁 STARTING APPROACH SEQUENCE')
        
        # Re-assert priority at start of approach
        self.get_logger().info('🎯 Re-asserting autonomous priority')
        self.priority_pub.publish(String(data='autonomous'))
        time.sleep(0.1)  # Brief pause for priority
        
        # Calculate approach position (10m from target at REDUCED altitude)
        approach_distance = 10.0  # Approach to 10m before centering
        
        dx = self.target_position['x'] - self.current_position['x']
        dy = self.target_position['y'] - self.current_position['y']
        current_distance = math.sqrt(dx**2 + dy**2)
        
        self.get_logger().info(f'📏 Current distance to target: {current_distance:.1f}m')
        
        if current_distance > approach_distance + 2.0:  # Add buffer to avoid overshooting
            # Calculate approach point
            ratio = approach_distance / current_distance
            approach_x = self.current_position['x'] + dx * ratio
            approach_y = self.current_position['y'] + dy * ratio
            # REDUCE altitude during approach for better view (but maintain safe minimum)
            approach_z = max(5.0, min(10.0, self.current_position['z']))  # Reduce to 10m max, 5m min
            
            # STORE APPROACH POSITION FOR TRACKING!
            self.approach_position = {
                'x': approach_x,
                'y': approach_y,
                'z': approach_z
            }
            
            self.get_logger().info(f'🎯 Target approach distance: {approach_distance}m')
            self.get_logger().info(f'📍 Approach waypoint: ({approach_x:.1f}, {approach_y:.1f}, {approach_z:.1f})')
            
            # Send approach command
            cmd = f"goto:{approach_x:.2f},{approach_y:.2f},{approach_z:.2f}"
            
            self.get_logger().info(f'📤 Publishing command: {cmd}')
            self.get_logger().info(f'   To topic: /autonomous/position_command')
            
            # Send command multiple times to ensure it's received
            for i in range(2):
                self.position_pub.publish(String(data=cmd))
                time.sleep(0.05)
            
            self.approach_command_time = time.time()
            self.approach_command_count = 1
            
            self.get_logger().info('='*50)
            
        else:
            # Already close enough, start SAR behavior
            self.get_logger().info(f'Already close enough ({current_distance:.1f}m), starting orbit')
            self.state = MissionState.TARGET_FOUND
            self.start_sar_behavior()
    
    def center_target(self):
        """V6.5: Center target in camera view using controlled yaw rotation"""
        self.centering_attempts += 1
        
        # Get latest detection to update target pixel position
        if self.latest_detection:
            try:
                # Handle both dict and list formats
                if isinstance(self.latest_detection, dict):
                    detection = self.latest_detection
                elif isinstance(self.latest_detection, list) and len(self.latest_detection) > 0:
                    detection = self.latest_detection[0]
                else:
                    detection = None
                    
                if detection:
                    bbox = detection.get('bbox', {})
                    self.target_pixel_x = bbox.get('x', 0.5) * 1280  # Convert normalized to pixels
                    self.get_logger().debug(f'Current target pixel: {self.target_pixel_x:.0f}')
            except Exception as e:
                self.get_logger().error(f'Error updating target pixel: {e}')
        
        # Calculate pixel error
        pixel_error = self.target_pixel_x - self.image_center_x
        
        if abs(pixel_error) < self.centering_tolerance:
            # Centered successfully
            self.get_logger().info('✅ Target centered in view')
            self.state = MissionState.LOGGING
            return
        
        # Check if max attempts reached
        if self.centering_attempts >= self.max_centering_attempts:
            self.get_logger().warning('Failed to center after max attempts, logging anyway')
            self.state = MissionState.LOGGING
            return
        
        # Check if we have fresh detection data
        if time.time() - self.last_detection_update > 2.0:
            self.get_logger().warning('No fresh detection for 2s, attempting small yaw to find target')
            # Make a small yaw to try to find the target again
            yaw_adjustment_rad = 0.1 if self.centering_attempts % 2 == 0 else -0.1
        else:
            # Calculate yaw angle adjustment based on pixel error
            # Assuming ~80° FOV horizontally, each pixel represents ~0.0625 degrees (80/1280)
            degrees_per_pixel = 80.0 / 1280.0
            
            # IMPORTANT FIX: NEGATE the adjustment!
            # When target is on RIGHT (positive pixel error), we need to yaw RIGHT (negative rotation)
            # When target is on LEFT (negative pixel error), we need to yaw LEFT (positive rotation)
            yaw_adjustment_deg = -pixel_error * degrees_per_pixel * 0.3  # Reduced gain from 0.5 to 0.3 for smoother control
            
            # Limit maximum adjustment per attempt to prevent overshooting
            max_adjustment_deg = 10.0  # Max 10 degrees per adjustment
            yaw_adjustment_deg = max(-max_adjustment_deg, min(max_adjustment_deg, yaw_adjustment_deg))
            
            yaw_adjustment_rad = math.radians(yaw_adjustment_deg)
            
            direction = "RIGHT" if yaw_adjustment_rad < 0 else "LEFT"
            self.get_logger().info(f'📐 Centering {self.centering_attempts}: target at pixel {self.target_pixel_x:.0f} (error={pixel_error:.0f}), yawing {direction} by {abs(math.degrees(yaw_adjustment_rad)):.1f}°')
        
        # Send yaw command using JSON format for controlled rotation
        yaw_cmd = json.dumps({
            'command': 'yaw_relative',
            'angle': yaw_adjustment_rad
        })
        
        # Only publish to autonomous position command - router will handle it
        self.position_pub.publish(String(data=yaw_cmd))  # To /autonomous/position_command
        self.get_logger().debug(f'Sent yaw command: {yaw_adjustment_rad:.3f} rad')
        
    
    def log_survivor(self):
        """V6.5: Log survivor location to CSV file"""
        if not self.target_position:
            self.get_logger().warning('No target position to log')
            return
            
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        
        # Convert local to GPS (simplified - would need proper conversion in real system)
        gps_lat = 47.641468 + (self.target_position['y'] / 111000.0)  # Rough conversion
        gps_lon = -122.140165 + (self.target_position['x'] / 111000.0)
        
        # Get latest detection confidence
        confidence = 0.9
        try:
            if self.latest_detection:
                if isinstance(self.latest_detection, dict):
                    confidence = self.latest_detection.get('confidence', 0.9)
                elif isinstance(self.latest_detection, list) and len(self.latest_detection) > 0:
                    confidence = self.latest_detection[0].get('confidence', 0.9)
        except Exception as e:
            self.get_logger().debug(f'Could not get confidence: {e}')
        
        # Calculate distance
        dx = self.target_position['x'] - self.current_position['x']
        dy = self.target_position['y'] - self.current_position['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        survivor_data = {
            'timestamp': timestamp,
            'gps_lat': gps_lat,
            'gps_lon': gps_lon,
            'local_x': self.target_position['x'],
            'local_y': self.target_position['y'],
            'altitude': self.target_position['z'],
            'confidence': confidence,
            'distance': distance
        }
        
        self.survivor_log.append(survivor_data)
        
        # Write to CSV
        try:
            with open(self.log_file, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    survivor_data['timestamp'],
                    f"{survivor_data['gps_lat']:.6f}",
                    f"{survivor_data['gps_lon']:.6f}",
                    f"{survivor_data['local_x']:.2f}",
                    f"{survivor_data['local_y']:.2f}",
                    f"{survivor_data['altitude']:.2f}",
                    f"{survivor_data['confidence']:.2f}",
                    f"{survivor_data['distance']:.1f}"
                ])
            
            # Position already added in confirm_target() to prevent duplicates
            # Just log that we've completed the full sequence for this survivor
            # (No need to add again as it was added when confirmed)
            
            self.get_logger().info(
                f"📍 SURVIVOR #{len(self.survivor_log)} LOGGED: GPS({gps_lat:.6f}, {gps_lon:.6f}) "
                f"Distance: {distance:.1f}m"
            )
            self.get_logger().info(f"   Total unique survivors tracked: {len(self.survivor_positions)}")
            self.get_logger().info(f"   (Position was marked when confirmed to prevent re-detection)")
            self.publish_status(f'Survivor #{len(self.survivor_log)} logged at {distance:.1f}m')
        except Exception as e:
            self.get_logger().error(f'Failed to write to log: {e}')
    
    # V6.5: Orbit methods no longer used - keeping for reference only
    def generate_orbit_waypoints(self):
        """DEPRECATED in V6.5 - Using centering/logging instead"""
        return  # Do nothing
        
    def OLD_generate_orbit_waypoints(self):
        """OLD CODE - Not used in V6.5"""
        if not self.target_position:
            return
        
        orbit_waypoints = []
        target_z = float(self.target_position.get('z', 0))
        # Ensure minimum safe altitude for orbiting (at least 5m)
        self.orbit_altitude = max(5.0, target_z + 3.0)  # 3m above target, min 5m altitude
        
        # Generate waypoints in a circle
        total_waypoints = self.waypoints_per_orbit * self.max_orbits
        
        for i in range(total_waypoints):
            angle = (2 * math.pi * i) / self.waypoints_per_orbit
            x = float(self.target_position['x'] + self.orbit_radius * math.cos(angle))
            y = float(self.target_position['y'] + self.orbit_radius * math.sin(angle))
            z = float(self.orbit_altitude)
            
            self.orbit_waypoints.append({'x': x, 'y': y, 'z': z})
        
        self.get_logger().info(f'Generated {len(self.orbit_waypoints)} orbit waypoints')
    
    def start_sar_behavior(self):
        """Start the SAR behavior sequence"""
        if not self.target_position:
            self.get_logger().error('No target position for SAR behavior!')
            return
        
        # Keep pattern paused (not stopped) to preserve state
        self.get_logger().info('Keeping pattern paused during SAR behavior...')
        self.pattern_ctrl_pub.publish(String(data='pause'))
        time.sleep(0.2)  # Give time for pause to propagate
        
        # Set priority to autonomous
        self.get_logger().info('🎯 Setting priority to autonomous for SAR behavior')
        priority_msg = String()
        priority_msg.data = 'autonomous'
        self.priority_pub.publish(priority_msg)
        time.sleep(0.5)  # Increased delay for priority to propagate
        
        self.get_logger().info('='*50)
        self.get_logger().info('🚁 V6.5: STARTING SIMPLIFIED SAR BEHAVIOR')
        self.get_logger().info('  1. Center target in camera view')
        self.get_logger().info('  2. Log GPS coordinates to CSV')
        self.get_logger().info('  3. Resume search pattern')
        self.get_logger().info('='*50)
        
        # V6.5: No orbit generation or waypoints needed
        # State transitions handled in control loop
        self.publish_status('SAR behavior ready')
    
    def send_next_waypoint(self):
        """Send the next orbit waypoint"""
        if self.orbit_index < len(self.orbit_waypoints):
            waypoint = self.orbit_waypoints[self.orbit_index]
            cmd = f"goto:{waypoint['x']:.2f},{waypoint['y']:.2f},{waypoint['z']:.2f}"
            
            # Log current position for debugging
            self.get_logger().info(f'Current pos: ({self.current_position["x"]:.2f}, {self.current_position["y"]:.2f}, {self.current_position["z"]:.2f})')
            self.get_logger().info(f'Sending waypoint {self.orbit_index+1}/{len(self.orbit_waypoints)}: {cmd}')
            
            self.position_pub.publish(String(data=cmd))
            self.last_waypoint_time = time.time()
            
            # Track orbit progress
            orbits_completed = self.orbit_index // self.waypoints_per_orbit
            if orbits_completed > self.orbit_count:
                self.orbit_count = orbits_completed
                self.get_logger().info(f'✓ Completed orbit {self.orbit_count}/{self.max_orbits}')
                self.publish_status(f'Orbiting target ({self.orbit_count}/{self.max_orbits})')
        else:
            # Orbiting complete, move to hover
            self.get_logger().info('✓ All orbits complete, moving to hover position')
            self.start_hover_inspection()
    
    def start_hover_inspection(self):
        """Start hovering for close inspection"""
        # Calculate hover position
        dx = float(self.current_position['x'] - self.target_position['x'])
        dy = float(self.current_position['y'] - self.target_position['y'])
        angle = float(math.atan2(dy, dx))
        
        hover_x = float(self.target_position['x'] + self.hover_distance * math.cos(angle))
        hover_y = float(self.target_position['y'] + self.hover_distance * math.sin(angle))
        hover_z = float(self.target_position['z'] + 1)  # 1m above target
        
        # Send hover position
        cmd = f"goto:{hover_x:.2f},{hover_y:.2f},{hover_z:.2f}"
        self.position_pub.publish(String(data=cmd))
        
        self.state = MissionState.HOVERING_INSPECTION
        self.hover_start_time = time.time()
        self.publish_status('Hovering for inspection')
    
    def control_loop(self):
        """Main control loop"""
        rate = 10  # Hz
        
        while rclpy.ok():
            try:
                if self.state == MissionState.SEARCHING:
                    # Pattern search
                    if not self.pattern_started and self.mission:
                        self.start_search()
                        self.pattern_started = True
                    # Note: Pattern completion is handled by pattern executor
                    # Don't auto-complete here as pattern_active may briefly go false between waypoints
                
                elif self.state == MissionState.CONFIRMING:
                    # NO POSITION COMMANDS - Keep-alive node maintains OFFBOARD mode
                    # PX4 will hold position internally once we sent the initial hold command
                    current_time = time.time()
                    
                    # Just log status periodically, no position commands needed
                    if hasattr(self, 'hold_position'):
                        dx = self.current_position['x'] - self.hold_position['x']
                        dy = self.current_position['y'] - self.hold_position['y']
                        dz = self.current_position['z'] - self.hold_position['z']
                        distance_from_hold = math.sqrt(dx**2 + dy**2 + dz**2)
                        
                        # Log drift for monitoring, but don't send commands
                        if int(current_time) % 5 == 0:  # Log every 5 seconds
                            if distance_from_hold > 2.0:
                                self.get_logger().info(f'⚠️ Drift detected: {distance_from_hold:.1f}m from hold position (no correction sent - relying on PX4 hold)')
                            else:
                                self.get_logger().debug(f'✅ Position stable: {distance_from_hold:.1f}m from hold position')
                    
                    # Check for confirmation timeout
                    if self.confirmation_start_time:
                        elapsed = time.time() - self.confirmation_start_time
                        if elapsed > self.confirmation_timeout:
                            self.get_logger().warn('⏱️ Confirmation timeout - resuming search')
                            self.reset_confirmation()
                    
                    # Check for detection gap timeout
                    if time.time() - self.last_detection_time > self.max_detection_gap:
                        self.get_logger().warn('⏱️ No detection for too long - resetting')
                        self.reset_confirmation()
                
                elif self.state == MissionState.PREPARING:
                    # Wait for hold time before transitioning to approach
                    if self.confirmation_complete_time:
                        elapsed = time.time() - self.confirmation_complete_time
                        remaining = self.confirmation_hold_time - elapsed
                        
                        if remaining > 0:
                            # NO POSITION COMMANDS during preparation - rely on PX4 internal hold
                            current_time = time.time()
                            
                            # Update status periodically
                            if not hasattr(self, 'last_status_update'):
                                self.last_status_update = 0
                            
                            if current_time - self.last_status_update >= 0.5:  # Update every 0.5 seconds
                                self.publish_status(f'🚁 Preparing... {remaining:.1f}s')
                                self.get_logger().info(f'⏳ Holding position: {remaining:.1f}s remaining')
                                self.last_status_update = current_time
                        else:
                            # Hold complete - start approach
                            self.get_logger().info('✅ Preparation complete - starting approach')
                            self.state = MissionState.APPROACHING
                            self.publish_status('Starting approach to target')
                            self.start_approach_sequence()
                
                elif self.state == MissionState.APPROACHING:
                    # Monitor approach progress
                    if self.approach_position:
                        # Check distance to APPROACH position, not target!
                        dx_approach = self.approach_position['x'] - self.current_position['x']
                        dy_approach = self.approach_position['y'] - self.current_position['y']
                        dist_to_approach = math.sqrt(dx_approach**2 + dy_approach**2)
                        
                        # Also check distance to actual target
                        dx_target = self.target_position['x'] - self.current_position['x']
                        dy_target = self.target_position['y'] - self.current_position['y']
                        dist_to_target = math.sqrt(dx_target**2 + dy_target**2)
                        
                        # Log progress every second
                        if int(time.time()) % 2 == 0:
                            self.get_logger().info(f'Approach progress: {dist_to_approach:.1f}m to waypoint, {dist_to_target:.1f}m to target')
                        
                        # Check if approach position reached OR close enough to target
                        if dist_to_approach < 3.0 or dist_to_target <= 12.0:
                            self.get_logger().info('✅ Approach complete - starting centering')
                            
                            # Store original yaw before centering
                            self.original_yaw = self.current_yaw
                            self.get_logger().info(f'📐 Storing original yaw: {math.degrees(self.original_yaw):.1f}°')
                            
                            self.state = MissionState.CENTERING
                            self.centering_attempts = 0
                            
                            # Ensure autonomous priority for centering
                            self.priority_pub.publish(String(data='autonomous'))
                            self.approach_position = None  # Clear approach position
                            self.start_sar_behavior()
                        
                        # Resend command if stuck (no progress after 5 seconds)
                        elif time.time() - self.approach_command_time > 5.0:
                            if self.approach_command_count < 3:  # Max 3 attempts
                                self.get_logger().warn(f'⚠️ No progress detected after 5s')
                                self.get_logger().warn(f'   Current pos: ({self.current_position["x"]:.1f}, {self.current_position["y"]:.1f})')
                                self.get_logger().warn(f'   Approach pos: ({self.approach_position["x"]:.1f}, {self.approach_position["y"]:.1f})')
                                self.get_logger().warn(f'   Distance remaining: {dist_to_approach:.1f}m')
                                self.get_logger().warn(f'📤 Resending approach command (attempt {self.approach_command_count + 1}/3)')
                                
                                cmd = f"goto:{self.approach_position['x']:.2f},{self.approach_position['y']:.2f},{self.approach_position['z']:.2f}"
                                self.position_pub.publish(String(data=cmd))
                                
                                # Re-assert priority
                                self.priority_pub.publish(String(data='autonomous'))
                                
                                self.approach_command_time = time.time()
                                self.approach_command_count += 1
                            else:
                                # Too many retries, skip approach and go straight to orbit
                                self.get_logger().warn('Approach failed after 3 attempts, starting orbit anyway')
                                self.state = MissionState.TARGET_FOUND
                                self.start_sar_behavior()
                    else:
                        # No approach position set, go straight to orbit
                        self.get_logger().warn('No approach position, transitioning to orbit')
                        self.state = MissionState.TARGET_FOUND
                        self.start_sar_behavior()
                
                elif self.state == MissionState.CENTERING:
                    # Center target in camera view
                    # Add delay between centering attempts for smoother control
                    if not hasattr(self, 'last_center_time'):
                        self.last_center_time = 0
                    
                    if time.time() - self.last_center_time >= 1.0:  # Increased to 1.0s between attempts to allow yaw to settle
                        self.center_target()
                        self.last_center_time = time.time()
                
                elif self.state == MissionState.LOGGING:
                    # Log survivor and wait briefly
                    if self.logging_start_time is None:
                        self.log_survivor()
                        self.logging_start_time = time.time()
                    
                    elapsed = time.time() - self.logging_start_time
                    if elapsed >= self.logging_duration:
                        self.get_logger().info('✅ Survivor logged - returning to original yaw')
                        self.state = MissionState.RETURNING_TO_YAW
                        self.logging_start_time = None
                        self.yaw_restored = False
                        self.yaw_return_attempts = 0
                        self.publish_status(f'Returning to original heading')
                
                elif self.state == MissionState.RETURNING_TO_YAW:
                    # V6.5: Return to original yaw before returning to altitude
                    if not self.yaw_restored and self.original_yaw is not None:
                        current_yaw_deg = math.degrees(self.current_yaw)
                        original_yaw_deg = math.degrees(self.original_yaw)
                        
                        # Calculate yaw error
                        yaw_error = self.original_yaw - self.current_yaw
                        
                        # Normalize to [-pi, pi]
                        while yaw_error > math.pi:
                            yaw_error -= 2 * math.pi
                        while yaw_error < -math.pi:
                            yaw_error += 2 * math.pi
                        
                        if abs(yaw_error) > math.radians(5):  # More than 5 degrees off
                            # Send yaw command to return to original heading
                            yaw_cmd = json.dumps({
                                'command': 'yaw_to',
                                'angle': self.original_yaw
                            })
                            
                            # Only publish to autonomous position command
                            self.position_pub.publish(String(data=yaw_cmd))
                            
                            if self.yaw_return_attempts % 10 == 0:  # Log every 10 attempts (1 second)
                                self.get_logger().info(f'🧭 Returning to original yaw: {current_yaw_deg:.1f}° → {original_yaw_deg:.1f}° (error: {math.degrees(yaw_error):.1f}°)')
                            
                            self.yaw_return_attempts += 1
                            
                            # Timeout after 5 seconds
                            if self.yaw_return_attempts > 50:
                                self.get_logger().warning('Yaw return timeout, proceeding anyway')
                                self.yaw_restored = True
                        else:
                            self.yaw_restored = True
                            self.get_logger().info(f'✅ Returned to original yaw: {original_yaw_deg:.1f}°')
                    
                    if self.yaw_restored or self.original_yaw is None:
                        # Now return to altitude
                        self.get_logger().info('📏 Now returning to search altitude')
                        self.state = MissionState.RETURNING_TO_ALTITUDE
                        self.altitude_reached = False
                        self.publish_status(f'Returning to {self.search_altitude:.1f}m altitude')
                
                elif self.state == MissionState.RETURNING_TO_ALTITUDE:
                    # V6.5: Return to search altitude AND move away before resuming
                    if not self.altitude_reached and self.search_altitude:
                        current_alt = self.current_position.get('z', 0) if self.current_position else 0
                        altitude_error = abs(current_alt - self.search_altitude)
                        
                        if altitude_error > 1.0:  # More than 1m difference
                            # Move away from survivor area while climbing
                            # Calculate movement away from last survivor position
                            if self.target_position:
                                dx = self.current_position['x'] - self.target_position['x']
                                dy = self.current_position['y'] - self.target_position['y']
                                dist = math.sqrt(dx**2 + dy**2)
                                
                                # If too close, move 30m away before resuming
                                if dist < 30.0:
                                    # Move in opposite direction from survivor
                                    if dist > 0:
                                        move_x = self.current_position['x'] + (dx/dist) * 30
                                        move_y = self.current_position['y'] + (dy/dist) * 30
                                    else:
                                        # If exactly on top, move forward
                                        move_x = self.current_position['x'] + 30 * math.cos(self.current_yaw)
                                        move_y = self.current_position['y'] + 30 * math.sin(self.current_yaw)
                                    
                                    cmd = f"goto:{move_x:.2f},{move_y:.2f},{self.search_altitude:.2f}"
                                    if int(time.time()) % 2 == 0:
                                        self.get_logger().info(f'Moving away from survivor area and climbing: {dist:.1f}m → 30m')
                                else:
                                    # Just climb if already far enough
                                    cmd = f"goto:{self.current_position['x']:.2f},{self.current_position['y']:.2f},{self.search_altitude:.2f}"
                                    if int(time.time()) % 2 == 0:
                                        self.get_logger().info(f'Returning to altitude: {current_alt:.1f}m → {self.search_altitude:.1f}m')
                            else:
                                # No target position, just climb
                                cmd = f"goto:{self.current_position['x']:.2f},{self.current_position['y']:.2f},{self.search_altitude:.2f}"
                            
                            self.position_pub.publish(String(data=cmd))
                        else:
                            self.altitude_reached = True
                            self.get_logger().info(f'✅ Reached search altitude: {self.search_altitude:.1f}m')
                    
                    if self.altitude_reached:
                        # Add a brief cooldown before resuming search to clear the area
                        if not hasattr(self, 'clear_area_time') or self.clear_area_time is None:
                            self.clear_area_time = time.time()
                            self.get_logger().info('⏱️ Waiting 3s before resuming search...')
                        
                        if self.clear_area_time and time.time() - self.clear_area_time >= 3.0:
                            # Now resume the search pattern FROM WHERE IT LEFT OFF
                            self.get_logger().info('📍 Resuming search pattern from last waypoint')
                            self.state = MissionState.SEARCHING
                            self.clear_area_time = None  # Reset for next time
                            self.last_sar_completion_time = time.time()  # Start detection cooldown
                        
                        # Set priority back to pattern focus for searching
                        self.priority_pub.publish(String(data='pattern_focus'))
                        time.sleep(0.1)  # Give priority time to switch
                        
                        # Clear any position holds before resuming
                        self.position_pub.publish(String(data='stop'))
                        time.sleep(0.1)
                        
                        # Resume pattern from where it was paused (not restart!)
                        self.pattern_ctrl_pub.publish(String(data='resume'))
                        self.pattern_active = True
                        self.publish_status('Search pattern resumed from last position')
                        
                        # Reset for next detection
                        self.search_altitude = None
                        self.original_yaw = None  # Reset original yaw tracking
                        self.target_position = None
                        self.latest_detection = None
                        self.centering_attempts = 0
                        # Note: clear_area_time already reset in RETURNING_TO_ALTITUDE state
                
                elif self.state == MissionState.HOVERING_INSPECTION:
                    # Check hover duration
                    if self.hover_start_time:
                        elapsed = time.time() - self.hover_start_time
                        remaining = self.hover_duration - elapsed
                        
                        if remaining > 0:
                            self.publish_status(f'Inspection hover ({remaining:.1f}s remaining)')
                        else:
                            self.complete_mission()
                
                time.sleep(1.0 / rate)
                
            except Exception as e:
                import traceback
                self.get_logger().error(f'Control loop error: {str(e)}')
                self.get_logger().debug(f'Traceback: {traceback.format_exc()}')
                time.sleep(1)
    
    def check_waypoint_reached(self):
        """Check if current waypoint is reached"""
        if self.orbit_index >= len(self.orbit_waypoints):
            # All waypoints done, transition to hover
            if self.state == MissionState.ORBITING:
                self.get_logger().info('All orbit waypoints complete, starting hover')
                self.start_hover_inspection()
            return
        
        waypoint = self.orbit_waypoints[self.orbit_index]
        
        # Calculate distance to waypoint
        dist = math.sqrt(
            (self.current_position['x'] - waypoint['x'])**2 +
            (self.current_position['y'] - waypoint['y'])**2 +
            (self.current_position['z'] - waypoint['z'])**2
        )
        
        # Check if reached or timeout
        elapsed = time.time() - self.last_waypoint_time
        
        if dist < self.waypoint_threshold or elapsed > self.waypoint_timeout:
            if dist < self.waypoint_threshold:
                self.get_logger().debug(f'Waypoint {self.orbit_index} reached (dist: {dist:.2f}m)')
            else:
                self.get_logger().warn(f'Waypoint {self.orbit_index} timeout, moving to next')
            
            # Move to next waypoint
            self.orbit_index += 1
            self.get_logger().info(f'Waypoint reached, moving to {self.orbit_index+1}/{len(self.orbit_waypoints)}')
            self.send_next_waypoint()
    
    def complete_mission(self):
        """Complete the mission"""
        self.get_logger().info('='*50)
        self.get_logger().info('✅ MISSION COMPLETE!')
        self.get_logger().info(f'   Survivors found: {len(self.survivor_positions)}')
        self.get_logger().info(f'   Log file: {self.log_file}')
        self.get_logger().info('='*50)
        
        self.state = MissionState.MISSION_COMPLETE
        self.publish_status(f'Mission complete - {len(self.survivor_positions)} survivors found')
        
        # Stop any active pattern
        if self.pattern_active:
            self.pattern_ctrl_pub.publish(String(data='stop'))
            self.pattern_active = False
            self.pattern_started = False
        
        # Send stop command to position controller
        self.position_pub.publish(String(data='stop'))
        
        # Reset priority
        self.get_logger().info('Resetting priority to manual_priority')
        self.priority_pub.publish(String(data='manual_priority'))
        
        # Clear mission-specific variables (but keep survivor positions)
        self.mission = None
        self.target_position = None
        self.latest_detection = None
        
        self.get_logger().info('Ready for next mission command.')
    
    def start_search(self):
        """Start search pattern (only called at mission start)"""
        pattern = self.mission.get('search_pattern', 'expanding_square')
        size = self.mission.get('search_size', 30)
        iterations = self.mission.get('search_iterations', 3)  # Get iterations from mission
        
        if pattern == 'expanding_square':
            cmd = f'expanding_square:{size},{iterations}'
        elif pattern == 'spiral':
            # Spiral uses radius and spacing
            spacing = 5  # Default 5m spacing between spiral lines
            cmd = f'spiral:{size},{spacing}'
        elif pattern == 'zigzag':
            # Zigzag uses width, length, and spacing (not iterations!)
            spacing = 5  # Default 5m spacing between zigzag lines
            # For zigzag, we can use size for both width and length
            cmd = f'zigzag:{size},{size},{spacing}'
        else:
            # Default to expanding square
            cmd = f'expanding_square:{size},{iterations}'
        
        # Set priority for pattern execution FIRST
        self.get_logger().info('🎯 Setting priority to pattern_focus for search')
        self.priority_pub.publish(String(data='pattern_focus'))
        time.sleep(0.2)  # Give time for priority to take effect
        
        # Clear any previous position holds (only on initial start)
        self.position_pub.publish(String(data='stop'))
        time.sleep(0.1)
        
        self.get_logger().info(f'Starting NEW search pattern: {cmd}')
        self.pattern_pub.publish(String(data=cmd))
        time.sleep(0.5)
        self.pattern_ctrl_pub.publish(String(data='start'))
        self.pattern_active = True
        self.pattern_start_time = time.time()  # Record when pattern started
        self.get_logger().info('✅ Pattern started - will pause/resume to preserve state')
    
    def parse_mission(self, msg):
        """Parse mission command"""
        if self.state not in [MissionState.IDLE, MissionState.MISSION_COMPLETE, MissionState.ABORTED]:
            self.get_logger().warn(f'Cannot start new mission - current state: {self.state.value}')
            self.get_logger().warn('Please wait for current mission to complete or send abort command')
            return
        
        # Clean up any previous mission state
        if self.state == MissionState.MISSION_COMPLETE or self.state == MissionState.ABORTED:
            self.get_logger().info('🔄 Cleaning up previous mission state...')
            # Stop any existing pattern
            self.pattern_ctrl_pub.publish(String(data='stop'))
            time.sleep(0.1)
            # Clear all state variables
            self.pattern_started = False
            self.pattern_active = False
            self.detection_count = 0
            self.detection_positions = []
            self.target_position = None
            self.latest_detection = None
            self.centering_attempts = 0
            # Note: Keep survivor_positions to avoid re-detecting same survivors
        
        mission_text = msg.data
        self.get_logger().info(f'📋 New Mission: "{mission_text}"')
        
        # Enhanced parsing to extract pattern type and size
        mission_lower = mission_text.lower()
        
        # Determine pattern type
        if 'spiral' in mission_lower:
            pattern = 'spiral'
        elif 'zigzag' in mission_lower or 'zig-zag' in mission_lower:
            pattern = 'zigzag'
        elif 'expanding' in mission_lower or 'square' in mission_lower:
            pattern = 'expanding_square'
        else:
            pattern = 'expanding_square'  # default
        
        # Extract size and iterations from mission text
        import re
        size = 30  # default size
        iterations = 3  # default iterations
        
        # Look for size patterns like "20 meter", "20m", "20-meter", "size 20", etc.
        size_patterns = [
            r'(\d+)\s*(?:meter|metre|m)\b',  # "20 meter", "20m"
            r'size\s*(?:of\s*)?(\d+)',        # "size 20", "size of 20"
            r'(\d+)\s*(?:by|x)\s*\d+',        # "20x20", "20 by 20"
            r'area\s*(?:of\s*)?(\d+)',        # "area of 20"
        ]
        
        for pattern_regex in size_patterns:
            match = re.search(pattern_regex, mission_lower)
            if match:
                try:
                    size = int(match.group(1))
                    self.get_logger().info(f'📏 Extracted size: {size}m from pattern "{match.group(0)}"')
                    break
                except:
                    pass
        
        # Look for iteration patterns like "4 iterations", "5 times", "repeat 3", etc.
        iteration_patterns = [
            r'(\d+)\s*(?:iteration|iterations|loop|loops|time|times)\b',  # "4 iterations", "5 times"
            r'(?:repeat|loop)\s*(\d+)',                                    # "repeat 3", "loop 4"
            r'(\d+)\s*(?:pass|passes|cycle|cycles)\b',                    # "3 passes", "4 cycles"
        ]
        
        for pattern_regex in iteration_patterns:
            match = re.search(pattern_regex, mission_lower)
            if match:
                try:
                    iterations = int(match.group(1))
                    self.get_logger().info(f'🔄 Extracted iterations: {iterations} from pattern "{match.group(0)}"')
                    break
                except:
                    pass
        
        self.mission = {
            'target': 'person',
            'search_pattern': pattern,
            'search_size': size,
            'search_iterations': iterations,
            'description': mission_text
        }
        
        self.get_logger().info(f'📋 Mission parsed:')
        self.get_logger().info(f'   • Pattern: {pattern}')
        self.get_logger().info(f'   • Size: {size}m')
        self.get_logger().info(f'   • Iterations: {iterations}')
        self.get_logger().info(f'   • Target: person')
        
        self.state = MissionState.SEARCHING
        self.pattern_started = False
        self.publish_status('Starting new search mission')
        self.get_logger().info('✅ Mission started - searching for targets')
    
    def handle_mission_control(self, msg):
        """Handle abort/pause/resume/clear"""
        command = msg.data.lower()
        
        if command in ['stop', 'abort']:
            self.get_logger().info('🛑 Aborting mission...')
            self.state = MissionState.ABORTED
            self.pattern_ctrl_pub.publish(String(data='stop'))
            self.position_pub.publish(String(data='stop'))
            self.pattern_started = False
            self.pattern_active = False
            self.mission = None
            self.publish_status('Mission aborted')
            
            # Reset priority
            self.priority_pub.publish(String(data='manual_priority'))
            self.get_logger().info('Mission aborted - ready for new mission')
            
        elif command == 'clear_survivors':
            # Clear survivor history for fresh search
            self.get_logger().info('🔄 Clearing survivor history')
            self.survivor_positions = []
            self.survivor_log = []
            self.publish_status('Survivor history cleared')
            self.get_logger().info(f'Cleared {len(self.survivor_positions)} survivor positions')
    
    def update_position(self, msg):
        """Update current position"""
        self.current_position = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        }
        
        # Calculate yaw
        q = msg.pose.orientation
        self.current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    
    def update_pattern_status(self, msg):
        """Track pattern status"""
        status = msg.data.lower()
        
        # Check if pattern is active
        self.pattern_active = 'executing' in status or 'running' in status or 'paused' in status
        
        # Check if pattern completed (not just paused or between waypoints)
        if 'complete' in status or 'finished' in status:
            # Only complete if we've been searching for minimum duration (avoid false positives)
            if self.state == MissionState.SEARCHING and self.pattern_start_time:
                elapsed = time.time() - self.pattern_start_time
                if elapsed > self.min_pattern_duration:
                    self.get_logger().info(f'📍 Search pattern completed after {elapsed:.1f}s')
                    self.complete_mission()
                else:
                    self.get_logger().warn(f'⚠️ IGNORING early pattern completion!')
                    self.get_logger().warn(f'   Pattern reported complete after only {elapsed:.1f}s')
                    self.get_logger().warn(f'   Minimum duration required: {self.min_pattern_duration}s')
                    self.get_logger().warn(f'   Continuing search...')
        
        # Enhanced debug logging for pattern state changes
        if hasattr(self, '_last_pattern_status') and self._last_pattern_status != status:
            self.get_logger().info(f'📊 Pattern status changed: "{self._last_pattern_status}" → "{status}"')
            
            # Log waypoint info if available
            if 'waypoint' in status.lower():
                self.get_logger().info(f'   Current status: {status}')
                
        self._last_pattern_status = status
    
    def publish_status(self, message):
        """Publish status"""
        status = {
            'state': self.state.value,
            'message': message,
            'target_locked': self.target_position is not None,
            'target_distance': self.target_distance,
            'survivors_found': len(self.survivor_log),  # V6.5: Track survivors instead of orbits
            'unique_survivors': len(self.survivor_positions),  # Track unique positions
            'mission': self.mission.get('description', '') if self.mission else ''
        }
        
        # Add survivor positions for visualization
        if self.survivor_positions:
            status['survivor_locations'] = [
                {'x': pos['x'], 'y': pos['y'], 'z': pos['z']}
                for pos in self.survivor_positions[-5:]  # Last 5 survivors
            ]
        
        self.status_pub.publish(String(data=json.dumps(status)))

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousSARExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()