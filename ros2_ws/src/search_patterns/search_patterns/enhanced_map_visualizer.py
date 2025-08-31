#!/usr/bin/env python3
"""
Enhanced Map Visualizer - Shows drone path AND detected person location
Includes distance estimation and SAR status visualization
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch
from collections import deque
import json
import math
import numpy as np
import time

class EnhancedMapVisualizer(Node):
    def __init__(self):
        super().__init__('enhanced_map_visualizer')
        
        # QoS for MAVROS
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # CV Bridge for depth
        self.bridge = CvBridge()
        
        # Subscribe to drone position
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self.update_position,
            qos_profile=self.mavros_qos
        )
        
        # Subscribe to pattern waypoints
        self.waypoints_sub = self.create_subscription(
            String, '/pattern_waypoints',
            self.update_waypoints, 10
        )
        
        # Subscribe to pattern status to detect completion
        self.pattern_status_sub = self.create_subscription(
            String, '/pattern_status',
            self.update_pattern_status, 10
        )
        
        # Subscribe to YOLO detections for person tracking
        self.detection_sub = self.create_subscription(
            String, '/yolo/detections',
            self.handle_detections, 10
        )
        
        # Subscribe to SAR status
        self.sar_status_sub = self.create_subscription(
            String, '/autonomous/status',
            self.update_sar_status, 10
        )
        
        # Subscribe to depth for distance visualization
        self.depth_sub = self.create_subscription(
            Image, '/airsim_node/PX4/depth_camera_DepthPerspective/image',
            self.handle_depth, 10
        )
        
        # Data storage
        self.path_history = deque(maxlen=1000)  # Longer trail
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0
        self.waypoints = []
        self.first_position_received = False
        self.start_position = None
        
        # Person detection data
        self.person_detected = False
        self.person_position = None
        self.person_distance = None
        self.person_confidence = 0.0
        self.detection_history = deque(maxlen=50)  # Trail of detections
        self.depth_image = None
        
        # SAR status
        self.sar_state = "idle"
        self.sar_message = ""
        self.orbit_waypoints = []
        
        # Plot will be created in main thread
        self.fig = None
        self.ax = None
        self.info_ax = None  # For status panel
        
        self.get_logger().info('🗺️ Enhanced Map Visualizer started - Tracking drone and persons!')
    
    def setup_plot(self):
        """Setup the plot with grid and info panel"""
        # Set dark theme for professional look
        self.ax.set_facecolor('#1e1e1e')
        self.fig.patch.set_facecolor('#2b2b2b')
        
        # Main plot with enhanced styling
        self.ax.set_xlabel('X Position (meters)', fontsize=11, color='#e0e0e0', fontweight='medium')
        self.ax.set_ylabel('Y Position (meters)', fontsize=11, color='#e0e0e0', fontweight='medium')
        self.ax.set_title('🚁 SAR DRONE OPERATIONS CENTER', fontsize=16, fontweight='bold', 
                         color='#00ff41', pad=20, family='monospace')
        
        # Enhanced grid with major/minor lines
        self.ax.grid(True, which='major', linestyle='-', alpha=0.2, color='#4a4a4a', linewidth=0.8)
        self.ax.grid(True, which='minor', linestyle=':', alpha=0.1, color='#3a3a3a', linewidth=0.5)
        self.ax.minorticks_on()
        
        # Origin crosshairs
        self.ax.axhline(y=0, color='#00ff41', linestyle='-', alpha=0.3, linewidth=1.5)
        self.ax.axvline(x=0, color='#00ff41', linestyle='-', alpha=0.3, linewidth=1.5)
        
        # Tick styling
        self.ax.tick_params(colors='#b0b0b0', which='both', labelsize=9)
        self.ax.spines['bottom'].set_color('#4a4a4a')
        self.ax.spines['top'].set_color('#4a4a4a')
        self.ax.spines['left'].set_color('#4a4a4a')
        self.ax.spines['right'].set_color('#4a4a4a')
        
        # Set initial view
        self.ax.set_xlim(-30, 30)
        self.ax.set_ylim(-30, 30)
        self.ax.set_aspect('equal')
        
        # Info panel setup with background
        self.info_ax.axis('off')
        self.info_ax.set_xlim(0, 1)
        self.info_ax.set_ylim(0, 1)
        self.info_ax.set_facecolor('#1a1a1a')
        
        # Add decorative border to info panel
        border = FancyBboxPatch((0.02, 0.02), 0.96, 0.96, 
                                boxstyle="round,pad=0.01",
                                linewidth=2, edgecolor='#00ff41', 
                                facecolor='none', alpha=0.3)
        self.info_ax.add_patch(border)
        
        plt.show()
    
    def handle_depth(self, msg):
        """Store depth image for distance calculation"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        except:
            pass
    
    def update_position(self, msg):
        """Update drone position"""
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        
        # Calculate yaw from quaternion
        q = msg.pose.orientation
        self.current_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        
        # Save start position on first update
        if not self.first_position_received:
            self.start_position = (self.current_x, self.current_y, self.current_z)
            self.first_position_received = True
            self.get_logger().info(f'Started tracking at: X={self.current_x:.2f}, Y={self.current_y:.2f}, Z={self.current_z:.2f}')
            self.path_history.append((self.current_x, self.current_y))
        
        # Add to path history
        self.path_history.append((self.current_x, self.current_y))
    
    def handle_detections(self, msg):
        """Process YOLO detections to extract person location"""
        # Reset person detection at start of each update
        self.person_detected = False
        self.person_position = None
        
        try:
            data = json.loads(msg.data)
            detections = data.get('detections', [])
            
            for detection in detections:
                obj_class = detection.get('class', '').lower()
                confidence = detection.get('confidence', 0)
                
                # Check if it's a person first
                if obj_class == 'person':
                    # Log low confidence detections that are filtered out
                    if confidence < 0.7:
                        self.get_logger().debug(f'Person detected but confidence too low: {confidence:.1%} < 70%')
                        continue
                    
                    # Must have >= 70% confidence to be considered valid
                    if confidence >= 0.7:
                        # Estimate distance first
                        bbox = detection.get('bbox', {})
                        distance = self.estimate_distance(bbox)
                        
                        # Only track person if within 100m radius
                        if distance <= 100.0:
                            self.person_detected = True
                            self.person_confidence = confidence
                            self.person_distance = distance
                            
                            # Calculate world position of person
                            person_world_pos = self.calculate_person_position(bbox, distance)
                            if person_world_pos:
                                # Double-check distance using world coordinates
                                dx = person_world_pos[0] - self.current_x
                                dy = person_world_pos[1] - self.current_y
                                world_distance = math.sqrt(dx**2 + dy**2)
                                
                                if world_distance <= 100.0:  # Verify within 100m
                                    self.person_position = person_world_pos
                                    # Don't append to history - we only want current detection
                                    # self.detection_history.append(person_world_pos)
                                    
                                    self.get_logger().info(f'👤 Person detected: position ({person_world_pos[0]:.1f}, {person_world_pos[1]:.1f}), distance: {distance:.1f}m, confidence: {confidence:.1%}')
                                else:
                                    self.get_logger().debug(f'Person detected but too far: {world_distance:.1f}m > 100m')
                        else:
                            self.get_logger().debug(f'Person detected but too far: {distance:.1f}m > 100m (confidence: {confidence:.1%})')
                        break
                    
        except Exception as e:
            self.get_logger().error(f'Detection processing error: {e}')
    
    def estimate_distance(self, bbox):
        """Estimate distance to person using calibrated parameters"""
        distance_estimates = []
        
        # Method 1: Depth camera (if available)
        if self.depth_image is not None and bbox:
            try:
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
                    distance_estimates.append((depth_value, 3.0))  # Weight 3x
            except:
                pass
        
        # Method 2: Bounding box size (calibrated)
        bbox_height = bbox.get('height', 0)
        if bbox_height > 0:
            human_height = 1.8  # Calibrated
            image_height = 1080
            fov_rad = math.radians(80)  # Calibrated FOV
            
            pixel_height = bbox_height * image_height
            distance_bbox = (human_height * image_height) / (pixel_height * 2 * math.tan(fov_rad/2))
            distance_estimates.append((distance_bbox, 1.0))  # Weight 1x
        
        # Calculate weighted average
        if distance_estimates:
            weighted_sum = sum(dist * weight for dist, weight in distance_estimates)
            weight_sum = sum(weight for _, weight in distance_estimates)
            return weighted_sum / weight_sum
        else:
            return 10.0  # Default
    
    def calculate_person_position(self, bbox, distance):
        """Calculate world position of person from detection"""
        try:
            # Get image-space coordinates (-0.5 to 0.5)
            img_x = bbox.get('x', 0.5) - 0.5
            img_y = bbox.get('y', 0.5) - 0.5
            
            # Camera FOV
            fov_h = math.radians(90)
            fov_v = math.radians(67.5)
            
            # Calculate angles
            angle_h = img_x * fov_h
            angle_v = img_y * fov_v
            
            # Camera pitch (looking slightly down)
            camera_pitch = math.radians(10)
            total_pitch = camera_pitch + angle_v
            
            # Calculate relative position
            forward_distance = distance * math.cos(total_pitch) * math.cos(angle_h)
            lateral_distance = distance * math.cos(total_pitch) * math.sin(angle_h)
            
            # Transform to world coordinates using drone's yaw
            world_x = self.current_x + forward_distance * math.cos(self.current_yaw) - lateral_distance * math.sin(self.current_yaw)
            world_y = self.current_y + forward_distance * math.sin(self.current_yaw) + lateral_distance * math.cos(self.current_yaw)
            
            return (world_x, world_y)
            
        except Exception as e:
            self.get_logger().error(f'Position calculation error: {e}')
            return None
    
    def update_waypoints(self, msg):
        """Update waypoint list"""
        try:
            data = json.loads(msg.data)
            self.waypoints = data.get('waypoints', [])
        except:
            pass
    
    def update_pattern_status(self, msg):
        """Monitor pattern status and clear waypoints when complete"""
        try:
            status = msg.data
            status_lower = status.lower()
            
            # Only clear waypoints when the ENTIRE PATTERN completes
            # Check for "Pattern complete" specifically (capital P)
            if status == "Pattern complete" or 'pattern complete' in status_lower:
                if self.waypoints:  # Only log if we're actually clearing waypoints
                    self.get_logger().info(f'🔄 {status} - clearing waypoints from display')
                    self.waypoints = []
            # Clear on stop/abort commands
            elif 'stopped' in status_lower or 'aborted' in status_lower or 'idle' in status_lower:
                if self.waypoints:
                    self.get_logger().info(f'🛑 Pattern {status} - clearing waypoints')
                    self.waypoints = []
            # Clear when starting a new pattern (but not on waypoint updates)
            elif ('starting' in status_lower or 'initializing' in status_lower) and 'waypoint' not in status_lower:
                if self.waypoints:
                    self.get_logger().info('🆕 New pattern starting - clearing old waypoints')
                    self.waypoints = []
            # Debug log for waypoint progress (don't clear!)
            elif 'waypoint' in status_lower or 'corner' in status_lower:
                self.get_logger().debug(f'Pattern progress: {status}')
                # Don't clear waypoints - just progress update
        except Exception as e:
            self.get_logger().debug(f'Pattern status processing error: {e}')
    
    def update_sar_status(self, msg):
        """Update SAR mission status"""
        try:
            data = json.loads(msg.data)
            self.sar_state = data.get('state', 'idle')
            self.sar_message = data.get('message', '')
            
            # Extract orbit waypoints if available
            orbit_progress = data.get('orbit_progress', '0/0')
            if '/' in orbit_progress:
                current, total = orbit_progress.split('/')
                if int(total) > 0:
                    # SAR is orbiting
                    pass
                    
        except:
            pass
    
    def update_plot(self):
        """Update plot - called from main thread"""
        try:
            # Clear main plot
            self.ax.clear()
            
            # Restore dark background
            self.ax.set_facecolor('#1e1e1e')
            
            # Setup enhanced grid again
            self.ax.grid(True, which='major', linestyle='-', alpha=0.2, color='#4a4a4a', linewidth=0.8)
            self.ax.grid(True, which='minor', linestyle=':', alpha=0.1, color='#3a3a3a', linewidth=0.5)
            self.ax.minorticks_on()
            self.ax.axhline(y=0, color='#00ff41', linestyle='-', alpha=0.3, linewidth=1.5)
            self.ax.axvline(x=0, color='#00ff41', linestyle='-', alpha=0.3, linewidth=1.5)
            
            # Only draw if we have received position data
            if self.first_position_received:
                # Draw start position with glow effect
                if self.start_position:
                    self.ax.plot(self.start_position[0], self.start_position[1], 'D', 
                               markersize=12, color='#00ff41', markeredgecolor='white',
                               markeredgewidth=2, label='START', zorder=5)
                    # Add glow effect
                    for size, alpha in [(20, 0.1), (16, 0.2)]:
                        self.ax.plot(self.start_position[0], self.start_position[1], 'o',
                                   markersize=size, color='#00ff41', alpha=alpha, zorder=4)
                
                # Draw path history with gradient effect
                if len(self.path_history) > 1:
                    path_x = [p[0] for p in self.path_history]
                    path_y = [p[1] for p in self.path_history]
                    # Draw with fading trail effect
                    for i in range(1, len(path_x)):
                        alpha = 0.3 + (i / len(path_x)) * 0.5
                        self.ax.plot(path_x[i-1:i+1], path_y[i-1:i+1], 
                                   color='#00bfff', linewidth=2.5, alpha=alpha, solid_capstyle='round')
                    # Add label for legend
                    self.ax.plot([], [], color='#00bfff', linewidth=2.5, label='Flight Path')
                
                # Draw waypoints with better styling (only if active pattern)
                # Changed to ORANGE/YELLOW to contrast with blue drone path
                if self.waypoints:
                    wp_x = [w['x'] for w in self.waypoints]
                    wp_y = [w['y'] for w in self.waypoints]
                    # Use orange/amber color for planned path (#ff9500 or #ffa500)
                    self.ax.plot(wp_x, wp_y, '--', color='#ff9500', alpha=0.5, linewidth=1.5, label='Planned Pattern')
                    # Waypoint markers with numbers - also orange
                    for i, (x, y) in enumerate(zip(wp_x, wp_y)):
                        if i % 5 == 0:  # Show every 5th waypoint number
                            self.ax.plot(x, y, 's', markersize=5, color='#ff9500', alpha=0.7)
                            self.ax.text(x, y, str(i), fontsize=7, color='#ffb833', 
                                       ha='center', va='bottom', alpha=0.8)
                
                # Removed detection history trail - only show current detection
                # This keeps the display clean and only shows active detections
                
                # Draw current person position with pulse effect
                if self.person_position and self.person_detected:
                    # Animated pulse rings
                    for radius, alpha in [(5, 0.15), (3, 0.25), (1.5, 0.35)]:
                        pulse = Circle(self.person_position, radius, 
                                     color='#ff0040', fill=False, alpha=alpha, linewidth=2)
                        self.ax.add_patch(pulse)
                    
                    # Person marker with warning icon style
                    self.ax.plot(self.person_position[0], self.person_position[1], 
                               marker='*', markersize=22, color='#ff0040', 
                               markeredgecolor='#ffff00', markeredgewidth=2,
                               label=f'PERSON [{self.person_confidence:.0%} conf]', 
                               zorder=10)
                    
                    # Detection uncertainty zone
                    if self.person_distance:
                        uncertainty = max(2, self.person_distance * 0.1)
                        circle = Circle(self.person_position, uncertainty, 
                                      color='#ff6666', fill=True, alpha=0.1)
                        self.ax.add_patch(circle)
                        circle2 = Circle(self.person_position, uncertainty, 
                                       color='#ff0040', fill=False, alpha=0.4, 
                                       linestyle='--', linewidth=1.5)
                        self.ax.add_patch(circle2)
                    
                    # Enhanced line from drone to person
                    self.ax.plot([self.current_x, self.person_position[0]], 
                               [self.current_y, self.person_position[1]], 
                               color='#ff6666', linestyle=':', alpha=0.7, linewidth=2)
                    
                    # Distance label with better styling
                    if self.person_distance:
                        mid_x = (self.current_x + self.person_position[0]) / 2
                        mid_y = (self.current_y + self.person_position[1]) / 2
                        self.ax.text(mid_x, mid_y, f'📏 {self.person_distance:.1f}m', 
                                   fontsize=10, ha='center', color='white', weight='bold',
                                   bbox=dict(boxstyle='round,pad=0.4', 
                                           facecolor='#ff0040', edgecolor='#ffff00',
                                           alpha=0.8, linewidth=2))
                
                # Draw drone with enhanced styling
                # Drone glow effect
                for size, alpha in [(25, 0.1), (18, 0.2)]:
                    self.ax.plot(self.current_x, self.current_y, 'o', 
                               markersize=size, color='#00bfff', alpha=alpha, zorder=7)
                
                # Main drone marker
                self.ax.plot(self.current_x, self.current_y, marker='h', markersize=14,
                           color='#00bfff', markeredgecolor='white', markeredgewidth=2,
                           label='DRONE', zorder=8)
                
                # Draw 100m detection radius with radar sweep effect
                detection_circle = Circle((self.current_x, self.current_y), 100, 
                                        color='#00ff41', fill=False, alpha=0.15, 
                                        linestyle=':', linewidth=1.5, label='100m Radar')
                self.ax.add_patch(detection_circle)
                
                # Add radar range markers
                for radius in [25, 50, 75]:
                    range_circle = Circle((self.current_x, self.current_y), radius,
                                        color='#00ff41', fill=False, alpha=0.05,
                                        linestyle=':', linewidth=0.5)
                    self.ax.add_patch(range_circle)
                
                # Enhanced heading indicator with gradient
                arrow_len = 4
                arrow_x = self.current_x + arrow_len * math.cos(self.current_yaw)
                arrow_y = self.current_y + arrow_len * math.sin(self.current_yaw)
                
                # Multiple arrows for gradient effect
                for i, (length, width, alpha) in enumerate([(4, 0.8, 0.8), (3.5, 0.6, 0.6), (3, 0.4, 0.4)]):
                    end_x = self.current_x + length * math.cos(self.current_yaw)
                    end_y = self.current_y + length * math.sin(self.current_yaw)
                    self.ax.arrow(self.current_x, self.current_y, 
                                end_x - self.current_x, end_y - self.current_y,
                                head_width=width, head_length=0.5, 
                                fc='#00bfff', ec='#00bfff', alpha=alpha, zorder=7-i)
                
                # SAR state indicator with better styling
                if self.sar_state != "idle":
                    state_styles = {
                        'searching': ('#ffff00', '🔍 SEARCHING'),
                        'target_found': ('#ff8800', '⚠️ TARGET FOUND'),
                        'orbiting': ('#ff0040', '🔄 ORBITING'),
                        'hovering_inspection': ('#ff00ff', '👁️ INSPECTING'),
                        'mission_complete': ('#00ff41', '✅ COMPLETE')
                    }
                    color, text = state_styles.get(self.sar_state, ('#808080', self.sar_state.upper()))
                    
                    self.ax.text(self.current_x, self.current_y - 5, text, 
                               fontsize=9, ha='center', color='white', weight='bold',
                               bbox=dict(boxstyle='round,pad=0.5', 
                                       facecolor=color, edgecolor='white',
                                       alpha=0.9, linewidth=2))
                
                # Set title with enhanced styling
                title = f'🚁 ALTITUDE: {self.current_z:.1f}m'
                if self.sar_state != "idle":
                    title += f'  |  {self.sar_message}'
                self.ax.set_title(title, fontsize=14, fontweight='bold', 
                                color='#00ff41', pad=15, family='monospace')
                
                # Dynamic axis limits
                all_x = [p[0] for p in self.path_history] + [self.current_x]
                all_y = [p[1] for p in self.path_history] + [self.current_y]
                
                if self.person_position:
                    all_x.append(self.person_position[0])
                    all_y.append(self.person_position[1])
                
                if self.waypoints:
                    all_x.extend([w['x'] for w in self.waypoints])
                    all_y.extend([w['y'] for w in self.waypoints])
                
                margin = 10
                x_min, x_max = min(all_x) - margin, max(all_x) + margin
                y_min, y_max = min(all_y) - margin, max(all_y) + margin
                
                # Ensure minimum range
                range_min = 30
                if x_max - x_min < range_min:
                    center = (x_max + x_min) / 2
                    x_min, x_max = center - range_min/2, center + range_min/2
                if y_max - y_min < range_min:
                    center = (y_max + y_min) / 2
                    y_min, y_max = center - range_min/2, center + range_min/2
                
                self.ax.set_xlim(x_min, x_max)
                self.ax.set_ylim(y_min, y_max)
            else:
                self.ax.set_title('⏳ WAITING FOR DRONE SIGNAL...', fontsize=14, 
                                fontweight='bold', color='#ffff00', family='monospace')
                self.ax.set_xlim(-30, 30)
                self.ax.set_ylim(-30, 30)
            
            # Set labels and legend with dark theme
            self.ax.set_xlabel('X Position (meters)', fontsize=11, color='#e0e0e0', fontweight='medium')
            self.ax.set_ylabel('Y Position (meters)', fontsize=11, color='#e0e0e0', fontweight='medium')
            
            # Enhanced legend styling
            legend = self.ax.legend(loc='upper right', fontsize=9, 
                                  facecolor='#1a1a1a', edgecolor='#00ff41',
                                  framealpha=0.9, borderpad=1)
            for text in legend.get_texts():
                text.set_color('#e0e0e0')
            
            # Restore tick and spine colors
            self.ax.tick_params(colors='#b0b0b0', which='both', labelsize=9)
            for spine in self.ax.spines.values():
                spine.set_color('#4a4a4a')
            
            self.ax.set_aspect('equal')
            
            # Update info panel
            self.update_info_panel()
            
            # Refresh plot
            plt.draw()
            plt.pause(0.01)
            
        except Exception as e:
            self.get_logger().error(f'Plot update error: {e}')
    
    def update_info_panel(self):
        """Update the information panel"""
        self.info_ax.clear()
        self.info_ax.axis('off')
        self.info_ax.set_facecolor('#1a1a1a')
        
        # Re-add decorative border
        border = FancyBboxPatch((0.02, 0.02), 0.96, 0.96, 
                                boxstyle="round,pad=0.01",
                                linewidth=2, edgecolor='#00ff41', 
                                facecolor='none', alpha=0.3)
        self.info_ax.add_patch(border)
        
        # Title section
        self.info_ax.text(0.5, 0.93, '╔═══ SYSTEM STATUS ═══╗', 
                         transform=self.info_ax.transAxes,
                         fontsize=11, color='#00ff41', weight='bold',
                         ha='center', family='monospace')
        
        y_pos = 0.85
        
        # Drone section with box
        drone_box = FancyBboxPatch((0.05, y_pos-0.18), 0.9, 0.16,
                                   boxstyle="round,pad=0.01",
                                   facecolor='#002a3a', edgecolor='#00bfff',
                                   alpha=0.3, linewidth=1)
        self.info_ax.add_patch(drone_box)
        
        self.info_ax.text(0.1, y_pos, '🚁 DRONE STATUS', transform=self.info_ax.transAxes,
                         fontsize=10, color='#00bfff', weight='bold', family='monospace')
        y_pos -= 0.04
        self.info_ax.text(0.1, y_pos, f'Position: ({self.current_x:.1f}, {self.current_y:.1f}, {self.current_z:.1f})m',
                         transform=self.info_ax.transAxes, fontsize=9, color='#b0b0b0', family='monospace')
        y_pos -= 0.035
        self.info_ax.text(0.1, y_pos, f'Heading: {math.degrees(self.current_yaw):.0f}°',
                         transform=self.info_ax.transAxes, fontsize=9, color='#b0b0b0', family='monospace')
        
        if self.start_position:
            dx = self.current_x - self.start_position[0]
            dy = self.current_y - self.start_position[1]
            dist_from_start = math.sqrt(dx**2 + dy**2)
            y_pos -= 0.035
            self.info_ax.text(0.1, y_pos, f'Range: {dist_from_start:.1f}m from start',
                             transform=self.info_ax.transAxes, fontsize=9, color='#b0b0b0', family='monospace')
        
        y_pos -= 0.08
        
        # Detection section with box
        if self.person_detected and self.person_position:
            detect_box = FancyBboxPatch((0.05, y_pos-0.22), 0.9, 0.2,
                                       boxstyle="round,pad=0.01",
                                       facecolor='#3a0000', edgecolor='#ff0040',
                                       alpha=0.3, linewidth=1)
            color_detect = '#ff0040'
            status_text = '⚠️ PERSON DETECTED'
        else:
            detect_box = FancyBboxPatch((0.05, y_pos-0.15), 0.9, 0.13,
                                       boxstyle="round,pad=0.01",
                                       facecolor='#1a1a1a', edgecolor='#4a4a4a',
                                       alpha=0.3, linewidth=1)
            color_detect = '#808080'
            status_text = '👤 NO DETECTION'
        self.info_ax.add_patch(detect_box)
        
        self.info_ax.text(0.1, y_pos, status_text, transform=self.info_ax.transAxes,
                         fontsize=10, color=color_detect, weight='bold', family='monospace')
        y_pos -= 0.04
        self.info_ax.text(0.1, y_pos, f'Filters: ≥70% conf, ≤100m',
                         transform=self.info_ax.transAxes, fontsize=8, color='#808080', family='monospace')
        
        if self.person_detected and self.person_position:
            y_pos -= 0.035
            self.info_ax.text(0.1, y_pos, f'Pos: ({self.person_position[0]:.1f}, {self.person_position[1]:.1f})m',
                             transform=self.info_ax.transAxes, fontsize=9, color='#ff6666', family='monospace')
            y_pos -= 0.035
            self.info_ax.text(0.1, y_pos, f'Distance: {self.person_distance:.1f}m',
                             transform=self.info_ax.transAxes, fontsize=9, color='#ff6666', family='monospace')
            y_pos -= 0.035
            self.info_ax.text(0.1, y_pos, f'Confidence: {self.person_confidence:.0%}',
                             transform=self.info_ax.transAxes, fontsize=9, color='#ff6666', family='monospace')
            
            method = "Depth+BBox" if self.depth_image is not None else "BBox only"
            y_pos -= 0.035
            self.info_ax.text(0.1, y_pos, f'Method: {method}',
                             transform=self.info_ax.transAxes, fontsize=8, color='#808080', family='monospace')
        
        y_pos -= 0.08
        
        # SAR Mission section
        mission_color = '#00ff41' if self.sar_state != 'idle' else '#808080'
        mission_box = FancyBboxPatch((0.05, y_pos-0.12), 0.9, 0.1,
                                     boxstyle="round,pad=0.01",
                                     facecolor='#002a00' if self.sar_state != 'idle' else '#1a1a1a',
                                     edgecolor=mission_color,
                                     alpha=0.3, linewidth=1)
        self.info_ax.add_patch(mission_box)
        
        self.info_ax.text(0.1, y_pos, '🎯 SAR MISSION', transform=self.info_ax.transAxes,
                         fontsize=10, color=mission_color, weight='bold', family='monospace')
        y_pos -= 0.04
        state_text = self.sar_state.replace('_', ' ').upper()
        self.info_ax.text(0.1, y_pos, f'State: {state_text}',
                         transform=self.info_ax.transAxes, fontsize=9, color='#b0b0b0', family='monospace')
        if self.sar_message:
            y_pos -= 0.035
            self.info_ax.text(0.1, y_pos, f'{self.sar_message[:20]}...' if len(self.sar_message) > 20 else self.sar_message,
                             transform=self.info_ax.transAxes, fontsize=8, color='#808080', family='monospace')

def main(args=None):
    rclpy.init(args=args)
    node = EnhancedMapVisualizer()
    
    # Setup plot in main thread with subplots
    plt.ion()
    node.fig = plt.figure(figsize=(16, 10))
    
    # Create grid: main map on left (larger), info panel on right
    node.ax = plt.subplot2grid((1, 4), (0, 0), colspan=3)
    node.info_ax = plt.subplot2grid((1, 4), (0, 3))
    
    node.setup_plot()
    
    try:
        # Main loop - handle both ROS and matplotlib in main thread
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            node.update_plot()
            
    except KeyboardInterrupt:
        pass
    finally:
        plt.close('all')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()