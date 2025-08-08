#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage, RegionOfInterest
from drone_interfaces.msg import SceneDescription, DetectedObject
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
from datetime import datetime
from ultralytics import YOLO
import torch

class VisionToTextNode(Node):
    def __init__(self):
        super().__init__('vision_to_text_node')
        
        # Parameters
        self.declare_parameter('use_compressed', False)
        self.declare_parameter('camera_topic', '/airsim_node/drone_1/front_center/Scene')
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('yolo_model', 'yolov8m.pt')  # Using medium for RTX 5090
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('process_every_n_frames', 3)  # Process every 3rd frame
        
        self.use_compressed = self.get_parameter('use_compressed').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.enable_viz = self.get_parameter('enable_visualization').value
        self.yolo_model_name = self.get_parameter('yolo_model').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        self.process_every_n = self.get_parameter('process_every_n_frames').value
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.scene_pub = self.create_publisher(
            SceneDescription, 
            '/drone/scene_description', 
            10
        )
        
        if self.enable_viz:
            self.debug_img_pub = self.create_publisher(
                Image,
                '/drone/vision/debug_image',
                10
            )
        
        # Subscribers
        if self.use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage,
                self.camera_topic + '/compressed',
                self.compressed_image_callback,
                10
            )
        else:
            self.image_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.image_callback,
                10
            )
        
        # Processing state
        self.frame_counter = 0
        self.last_process_time = datetime.now()
        
        # Initialize YOLO model
        self.init_models()
        
        self.get_logger().info(f'Vision to Text Node initialized')
        self.get_logger().info(f'Subscribing to: {self.camera_topic}')
        self.get_logger().info(f'Using YOLO model: {self.yolo_model_name}')
        self.get_logger().info(f'GPU Available: {torch.cuda.is_available()}')
        if torch.cuda.is_available():
            self.get_logger().info(f'GPU Device: {torch.cuda.get_device_name(0)}')
        
    def init_models(self):
        """Initialize YOLO model"""
        self.get_logger().info('Initializing YOLO model...')
        try:
            # Load YOLO model
            self.yolo = YOLO(self.yolo_model_name)
            
            # Verify model is on GPU
            if torch.cuda.is_available():
                self.yolo.to('cuda')
                self.get_logger().info('YOLO model loaded on GPU')
            else:
                self.get_logger().warn('Running on CPU - this will be slow!')
                
            # Get class names
            self.class_names = self.yolo.names
            self.get_logger().info(f'Loaded {len(self.class_names)} object classes')
            
            # Warm up the model
            dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
            _ = self.yolo(dummy_img, verbose=False)
            self.get_logger().info('Model warmup complete')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')
            self.yolo = None
            
    def image_callback(self, msg):
        """Process raw image"""
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n != 0:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.process_image(cv_image, msg.header)
        except Exception as e:
            self.get_logger().error(f'Failed to process image: {e}')
            
    def compressed_image_callback(self, msg):
        """Process compressed image"""
        self.frame_counter += 1
        if self.frame_counter % self.process_every_n != 0:
            return
            
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self.process_image(cv_image, msg.header)
        except Exception as e:
            self.get_logger().error(f'Failed to process compressed image: {e}')
            
    def process_image(self, cv_image, header):
        """Main image processing pipeline with YOLO"""
        if self.yolo is None:
            self.get_logger().error('YOLO model not initialized!')
            return
            
        start_time = datetime.now()
        
        # Create scene description
        scene_msg = SceneDescription()
        scene_msg.header = header
        
        # Basic image analysis
        height, width = cv_image.shape[:2]
        
        # Run YOLO detection
        try:
            results = self.yolo(
                cv_image,
                conf=self.conf_threshold,
                iou=0.45,
                imgsz=640,
                half=True,  # Use FP16 on RTX 5090
                verbose=False
            )[0]  # Get first result
            
            # Process YOLO detections
            detected_objects = self.process_yolo_results(results, width, height)
            scene_msg.detected_objects = detected_objects
            
            # Analyze image properties
            scene_msg.visibility_score = self.calculate_visibility(cv_image)
            scene_msg.lighting_conditions = self.analyze_lighting(cv_image)
            scene_msg.weather_conditions = self.analyze_weather(cv_image)
            
            # Generate description
            scene_msg.general_description = self.generate_scene_description(
                detected_objects, scene_msg.lighting_conditions, scene_msg.weather_conditions
            )
            
            # Analyze spatial relationships
            scene_msg.spatial_relationships = self.analyze_spatial_relationships(
                detected_objects, width, height
            )
            
            # Safety assessment
            scene_msg.safety_assessment = self.assess_safety(detected_objects)
            
            # Check for anomalies
            scene_msg.anomalies = self.detect_anomalies(cv_image, detected_objects)
            
            # Publish scene description
            self.scene_pub.publish(scene_msg)
            
            # Visualize if enabled
            if self.enable_viz:
                self.visualize_results(cv_image, detected_objects, results)
                
            # Log performance
            process_time = (datetime.now() - start_time).total_seconds()
            self.get_logger().info(
                f'Processed frame in {process_time:.3f}s - Found {len(detected_objects)} objects',
                throttle_duration_sec=2.0
            )
            
        except Exception as e:
            self.get_logger().error(f'YOLO processing error: {e}')
            
    def process_yolo_results(self, results, img_width, img_height):
        """Convert YOLO results to DetectedObject messages"""
        detected_objects = []
        
        if results.boxes is None:
            return detected_objects
            
        boxes = results.boxes
        
        for i, box in enumerate(boxes):
            obj = DetectedObject()
            obj.object_id = i
            
            # Get class name and confidence
            class_id = int(box.cls[0])
            obj.label = self.class_names[class_id]
            obj.confidence = float(box.conf[0])
            
            # Get bounding box
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            
            # Create RegionOfInterest
            obj.bounding_box = RegionOfInterest()
            obj.bounding_box.x_offset = int(x1)
            obj.bounding_box.y_offset = int(y1)
            obj.bounding_box.width = int(x2 - x1)
            obj.bounding_box.height = int(y2 - y1)
            obj.bounding_box.do_rectify = False
            
            # Calculate relative position (normalized to image center)
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            obj.position = Point()
            obj.position.x = (center_x - img_width/2) / (img_width/2)
            obj.position.y = (img_height/2 - center_y) / (img_height/2)
            obj.position.z = 0.0  # No depth info from single camera
            
            # Estimate distance based on object size (heuristic)
            obj.distance = self.estimate_distance(obj.label, obj.bounding_box, img_height)
            
            # Determine size category
            bbox_area = obj.bounding_box.width * obj.bounding_box.height
            img_area = img_width * img_height
            area_ratio = bbox_area / img_area
            
            if area_ratio > 0.3:
                obj.size = "large"
            elif area_ratio > 0.1:
                obj.size = "medium"
            else:
                obj.size = "small"
                
            detected_objects.append(obj)
            
        return detected_objects
        
    def estimate_distance(self, label, bbox, img_height):
        """Estimate distance based on object type and size"""
        # Simple heuristic - in real application, use depth camera or stereo
        # Assume average heights for common objects
        object_heights = {
            'person': 1.7,  # meters
            'car': 1.5,
            'truck': 3.0,
            'bicycle': 1.0,
            'motorcycle': 1.2,
            'bird': 0.2,
            'dog': 0.5,
            'cat': 0.3,
        }
        
        # Get assumed height
        assumed_height = object_heights.get(label, 1.0)
        
        # Simple pinhole camera model (very rough estimate)
        # Distance = (real_height * focal_length) / pixel_height
        # Assuming focal length ~= image height for typical FOV
        if bbox.height > 0:
            distance = (assumed_height * img_height) / bbox.height
            return float(min(distance, 100.0))  # Cap at 100m
        return 50.0  # Default distance
        
    def calculate_visibility(self, image):
        """Calculate visibility score based on image clarity"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Use Laplacian variance as sharpness measure
        laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
        
        # Normalize to 0-1 range (adjust these values based on testing)
        visibility = min(laplacian_var / 1000.0, 1.0)
        return float(visibility)
        
    def analyze_lighting(self, image):
        """Analyze lighting conditions"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        avg_brightness = gray.mean()
        
        if avg_brightness < 30:
            return "very_dark"
        elif avg_brightness < 60:
            return "dark"
        elif avg_brightness < 120:
            return "dim"
        elif avg_brightness < 200:
            return "normal"
        else:
            return "bright"
            
    def analyze_weather(self, image):
        """Analyze weather conditions from image"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Check contrast as fog indicator
        contrast = gray.std()
        
        # Check for rain/snow (would need more sophisticated detection)
        if contrast < 15:
            return "heavy_fog"
        elif contrast < 30:
            return "foggy"
        elif contrast < 50:
            return "hazy"
        else:
            return "clear"
            
    def generate_scene_description(self, objects, lighting, weather):
        """Generate comprehensive textual description"""
        desc_parts = []
        
        # Overall scene
        if not objects:
            desc_parts.append("No significant objects detected in view")
        else:
            # Count objects by type
            object_counts = {}
            for obj in objects:
                if obj.label not in object_counts:
                    object_counts[obj.label] = 0
                object_counts[obj.label] += 1
                
            # Describe what's detected
            count_strs = []
            for label, count in object_counts.items():
                if count == 1:
                    count_strs.append(f"1 {label}")
                else:
                    count_strs.append(f"{count} {label}s")
            desc_parts.append(f"Detected: {', '.join(count_strs)}")
            
        # Add environmental conditions
        desc_parts.append(f"Lighting: {lighting}")
        desc_parts.append(f"Weather: {weather}")
        
        return ". ".join(desc_parts)
        
    def analyze_spatial_relationships(self, objects, width, height):
        """Analyze spatial relationships between detected objects"""
        relationships = []
        
        if len(objects) < 2:
            return relationships
            
        # Sort objects by distance
        objects_sorted = sorted(objects, key=lambda x: x.distance)
        
        # Describe nearest objects
        if objects_sorted:
            nearest = objects_sorted[0]
            relationships.append(f"Nearest object: {nearest.label} at ~{nearest.distance:.1f}m")
            
        # Describe relative positions
        for obj in objects[:5]:  # Limit to first 5 objects
            if obj.position.x < -0.3:
                direction = "left"
            elif obj.position.x > 0.3:
                direction = "right"
            else:
                direction = "center"
                
            if obj.position.y > 0.3:
                direction += "-top"
            elif obj.position.y < -0.3:
                direction += "-bottom"
                
            relationships.append(f"{obj.label} in {direction} of view")
            
        return relationships
        
    def assess_safety(self, objects):
        """Assess safety based on detected objects"""
        if not objects:
            return "Clear path - no obstacles detected"
            
        # Check for immediate threats
        min_distance = min([obj.distance for obj in objects] + [100])
        
        for obj in objects:
            if obj.distance < 3.0:
                return f"WARNING: {obj.label} very close ({obj.distance:.1f}m)"
            elif obj.distance < 10.0 and obj.size in ["medium", "large"]:
                return f"CAUTION: {obj.label} nearby ({obj.distance:.1f}m)"
                
        if "person" in [obj.label for obj in objects]:
            closest_person = min([obj for obj in objects if obj.label == "person"], 
                               key=lambda x: x.distance)
            return f"Person detected at {closest_person.distance:.1f}m - approach carefully"
            
        return "Scene appears safe for navigation"
        
    def detect_anomalies(self, image, objects):
        """Detect anomalies in the scene"""
        anomalies = []
        
        # Check visibility
        visibility = self.calculate_visibility(image)
        if visibility < 0.3:
            anomalies.append("Low visibility conditions")
            
        # Check for unusual object combinations
        object_labels = [obj.label for obj in objects]
        
        # Examples of unusual combinations (customize for SAR)
        if "fire" in object_labels or "smoke" in object_labels:
            anomalies.append("Fire/smoke detected - potential emergency")
            
        if "person" in object_labels:
            # Check if person is in unusual position
            person_objs = [obj for obj in objects if obj.label == "person"]
            for person in person_objs:
                if person.position.y < -0.5:  # Lower part of image
                    anomalies.append("Person detected in unusual position")
                    
        return anomalies
        
    def visualize_results(self, image, objects, yolo_results):
        """Visualize detection results with annotations"""
        vis_image = image.copy()
        height, width = image.shape[:2]
        
        # Draw YOLO boxes with custom annotations
        if yolo_results.boxes is not None:
            for i, (box, obj) in enumerate(zip(yolo_results.boxes, objects)):
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                
                # Choose color based on distance
                if obj.distance < 5.0:
                    color = (0, 0, 255)  # Red for close
                elif obj.distance < 15.0:
                    color = (0, 165, 255)  # Orange for medium
                else:
                    color = (0, 255, 0)  # Green for far
                    
                # Draw box
                cv2.rectangle(vis_image, (x1, y1), (x2, y2), color, 2)
                
                # Create label with distance
                label = f"{obj.label} ({obj.confidence:.2f}) ~{obj.distance:.1f}m"
                
                # Draw label background
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(vis_image, 
                            (x1, y1 - label_size[1] - 4),
                            (x1 + label_size[0], y1),
                            color, -1)
                
                # Draw label text
                cv2.putText(vis_image, label,
                          (x1, y1 - 2),
                          cv2.FONT_HERSHEY_SIMPLEX,
                          0.5, (255, 255, 255), 2)
                          
        # Add scene info overlay
        info_text = [
            f"FPS: {self.frame_counter / max((datetime.now() - self.last_process_time).total_seconds(), 1):.1f}",
            f"Objects: {len(objects)}",
            f"Visibility: {self.calculate_visibility(image):.2f}",
            f"Lighting: {self.analyze_lighting(image)}"
        ]
        
        y_offset = 30
        for text in info_text:
            cv2.putText(vis_image, text, (10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            y_offset += 25
            
        # Publish debug image
        debug_msg = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
        self.debug_img_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionToTextNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down vision node...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
