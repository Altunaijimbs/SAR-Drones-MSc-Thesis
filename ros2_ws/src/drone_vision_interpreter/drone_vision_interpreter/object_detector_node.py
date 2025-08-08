#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from drone_interfaces.msg import DetectedObject, ObstacleArray, Obstacle
from geometry_msgs.msg import Point, Vector3
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import torch

class ObjectDetectorNode(Node):
    def __init__(self):
        super().__init__('object_detector_node')
        
        # Parameters
        self.declare_parameter('camera_topic', '/airsim_node/drone_1/front_center/Scene')
        self.declare_parameter('depth_topic', '/airsim_node/drone_1/front_center/DepthPlanar')
        self.declare_parameter('yolo_model', 'yolov8s.pt')  # Smaller model for obstacle detection
        self.declare_parameter('confidence_threshold', 0.5)
        
        self.camera_topic = self.get_parameter('camera_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.yolo_model_name = self.get_parameter('yolo_model').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Publishers
        self.obstacle_pub = self.create_publisher(
            ObstacleArray,
            '/drone/obstacles',
            10
        )
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )
        
        # Storage
        self.latest_depth = None
        self.obstacle_id_counter = 0
        
        # Initialize YOLO
        self.init_yolo()
        
        self.get_logger().info('Object Detector Node initialized')
        self.get_logger().info(f'Using YOLO model: {self.yolo_model_name}')
        
    def init_yolo(self):
        """Initialize YOLO for obstacle detection"""
        try:
            self.yolo = YOLO(self.yolo_model_name)
            if torch.cuda.is_available():
                self.yolo.to('cuda')
            self.get_logger().info('YOLO model loaded for obstacle detection')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO: {e}')
            self.yolo = None
        
    def depth_callback(self, msg):
        """Store latest depth image"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except Exception as e:
            self.get_logger().error(f'Failed to process depth: {e}')
            
    def image_callback(self, msg):
        """Process image for obstacles"""
        if self.yolo is None:
            return
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Run YOLO detection
            results = self.yolo(cv_image, conf=self.conf_threshold, verbose=False)[0]
            
            # Convert to obstacles
            obstacles = self.yolo_to_obstacles(results, cv_image.shape)
            
            # Publish obstacles
            obstacle_msg = ObstacleArray()
            obstacle_msg.header = msg.header
            obstacle_msg.obstacles = obstacles
            self.obstacle_pub.publish(obstacle_msg)
            
            if len(obstacles) > 0:
                self.get_logger().info(
                    f'Detected {len(obstacles)} obstacles',
                    throttle_duration_sec=2.0
                )
            
        except Exception as e:
            self.get_logger().error(f'Failed to detect objects: {e}')
            
    def yolo_to_obstacles(self, results, image_shape):
        """Convert YOLO detections to Obstacle messages"""
        obstacles = []
        height, width = image_shape[:2]
        
        if results.boxes is None:
            return obstacles
            
        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            center_x = (x1 + x2) / 2
            center_y = (y1 + y2) / 2
            
            # Create obstacle
            obstacle = Obstacle()
            obstacle.obstacle_id = self.obstacle_id_counter
            self.obstacle_id_counter += 1
            
            # Classify obstacle type
            class_name = self.yolo.names[int(box.cls[0])]
            if class_name in ['person', 'bicycle', 'car', 'motorcycle', 'bus', 'truck']:
                obstacle.type = "DYNAMIC"
            else:
                obstacle.type = "STATIC"
                
            obstacle.confidence = float(box.conf[0])
            
            # Estimate position (camera frame)
            obstacle.position = Point()
            obstacle.position.x = (center_x - width/2) / width * 10.0  # Rough scaling
            obstacle.position.y = 0.0  # Will be updated with depth
            obstacle.position.z = (height/2 - center_y) / height * 5.0
            
            # Get depth if available
            if self.latest_depth is not None:
                try:
                    depth_value = self.latest_depth[int(center_y), int(center_x)]
                    if not np.isnan(depth_value) and not np.isinf(depth_value):
                        obstacle.position.y = float(depth_value)
                except:
                    pass
                    
            # Estimate dimensions based on object type and distance
            obstacle.dimensions = Vector3()
            bbox_width = x2 - x1
            bbox_height = y2 - y1
            
            # Simple size estimation
            if obstacle.position.y > 0:
                obstacle.dimensions.x = (bbox_width / width) * obstacle.position.y * 2
                obstacle.dimensions.y = obstacle.dimensions.x * 0.5  # Rough estimate
                obstacle.dimensions.z = (bbox_height / height) * obstacle.position.y * 2
            else:
                obstacle.dimensions.x = 1.0
                obstacle.dimensions.y = 1.0
                obstacle.dimensions.z = 1.0
                
            obstacles.append(obstacle)
            
        return obstacles

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down object detector...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
