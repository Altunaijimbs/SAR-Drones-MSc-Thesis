#!/usr/bin/env python3
"""
Vision Detection Bridge - FIXED VERSION
Properly converts SceneDescription messages to /yolo/detections format
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from drone_interfaces.msg import SceneDescription, DetectedObject
import json

class VisionDetectionBridge(Node):
    def __init__(self):
        super().__init__('vision_detection_bridge')
        
        # Subscribe to ACTUAL SceneDescription type
        self.scene_sub = self.create_subscription(
            SceneDescription,  # Correct message type!
            '/drone/scene_description',
            self.process_scene,
            10
        )
        
        # Publish to YOLO detections format (String)
        self.detection_pub = self.create_publisher(
            String,
            '/yolo/detections',
            10
        )
        
        self.get_logger().info('Vision Detection Bridge FIXED - Proper SceneDescription handling')
        self.get_logger().info('Subscribing to: /drone/scene_description (SceneDescription type)')
        self.get_logger().info('Publishing to: /yolo/detections (String type)')
    
    def process_scene(self, msg):
        """Convert SceneDescription to YOLO detection format"""
        try:
            # Extract detected objects from SceneDescription
            detections = []
            
            for obj in msg.detected_objects:
                # Convert each DetectedObject to YOLO format
                detection = {
                    'class': obj.label.lower(),
                    'confidence': float(obj.confidence),
                    'bbox': {
                        # Convert RegionOfInterest to normalized coordinates
                        'x': float(obj.bounding_box.x_offset + obj.bounding_box.width/2) / 1920,  # Center X
                        'y': float(obj.bounding_box.y_offset + obj.bounding_box.height/2) / 1080,  # Center Y
                        'width': float(obj.bounding_box.width) / 1920,
                        'height': float(obj.bounding_box.height) / 1080
                    },
                    'distance': float(obj.distance) if obj.distance > 0 else None,
                    'position': {
                        'x': float(obj.position.x),
                        'y': float(obj.position.y),
                        'z': float(obj.position.z)
                    }
                }
                detections.append(detection)
                
                # Log detection
                self.get_logger().info(
                    f"Detected: {obj.label} (conf: {obj.confidence:.2f}, dist: {obj.distance:.1f}m)"
                )
            
            if detections:
                # Format as YOLO-style detections
                detection_msg = {
                    'detections': detections,
                    'frame_id': 'camera',
                    'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                }
                
                # Publish detection as JSON string
                det_str = String()
                det_str.data = json.dumps(detection_msg)
                self.detection_pub.publish(det_str)
                
                # Log summary
                classes = [d['class'] for d in detections]
                self.get_logger().info(f"Published {len(detections)} detections: {', '.join(classes)}")
            else:
                # Log scene description even if no objects
                if msg.general_description:
                    self.get_logger().debug(f"Scene: {msg.general_description[:100]}...")
                    
        except Exception as e:
            self.get_logger().error(f"Scene processing error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectionBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()