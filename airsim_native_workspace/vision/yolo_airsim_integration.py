#!/usr/bin/env python3
"""
YOLO + AirSim Integration for SimpleFlight
Real-time object detection on drone camera feed
"""

import cosysairsim as airsim
import numpy as np
import cv2
import time
from ultralytics import YOLO
import os
import sys

class YOLOAirSimDetector:
    def __init__(self, model_path=None):
        """Initialize YOLO detector with AirSim camera"""
        
        # Connect to AirSim
        print("Connecting to AirSim...")
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        print("✅ Connected to AirSim")
        
        # Load YOLO model
        if model_path is None:
            # Check for YOLO model in parent directory
            possible_paths = [
                "../../yolov8m.pt",
                "../../yolov8s.pt",
                "../yolov8m.pt",
                "../yolov8s.pt",
                "yolov8m.pt",
                "yolov8s.pt"
            ]
            
            for path in possible_paths:
                if os.path.exists(path):
                    model_path = path
                    break
            
            if model_path is None:
                print("Downloading YOLOv8m model...")
                model_path = "yolov8m.pt"
        
        print(f"Loading YOLO model: {model_path}")
        self.model = YOLO(model_path)
        print("✅ YOLO model loaded")
        
        # Detection settings
        self.confidence_threshold = 0.5
        self.target_classes = ['person', 'car', 'truck', 'boat']  # SAR relevant objects
        
        # Statistics
        self.frame_count = 0
        self.detection_count = 0
        self.start_time = None
        
    def get_camera_image(self):
        """Get image from AirSim camera"""
        try:
            # Get uncompressed images for better quality
            responses = self.client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
            ])
            
            if responses and len(responses) > 0:
                response = responses[0]
                # Convert to numpy array
                img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                img_rgb = img1d.reshape(response.height, response.width, 3)
                img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                return img_bgr
            
            return None
            
        except Exception as e:
            print(f"Camera error: {e}")
            return None
    
    def detect_objects(self, image):
        """Run YOLO detection on image"""
        results = self.model(image, conf=self.confidence_threshold)
        return results[0]  # Return first result (single image)
    
    def draw_detections(self, image, results):
        """Draw bounding boxes and labels on image"""
        detections = []
        
        if results.boxes is not None:
            for box in results.boxes:
                # Get box coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                confidence = box.conf[0].cpu().numpy()
                class_id = int(box.cls[0].cpu().numpy())
                class_name = self.model.names[class_id]
                
                # Check if this is a relevant class for SAR
                is_target = class_name in self.target_classes
                
                # Choose color based on class relevance
                if class_name == 'person':
                    color = (0, 0, 255)  # Red for people
                elif is_target:
                    color = (0, 255, 255)  # Yellow for vehicles
                else:
                    color = (0, 255, 0)  # Green for other objects
                
                # Draw bounding box
                cv2.rectangle(image, 
                            (int(x1), int(y1)), 
                            (int(x2), int(y2)), 
                            color, 2)
                
                # Draw label
                label = f"{class_name}: {confidence:.2f}"
                if is_target:
                    label = "⚠️ " + label  # Add warning emoji for targets
                
                label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                cv2.rectangle(image,
                            (int(x1), int(y1) - label_size[1] - 10),
                            (int(x1) + label_size[0], int(y1)),
                            color, -1)
                cv2.putText(image, label,
                          (int(x1), int(y1) - 5),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                          (255, 255, 255), 2)
                
                # Store detection info
                detections.append({
                    'class': class_name,
                    'confidence': confidence,
                    'bbox': [x1, y1, x2, y2],
                    'is_target': is_target
                })
        
        return detections
    
    def add_overlay(self, image, detections, fps):
        """Add information overlay to image"""
        h, w = image.shape[:2]
        
        # Add semi-transparent overlay for text
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (400, 150), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.3, image, 0.7, 0, image)
        
        # Add text info
        font = cv2.FONT_HERSHEY_SIMPLEX
        
        # Stats
        cv2.putText(image, f"FPS: {fps:.1f}", (20, 35), font, 0.7, (0, 255, 0), 2)
        cv2.putText(image, f"Frame: {self.frame_count}", (20, 65), font, 0.7, (0, 255, 0), 2)
        cv2.putText(image, f"Detections: {len(detections)}", (20, 95), font, 0.7, (0, 255, 0), 2)
        
        # Target count
        target_count = sum(1 for d in detections if d['is_target'])
        if target_count > 0:
            cv2.putText(image, f"TARGETS: {target_count}", (20, 125), 
                       font, 0.7, (0, 0, 255), 2)
        
        # Add detection list on the right
        if detections:
            y_offset = 30
            for det in detections[:5]:  # Show top 5 detections
                if det['is_target']:
                    text = f"⚠️ {det['class']}: {det['confidence']:.2f}"
                    color = (0, 0, 255)
                else:
                    text = f"{det['class']}: {det['confidence']:.2f}"
                    color = (0, 255, 0)
                
                cv2.putText(image, text, (w - 250, y_offset), 
                           font, 0.6, color, 2)
                y_offset += 30
        
        # Add instructions
        cv2.putText(image, "Press 'q' to quit, 's' to save", (20, h - 20), 
                   font, 0.6, (255, 255, 0), 2)
    
    def run_detection_loop(self):
        """Main detection loop"""
        print("\n" + "=" * 60)
        print("YOLO DETECTION STARTED")
        print("=" * 60)
        print("Controls:")
        print("  'q' - Quit")
        print("  's' - Save screenshot")
        print("  'r' - Reset statistics")
        print("=" * 60 + "\n")
        
        cv2.namedWindow("YOLO + AirSim Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("YOLO + AirSim Detection", 1280, 720)
        
        self.start_time = time.time()
        screenshot_count = 0
        
        try:
            while True:
                # Get camera image
                image = self.get_camera_image()
                
                if image is not None:
                    self.frame_count += 1
                    
                    # Run YOLO detection
                    results = self.detect_objects(image)
                    
                    # Draw detections
                    detections = self.draw_detections(image, results)
                    self.detection_count += len(detections)
                    
                    # Calculate FPS
                    elapsed = time.time() - self.start_time
                    fps = self.frame_count / elapsed if elapsed > 0 else 0
                    
                    # Add overlay
                    self.add_overlay(image, detections, fps)
                    
                    # Resize for display
                    display_image = cv2.resize(image, (1280, 720))
                    
                    # Show image
                    cv2.imshow("YOLO + AirSim Detection", display_image)
                    
                    # Print target detections
                    for det in detections:
                        if det['is_target']:
                            print(f"🎯 TARGET DETECTED: {det['class']} "
                                  f"(confidence: {det['confidence']:.2f})")
                else:
                    # Show error frame
                    error_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                    cv2.putText(error_frame, "No Camera Feed", (500, 360),
                               cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    cv2.imshow("YOLO + AirSim Detection", error_frame)
                
                # Handle key press
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    # Save screenshot
                    screenshot_count += 1
                    filename = f"yolo_detection_{screenshot_count}.png"
                    if image is not None:
                        cv2.imwrite(filename, image)
                        print(f"📸 Saved: {filename}")
                elif key == ord('r'):
                    # Reset statistics
                    self.frame_count = 0
                    self.detection_count = 0
                    self.start_time = time.time()
                    print("📊 Statistics reset")
                    
        except KeyboardInterrupt:
            print("\n⛔ Stopped by user")
        
        finally:
            cv2.destroyAllWindows()
            
            # Print statistics
            elapsed = time.time() - self.start_time
            avg_fps = self.frame_count / elapsed if elapsed > 0 else 0
            avg_detections = self.detection_count / self.frame_count if self.frame_count > 0 else 0
            
            print("\n" + "=" * 60)
            print("DETECTION STATISTICS")
            print("=" * 60)
            print(f"Total frames: {self.frame_count}")
            print(f"Total detections: {self.detection_count}")
            print(f"Average FPS: {avg_fps:.1f}")
            print(f"Average detections per frame: {avg_detections:.1f}")
            print(f"Screenshots saved: {screenshot_count}")
            print("=" * 60)

def main():
    print("\n🎯 YOLO + AIRSIM INTEGRATION FOR SAR\n")
    
    # Check for model path argument
    model_path = None
    if len(sys.argv) > 1:
        model_path = sys.argv[1]
    
    # Create detector
    detector = YOLOAirSimDetector(model_path)
    
    # Run detection
    detector.run_detection_loop()

if __name__ == "__main__":
    main()