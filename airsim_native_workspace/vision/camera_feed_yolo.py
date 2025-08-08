#!/usr/bin/env python3
"""
Camera Feed with YOLO Detection
Proper implementation for getting camera feed and detecting objects
"""

import cosysairsim as airsim
import numpy as np
import cv2
import time
from ultralytics import YOLO
import os

class AirSimVisionSystem:
    def __init__(self, model_path='yolov8m.pt'):
        """Initialize vision system with YOLO"""
        print("Initializing AirSim Vision System...")
        
        # Connect to AirSim
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        
        # Determine correct camera API format
        self.camera_format = self.detect_camera_api_format()
        
        # Load YOLO model
        if os.path.exists(model_path):
            print(f"Loading YOLO model: {model_path}")
            self.yolo = YOLO(model_path)
        else:
            print(f"Downloading YOLO model: {model_path}")
            self.yolo = YOLO(model_path)
        
        # Detection parameters
        self.target_classes = ['person', 'car', 'truck', 'boat']  # SAR relevant
        self.confidence_threshold = 0.5
        
        print("✅ Vision system ready!")
    
    def detect_camera_api_format(self):
        """Detect which camera API format works"""
        # Try different formats
        try:
            self.client.simGetCameraInfo("0")
            return 1  # Old format
        except:
            try:
                self.client.simGetCameraInfo("0", "")
                return 2  # Standard format
            except:
                try:
                    self.client.simGetCameraInfo("0", "", False)
                    return 3  # Cosys format
                except:
                    return 0  # Unknown - but images might still work
    
    def get_camera_image(self, camera_name="0"):
        """Get camera image from AirSim"""
        try:
            responses = self.client.simGetImages([
                airsim.ImageRequest(camera_name, airsim.ImageType.Scene, False, False)
            ])
            
            if responses and len(responses) > 0:
                response = responses[0]
                
                # Convert to numpy array
                img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                img_rgb = img1d.reshape(response.height, response.width, 3)
                
                # Convert RGB to BGR for OpenCV/YOLO
                img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                
                return img_bgr
            
        except Exception as e:
            print(f"Error getting camera image: {e}")
        
        return None
    
    def detect_objects(self, image):
        """Run YOLO detection on image"""
        if image is None:
            return []
        
        # Run YOLO
        results = self.yolo(image, conf=self.confidence_threshold)
        
        detections = []
        for r in results:
            boxes = r.boxes
            if boxes is not None:
                for box in boxes:
                    # Get detection info
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    class_name = self.yolo.names[cls]
                    
                    # Get bounding box
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    
                    detection = {
                        'class': class_name,
                        'confidence': conf,
                        'bbox': [x1, y1, x2, y2],
                        'center': [(x1+x2)/2, (y1+y2)/2]
                    }
                    
                    detections.append(detection)
                    
                    # Check if it's a target class (person for SAR)
                    if class_name in self.target_classes:
                        print(f"🎯 TARGET DETECTED: {class_name} (conf: {conf:.2f})")
        
        return detections
    
    def draw_detections(self, image, detections):
        """Draw bounding boxes on image"""
        img_copy = image.copy()
        
        for det in detections:
            x1, y1, x2, y2 = [int(x) for x in det['bbox']]
            
            # Color based on class
            if det['class'] == 'person':
                color = (0, 0, 255)  # Red for person
            elif det['class'] in self.target_classes:
                color = (0, 255, 255)  # Yellow for other targets
            else:
                color = (0, 255, 0)  # Green for others
            
            # Draw box
            cv2.rectangle(img_copy, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f"{det['class']}: {det['confidence']:.2f}"
            cv2.putText(img_copy, label, (x1, y1-10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return img_copy
    
    def run_detection_loop(self, display=True):
        """Run continuous detection loop"""
        print("\nStarting detection loop...")
        print("Press 'q' to quit")
        print("-" * 40)
        
        if display:
            cv2.namedWindow("AirSim YOLO Detection", cv2.WINDOW_NORMAL)
        
        frame_count = 0
        detection_count = 0
        start_time = time.time()
        
        try:
            while True:
                # Get image
                image = self.get_camera_image()
                
                if image is not None:
                    # Run detection
                    detections = self.detect_objects(image)
                    
                    # Update counts
                    frame_count += 1
                    if detections:
                        detection_count += len([d for d in detections if d['class'] in self.target_classes])
                    
                    # Display
                    if display:
                        img_with_boxes = self.draw_detections(image, detections)
                        
                        # Add status text
                        status = f"Frame: {frame_count} | Targets found: {detection_count}"
                        cv2.putText(img_with_boxes, status, (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                        
                        cv2.imshow("AirSim YOLO Detection", img_with_boxes)
                        
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            break
                    
                    # Print FPS every 30 frames
                    if frame_count % 30 == 0:
                        elapsed = time.time() - start_time
                        fps = frame_count / elapsed
                        print(f"FPS: {fps:.1f} | Targets detected: {detection_count}")
                
                else:
                    print("Warning: No image received")
                    time.sleep(0.1)
                    
        except KeyboardInterrupt:
            print("\nStopped by user")
        
        if display:
            cv2.destroyAllWindows()
        
        # Summary
        elapsed = time.time() - start_time
        print("\n" + "=" * 40)
        print(f"Detection Summary:")
        print(f"  Total frames: {frame_count}")
        print(f"  Total targets detected: {detection_count}")
        print(f"  Average FPS: {frame_count/elapsed:.1f}")
        print("=" * 40)
    
    def search_and_detect(self):
        """Perform search pattern while detecting"""
        print("\nStarting Search and Detect mission...")
        
        # Enable API control
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        
        # Take off
        print("Taking off...")
        self.client.takeoffAsync().join()
        
        # Move to search altitude
        z = -20  # 20 meters
        self.client.moveToZAsync(z, 2).join()
        
        # Search pattern
        search_path = [
            airsim.Vector3r(0, 0, z),
            airsim.Vector3r(20, 0, z),
            airsim.Vector3r(20, 20, z),
            airsim.Vector3r(0, 20, z),
            airsim.Vector3r(0, 0, z),
        ]
        
        print("Executing search pattern while detecting...")
        
        # Start movement
        self.client.moveOnPathAsync(
            search_path,
            velocity=3,  # Slower for better detection
            timeout_sec=60,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(False, 0),
            lookahead=5,
            adaptive_lookahead=1
        )
        
        # Detect while flying
        person_found = False
        while not person_found:
            image = self.get_camera_image()
            if image is not None:
                detections = self.detect_objects(image)
                
                for det in detections:
                    if det['class'] == 'person' and det['confidence'] > 0.7:
                        print("\n🚨 PERSON DETECTED! Stopping search...")
                        person_found = True
                        
                        # Stop and hover
                        self.client.moveByVelocityAsync(0, 0, 0, 0.1).join()
                        self.client.hoverAsync().join()
                        
                        # Get position
                        state = self.client.getMultirotorState()
                        pos = state.kinematics_estimated.position
                        print(f"📍 Location: X={pos.x_val:.1f}, Y={pos.y_val:.1f}")
                        
                        break
            
            # Check if pattern complete
            time.sleep(0.1)
        
        print("\nMission complete!")
        
        # Land
        self.client.landAsync().join()
        self.client.armDisarm(False)
        self.client.enableApiControl(False)

def main():
    print("\n" + "🎥🤖" * 20)
    print("     AIRSIM VISION SYSTEM WITH YOLO")
    print("🎥🤖" * 20 + "\n")
    
    # Check for YOLO model
    model_path = "/home/mbs/SAR-Drones-MSc-Thesis/yolov8m.pt"
    if not os.path.exists(model_path):
        model_path = "yolov8m.pt"  # Will download if not found
    
    # Initialize system
    vision = AirSimVisionSystem(model_path)
    
    print("\nOptions:")
    print("1. Test camera feed only")
    print("2. Run YOLO detection on live feed")
    print("3. Search and detect mission")
    
    choice = input("\nSelect option (1-3): ")
    
    if choice == "1":
        # Just show camera feed
        vision.run_detection_loop(display=True)
    elif choice == "2":
        # Run with YOLO
        vision.run_detection_loop(display=True)
    elif choice == "3":
        # Autonomous search
        vision.search_and_detect()
    else:
        print("Invalid choice")

if __name__ == "__main__":
    main()