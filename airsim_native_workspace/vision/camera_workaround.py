#!/usr/bin/env python3
"""
Camera workaround - directly construct the request to match server expectations
"""

import cosysairsim as airsim
import numpy as np
import cv2
import time

class CameraCapture:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        print("Connected to AirSim")
        
        # Find working camera configuration
        self.camera_name = "0"
        self.vehicle_name = "Drone1"
        
    def get_image_safe(self):
        """Get image using the safest method that should work"""
        try:
            # Method 1: Try the standard call first
            try:
                responses = self.client.simGetImages([
                    airsim.ImageRequest(self.camera_name, airsim.ImageType.Scene, False, False)
                ])
                if responses and responses[0].width > 0:
                    return self.process_response(responses[0])
            except:
                pass
            
            # Method 2: Try with explicit vehicle name
            try:
                responses = self.client.simGetImages([
                    airsim.ImageRequest(self.camera_name, airsim.ImageType.Scene, False, False)
                ], self.vehicle_name)
                if responses and responses[0].width > 0:
                    return self.process_response(responses[0])
            except:
                pass
            
            # Method 3: Try getting uncompressed image directly
            # This uses a different API path that might work
            try:
                # Get camera info first to check if camera exists
                info = self.client.simGetCameraInfo(self.camera_name)
                print(f"Camera exists with FOV: {info.fov}")
            except:
                pass
            
            # Method 4: Try the screenshot API as last resort
            # This captures the entire screen but at least gives us something
            try:
                # Take a screenshot (this usually works)
                png_data = self.client.simGetScreenshot()
                if png_data:
                    nparr = np.frombuffer(png_data, np.uint8)
                    img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    if img is not None:
                        print("Using screenshot as fallback")
                        return img
            except:
                pass
                
            return None
            
        except Exception as e:
            print(f"Error getting image: {e}")
            return None
    
    def process_response(self, response):
        """Convert response to OpenCV image"""
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(response.height, response.width, 3)
        img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
        return img_bgr
    
    def test_camera(self):
        """Test if we can get any image"""
        print("\nTrying to capture image...")
        img = self.get_image_safe()
        
        if img is not None:
            print(f"✅ Success! Got image with shape: {img.shape}")
            cv2.imwrite("test_capture.png", img)
            print("📸 Saved to test_capture.png")
            return True
        else:
            print("❌ Could not capture image")
            return False
    
    def show_live_feed(self):
        """Show live camera feed"""
        print("\nStarting live feed (press 'q' to quit)...")
        cv2.namedWindow("AirSim Camera", cv2.WINDOW_NORMAL)
        
        frame_count = 0
        start_time = time.time()
        
        while True:
            img = self.get_image_safe()
            
            if img is not None:
                # Add FPS counter
                frame_count += 1
                elapsed = time.time() - start_time
                fps = frame_count / elapsed if elapsed > 0 else 0
                
                # Add overlay
                cv2.putText(img, f"FPS: {fps:.1f}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Display
                cv2.imshow("AirSim Camera", img)
            else:
                # Show error frame
                error_img = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(error_img, "No Camera Feed", (200, 240),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.imshow("AirSim Camera", error_img)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cv2.destroyAllWindows()
        print(f"Captured {frame_count} frames")

if __name__ == "__main__":
    print("=" * 60)
    print("CAMERA WORKAROUND TEST")
    print("=" * 60)
    
    cam = CameraCapture()
    
    # Test single capture
    if cam.test_camera():
        print("\nWould you like to see live feed? (y/n): ", end="")
        if input().lower() == 'y':
            cam.show_live_feed()
    else:
        print("\nCamera capture not working.")
        print("The web interface will still work for drone control!")
        print("\nPossible solutions:")
        print("1. Update AirSim Python client: pip install --upgrade airsim")
        print("2. Check if UE5 is running with AirSim plugin")
        print("3. Use the web interface without camera feed")