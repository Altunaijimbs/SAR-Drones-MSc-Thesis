#!/usr/bin/env python3
"""
Camera Test - Properly get camera feed from AirSim
Tests different API formats to find what works
"""

import cosysairsim as airsim
import numpy as np
import cv2
import time

def test_camera_apis():
    """Test different camera API formats to find the working one"""
    print("=" * 60)
    print("     AIRSIM CAMERA API TEST")
    print("=" * 60)
    print()
    
    # Connect
    print("Connecting to AirSim...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("✅ Connected")
    print()
    
    # Test different simGetCameraInfo signatures
    print("Testing simGetCameraInfo API formats:")
    print("-" * 40)
    
    camera_info = None
    working_format = None
    
    # Format 1: Just camera name (old AirSim)
    try:
        print("  Format 1: simGetCameraInfo(camera_name)")
        camera_info = client.simGetCameraInfo("0")
        print("    ✅ This format works!")
        working_format = 1
    except Exception as e:
        print(f"    ❌ Failed: {str(e)[:50]}")
    
    # Format 2: Camera name + vehicle name (newer AirSim)
    if not camera_info:
        try:
            print("  Format 2: simGetCameraInfo(camera_name, vehicle_name)")
            camera_info = client.simGetCameraInfo("0", "")
            print("    ✅ This format works!")
            working_format = 2
        except Exception as e:
            print(f"    ❌ Failed: {str(e)[:50]}")
    
    # Format 3: Camera name + vehicle name + external (Cosys-AirSim)
    if not camera_info:
        try:
            print("  Format 3: simGetCameraInfo(camera_name, vehicle_name, external)")
            camera_info = client.simGetCameraInfo("0", "", False)
            print("    ✅ This format works!")
            working_format = 3
        except Exception as e:
            print(f"    ❌ Failed: {str(e)[:50]}")
    
    # Format 4: Try with vehicle name specified
    if not camera_info:
        try:
            print("  Format 4: simGetCameraInfo with vehicle='Drone1'")
            camera_info = client.simGetCameraInfo("0", "Drone1", False)
            print("    ✅ This format works!")
            working_format = 4
        except Exception as e:
            print(f"    ❌ Failed: {str(e)[:50]}")
    
    print()
    if camera_info:
        print(f"✅ Camera info retrieved using format {working_format}!")
        print(f"   FOV: {camera_info.fov} degrees")
    else:
        print("⚠️  Camera info not available, but we can still get images!")
    
    print()
    print("Testing simGetImages (actual camera feed):")
    print("-" * 40)
    
    # Test getting actual images
    responses = None
    
    # Try getting images
    try:
        print("  Getting RGB image...")
        responses = client.simGetImages([
            airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
        ])
        
        if responses and len(responses) > 0:
            response = responses[0]
            print(f"    ✅ Got image! Size: {response.width}x{response.height}")
            
            # Convert to numpy array
            img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
            img_rgb = img1d.reshape(response.height, response.width, 3)
            
            print(f"    ✅ Converted to numpy array: {img_rgb.shape}")
            
            # Save test image
            filename = "test_camera_image.png"
            cv2.imwrite(filename, cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR))
            print(f"    ✅ Saved test image to: {filename}")
            
        else:
            print("    ❌ No image data received")
            
    except Exception as e:
        print(f"    ❌ Failed to get images: {e}")
    
    print()
    print("Testing different camera names:")
    print("-" * 40)
    
    # Common camera names to try
    camera_names = ["0", "front_center", "front", "FrontCenter", "1", "high_res"]
    
    for cam_name in camera_names:
        try:
            responses = client.simGetImages([
                airsim.ImageRequest(cam_name, airsim.ImageType.Scene, False, False)
            ])
            if responses and len(responses) > 0 and responses[0].width > 0:
                print(f"  ✅ Camera '{cam_name}' works!")
                break
        except:
            print(f"  ❌ Camera '{cam_name}' not found")
    
    return working_format, responses

def capture_continuous_feed():
    """Capture continuous camera feed for YOLO integration"""
    print()
    print("Starting continuous camera capture...")
    print("Press Ctrl+C to stop")
    print("-" * 40)
    
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # Create OpenCV window
    cv2.namedWindow("AirSim Camera Feed", cv2.WINDOW_NORMAL)
    
    frame_count = 0
    start_time = time.time()
    
    try:
        while True:
            # Get image
            responses = client.simGetImages([
                airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
            ])
            
            if responses and len(responses) > 0:
                response = responses[0]
                
                # Convert to numpy/OpenCV format
                img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
                img_rgb = img1d.reshape(response.height, response.width, 3)
                img_bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
                
                # Add FPS counter
                frame_count += 1
                if frame_count % 30 == 0:
                    elapsed = time.time() - start_time
                    fps = frame_count / elapsed
                    print(f"  FPS: {fps:.1f}")
                
                # Add text overlay
                cv2.putText(img_bgr, f"Frame: {frame_count}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Display
                cv2.imshow("AirSim Camera Feed", img_bgr)
                
                # This is where YOLO detection would happen
                # detected_objects = yolo_detect(img_bgr)
                
                # Break on 'q' key
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
    except KeyboardInterrupt:
        print("\n  Stopped by user")
    
    cv2.destroyAllWindows()
    print(f"\n✅ Captured {frame_count} frames")

if __name__ == "__main__":
    print("\n🎥 AIRSIM CAMERA SYSTEM TEST\n")
    
    # Test APIs
    working_format, responses = test_camera_apis()
    
    if responses:
        print()
        print("=" * 60)
        print("✅ CAMERA SYSTEM WORKING!")
        print("=" * 60)
        print()
        print("Would you like to see live camera feed? (y/n): ", end="")
        
        import sys
        choice = input().lower()
        
        if choice == 'y':
            capture_continuous_feed()
    else:
        print()
        print("=" * 60)
        print("❌ CAMERA SYSTEM NOT WORKING")
        print("=" * 60)
        print()
        print("Troubleshooting:")
        print("1. Check ~/Documents/AirSim/settings.json")
        print("2. Ensure CameraDefaults or Cameras section exists")
        print("3. Try restarting UE5")