#!/usr/bin/env python3
"""
Fixed Camera Test - Uses simGetImage for compressed images
Works around the simGetImages API version issues
"""

import cosysairsim as airsim
import numpy as np
import cv2
import time

def test_camera_simple():
    """Test using simGetImage (compressed) instead of simGetImages"""
    print("=" * 60)
    print("     AIRSIM CAMERA TEST (FIXED)")
    print("=" * 60)
    print()
    
    # Connect
    print("Connecting to AirSim...")
    client = airsim.MultirotorClient()
    client.confirmConnection()
    print("✅ Connected")
    print()
    
    print("Testing simGetImage (compressed format):")
    print("-" * 40)
    
    # Try getting compressed image (PNG/JPEG)
    try:
        print("  Getting compressed image...")
        # simGetImage returns compressed image as bytes
        png_image = client.simGetImage("0", airsim.ImageType.Scene)
        
        if png_image:
            print(f"    ✅ Got compressed image! Size: {len(png_image)} bytes")
            
            # Convert to numpy array
            nparr = np.frombuffer(png_image, np.uint8)
            img_bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            
            if img_bgr is not None:
                print(f"    ✅ Decoded to numpy array: {img_bgr.shape}")
                
                # Save test image
                filename = "test_camera_fixed.png"
                cv2.imwrite(filename, img_bgr)
                print(f"    ✅ Saved test image to: {filename}")
                
                return True, img_bgr
            else:
                print("    ❌ Failed to decode image")
                return False, None
        else:
            print("    ❌ No image data received")
            return False, None
            
    except Exception as e:
        print(f"    ❌ Failed to get image: {e}")
        return False, None

def capture_continuous_feed_fixed():
    """Capture continuous camera feed using simGetImage"""
    print()
    print("Starting continuous camera capture (fixed)...")
    print("Press 'q' to quit, 's' to save screenshot")
    print("-" * 40)
    
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # Create OpenCV window
    cv2.namedWindow("AirSim Camera Feed", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("AirSim Camera Feed", 1280, 720)
    
    frame_count = 0
    start_time = time.time()
    screenshot_count = 0
    
    try:
        while True:
            try:
                # Get compressed image
                png_image = client.simGetImage("0", airsim.ImageType.Scene)
                
                if png_image:
                    # Decode compressed image
                    nparr = np.frombuffer(png_image, np.uint8)
                    img_bgr = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    if img_bgr is not None:
                        # Resize for display
                        img_display = cv2.resize(img_bgr, (1280, 720))
                        
                        # Add overlay info
                        frame_count += 1
                        elapsed = time.time() - start_time
                        fps = frame_count / elapsed if elapsed > 0 else 0
                        
                        # Add text overlay
                        cv2.putText(img_display, f"FPS: {fps:.1f}", (10, 30),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        cv2.putText(img_display, f"Frame: {frame_count}", (10, 60),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        cv2.putText(img_display, "Press 'q' to quit, 's' for screenshot", (10, 90),
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                        
                        # Display
                        cv2.imshow("AirSim Camera Feed", img_display)
                        
                        # Handle key press
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q'):
                            break
                        elif key == ord('s'):
                            # Save screenshot
                            screenshot_count += 1
                            filename = f"screenshot_{screenshot_count}.png"
                            cv2.imwrite(filename, img_bgr)
                            print(f"  📸 Saved screenshot: {filename}")
                
            except Exception as e:
                # Show error frame
                error_frame = np.zeros((720, 1280, 3), dtype=np.uint8)
                cv2.putText(error_frame, "Camera Error", (500, 360),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                cv2.putText(error_frame, str(e)[:80], (200, 400),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                cv2.imshow("AirSim Camera Feed", error_frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                    
    except KeyboardInterrupt:
        print("\n  Stopped by user")
    
    cv2.destroyAllWindows()
    print(f"\n✅ Captured {frame_count} frames")
    print(f"   Average FPS: {frame_count / elapsed:.1f}")
    if screenshot_count > 0:
        print(f"   Saved {screenshot_count} screenshots")

def test_all_camera_names():
    """Test different camera names to find valid ones"""
    print("\nTesting different camera names:")
    print("-" * 40)
    
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    # Common camera names to try
    camera_names = ["0", "1", "front_center", "front", "FrontCenter", 
                   "bottom", "back", "fpv", "main"]
    
    working_cameras = []
    
    for cam_name in camera_names:
        try:
            # Try to get image with this camera name
            png_image = client.simGetImage(cam_name, airsim.ImageType.Scene)
            
            if png_image and len(png_image) > 100:  # Check if we got actual data
                print(f"  ✅ Camera '{cam_name}' works! ({len(png_image)} bytes)")
                working_cameras.append(cam_name)
            else:
                print(f"  ❌ Camera '{cam_name}' - no data")
        except Exception as e:
            error_msg = str(e)
            if "doesn't exist" in error_msg.lower():
                print(f"  ❌ Camera '{cam_name}' doesn't exist")
            else:
                print(f"  ❌ Camera '{cam_name}' - error: {error_msg[:50]}")
    
    return working_cameras

if __name__ == "__main__":
    print("\n🎥 AIRSIM CAMERA SYSTEM TEST (FIXED)\n")
    
    # Test simple camera capture
    success, image = test_camera_simple()
    
    if success:
        print()
        print("Testing all camera names...")
        working_cameras = test_all_camera_names()
        
        if working_cameras:
            print(f"\n✅ Found {len(working_cameras)} working camera(s): {', '.join(working_cameras)}")
        
        print()
        print("=" * 60)
        print("✅ CAMERA SYSTEM WORKING!")
        print("=" * 60)
        print()
        print("Would you like to see live camera feed? (y/n): ", end="")
        
        choice = input().lower()
        
        if choice == 'y':
            capture_continuous_feed_fixed()
    else:
        print()
        print("=" * 60)
        print("❌ CAMERA SYSTEM NOT WORKING")
        print("=" * 60)
        print()
        print("Troubleshooting:")
        print("1. Check ~/Documents/AirSim/settings.json")
        print("2. Ensure Vehicles -> Drone1 -> Cameras section exists")
        print("3. Make sure UE5 is running and PLAY is pressed")
        print("4. Try restarting UE5")