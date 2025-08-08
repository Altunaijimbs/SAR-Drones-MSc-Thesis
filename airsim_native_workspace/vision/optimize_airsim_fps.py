#!/usr/bin/env python3
"""
Optimize AirSim for higher FPS by adjusting settings and testing different approaches
"""

import cosysairsim as airsim
import numpy as np
import cv2
import time
import json
import os
from pathlib import Path

class AirSimOptimizer:
    def __init__(self):
        print("Connecting to AirSim...")
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        print("✅ Connected\n")
    
    def check_current_settings(self):
        """Check current AirSim settings"""
        print("Current AirSim Settings:")
        print("=" * 60)
        
        settings_path = Path.home() / "Documents" / "AirSim" / "settings.json"
        
        if settings_path.exists():
            with open(settings_path, 'r') as f:
                settings = json.load(f)
            
            # Check relevant settings
            print(f"SimMode: {settings.get('SimMode', 'Not set')}")
            
            # Check vehicle settings
            vehicles = settings.get('Vehicles', {})
            for vehicle_name, vehicle_settings in vehicles.items():
                print(f"\nVehicle: {vehicle_name}")
                print(f"  Type: {vehicle_settings.get('VehicleType', 'Not set')}")
                
                # Check camera settings
                cameras = vehicle_settings.get('Cameras', {})
                for cam_name, cam_settings in cameras.items():
                    print(f"  Camera: {cam_name}")
                    capture_settings = cam_settings.get('CaptureSettings', [])
                    for cs in capture_settings:
                        print(f"    Resolution: {cs.get('Width', 'N/A')}x{cs.get('Height', 'N/A')}")
                        print(f"    Target FPS: {cs.get('TargetFPS', 'Not set')}")
            
            # Check physics settings
            physics_fps = settings.get('PhysicsEngineName', 'Not set')
            print(f"\nPhysics Engine: {physics_fps}")
            
            # Check clock speed
            clock_speed = settings.get('ClockSpeed', 1.0)
            print(f"Clock Speed: {clock_speed}")
            
            # Check view mode
            view_mode = settings.get('ViewMode', 'Not set')
            print(f"View Mode: {view_mode}")
        else:
            print("❌ settings.json not found!")
    
    def create_optimized_settings(self):
        """Create optimized settings.json for high FPS"""
        
        optimized_settings = {
            "SettingsVersion": 2.0,
            "SimMode": "Multirotor",
            "ClockSpeed": 1.0,  # Real-time
            "ViewMode": "",  # No view mode for performance
            "PhysicsEngineName": "FastPhysicsEngine",  # Try faster physics
            "Vehicles": {
                "Drone1": {
                    "VehicleType": "SimpleFlight",
                    "AutoCreate": True,
                    "Cameras": {
                        "0": {
                            "CaptureSettings": [
                                {
                                    "ImageType": 0,  # Scene
                                    "Width": 640,    # Lower resolution for higher FPS
                                    "Height": 480,
                                    "FOV_Degrees": 90,
                                    "TargetFPS": 60,  # Request 60 FPS
                                    "MotionBlurAmount": 0,  # Disable motion blur
                                    "AutoExposureSpeed": 100,
                                    "AutoExposureBias": 0
                                }
                            ],
                            "X": 0.5,
                            "Y": 0,
                            "Z": 0.1,
                            "Pitch": -10,
                            "Roll": 0,
                            "Yaw": 0,
                            "UsePhysicsEngine": False  # Disable physics for camera
                        }
                    },
                    "EnableTrace": False,  # Disable trace for performance
                    "EnableCollisions": True,
                    "AllowAPIAlways": True
                }
            },
            "CameraDefaults": {
                "CaptureSettings": [
                    {
                        "ImageType": 0,
                        "Width": 640,
                        "Height": 480,
                        "FOV_Degrees": 90,
                        "TargetFPS": 60,
                        "MotionBlurAmount": 0
                    }
                ]
            },
            # Reduce recording for performance
            "Recording": {
                "RecordOnMove": False,
                "RecordInterval": 0.5,
                "Enabled": False
            },
            # Optimize sub-windows
            "SubWindows": [
                {
                    "WindowID": 0,
                    "CameraName": "0",
                    "ImageType": 0,
                    "Visible": False  # Hide sub-windows for performance
                }
            ]
        }
        
        # Save to a test file
        test_settings_path = Path.home() / "Documents" / "AirSim" / "settings_optimized.json"
        
        with open(test_settings_path, 'w') as f:
            json.dump(optimized_settings, f, indent=2)
        
        print(f"\n✅ Created optimized settings at: {test_settings_path}")
        print("\nOptimizations applied:")
        print("  - Reduced resolution to 640x480")
        print("  - Disabled motion blur")
        print("  - Disabled recording")
        print("  - Hidden sub-windows")
        print("  - Requested 60 FPS target")
        print("\nTo use these settings:")
        print(f"  1. Backup current: mv ~/Documents/AirSim/settings.json ~/Documents/AirSim/settings_backup.json")
        print(f"  2. Apply optimized: cp ~/Documents/AirSim/settings_optimized.json ~/Documents/AirSim/settings.json")
        print(f"  3. Restart UE5")
    
    def test_minimal_capture(self):
        """Test absolute minimal capture for max FPS"""
        print("\nTesting Minimal Capture (no processing)...")
        print("-" * 50)
        
        frames = 0
        start = time.time()
        duration = 5
        
        while time.time() - start < duration:
            try:
                # Just request, don't process
                _ = self.client.simGetImage("0", airsim.ImageType.Scene)
                frames += 1
            except:
                pass
        
        elapsed = time.time() - start
        fps = frames / elapsed
        
        print(f"  Minimal capture FPS: {fps:.2f}")
        print(f"  This is the maximum achievable with current setup")
        
        return fps
    
    def test_async_capture(self):
        """Test async capture if available"""
        print("\nTesting Async Capture...")
        print("-" * 50)
        
        # Check if async methods exist
        if hasattr(self.client, 'simGetImagesAsync'):
            print("  ✅ Async methods available")
            
            frames = 0
            start = time.time()
            duration = 5
            
            while time.time() - start < duration:
                try:
                    # Try async capture
                    future = self.client.simGetImagesAsync([
                        airsim.ImageRequest("0", airsim.ImageType.Scene, False, False)
                    ])
                    responses = future.join()
                    if responses:
                        frames += 1
                except:
                    pass
            
            elapsed = time.time() - start
            fps = frames / elapsed
            print(f"  Async capture FPS: {fps:.2f}")
        else:
            print("  ❌ Async methods not available in this version")
    
    def suggest_ue5_settings(self):
        """Suggest UE5 project settings for better performance"""
        print("\n" + "=" * 60)
        print("UE5 OPTIMIZATION SUGGESTIONS")
        print("=" * 60)
        print("""
1. IN UNREAL ENGINE 5:
   
   a) Project Settings > Engine > Rendering:
      - Disable Motion Blur
      - Set Anti-Aliasing to FXAA (fastest)
      - Disable Screen Space Reflections
      - Reduce Shadow Quality to Medium/Low
      - Disable Ray Tracing (if enabled)
   
   b) Project Settings > Engine > General:
      - Set Fixed Frame Rate: 60 FPS
      - Smooth Frame Rate: OFF
   
   c) Editor Preferences > Performance:
      - Use Less CPU when in Background: OFF
      - Monitor Editor Performance: OFF
   
   d) In your Level:
      - Reduce Texture Quality
      - Disable unnecessary visual effects
      - Use simpler materials
      - Reduce number of light sources

2. SYSTEM LEVEL:
   
   a) Graphics Card Settings (NVIDIA/AMD):
      - Set to "Performance" mode
      - Disable V-Sync
      - Set to "Single Display Performance"
   
   b) Windows Settings:
      - Game Mode: ON
      - Hardware Accelerated GPU Scheduling: ON
      - Graphics Performance Preference: High Performance

3. AIRSIM SPECIFIC:
   
   a) In UE5 Viewport:
      - Set viewport to "Unlit" mode for testing
      - Reduce viewport quality to "Medium"
      - Close unnecessary editor windows
   
   b) Play Mode:
      - Use "Standalone Game" instead of PIE
      - Or use "New Editor Window (PIE)"
        """)

def main():
    print("=" * 60)
    print("AIRSIM FPS OPTIMIZATION TOOL")
    print("=" * 60)
    print()
    
    optimizer = AirSimOptimizer()
    
    # Check current settings
    optimizer.check_current_settings()
    
    # Test current performance
    print("\n" + "=" * 60)
    print("PERFORMANCE TESTS")
    print("=" * 60)
    
    min_fps = optimizer.test_minimal_capture()
    optimizer.test_async_capture()
    
    # Create optimized settings
    print("\n" + "=" * 60)
    print("CREATING OPTIMIZED SETTINGS")
    print("=" * 60)
    optimizer.create_optimized_settings()
    
    # Show UE5 suggestions
    optimizer.suggest_ue5_settings()
    
    # Final recommendations
    print("\n" + "=" * 60)
    print("ANALYSIS & RECOMMENDATIONS")
    print("=" * 60)
    
    if min_fps < 20:
        print("⚠️  FPS is limited by the simulator, not the client code!")
        print("\nThe bottleneck is likely:")
        print("1. UE5 rendering performance")
        print("2. AirSim plugin overhead")
        print("3. System hardware limitations")
        print("\nRECOMMENDED ACTIONS:")
        print("1. Apply the optimized settings.json")
        print("2. Reduce UE5 graphics quality")
        print("3. Use lower camera resolution (640x480)")
        print("4. Consider using PX4 mode with ROS2 image topics (may be faster)")
    else:
        print("✅ Acceptable FPS achieved!")
        print("Apply optimizations for even better performance.")
    
    print("\n" + "=" * 60)

if __name__ == "__main__":
    main()