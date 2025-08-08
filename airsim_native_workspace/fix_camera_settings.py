#!/usr/bin/env python3
"""
Fix AirSim settings.json to ensure camera is properly configured
"""

import json
import os
import shutil
from datetime import datetime

def fix_settings():
    settings_path = os.path.expanduser("~/Documents/AirSim/settings.json")
    
    print("=" * 60)
    print("     AIRSIM CAMERA SETTINGS FIX")
    print("=" * 60)
    print()
    
    # Check if settings file exists
    if not os.path.exists(settings_path):
        print(f"❌ Settings file not found at: {settings_path}")
        print("\nCreating new settings file...")
        
        # Create directory if needed
        os.makedirs(os.path.dirname(settings_path), exist_ok=True)
        
        # Create basic settings
        settings = {
            "SettingsVersion": 1.2,
            "SimMode": "Multirotor",
            "Vehicles": {
                "Drone1": {
                    "VehicleType": "SimpleFlight",
                    "AutoCreate": true
                }
            }
        }
    else:
        # Backup existing settings
        backup_path = settings_path + f".backup_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        shutil.copy2(settings_path, backup_path)
        print(f"✅ Backed up existing settings to: {backup_path}")
        
        # Load existing settings
        with open(settings_path, 'r') as f:
            settings = json.load(f)
    
    # Ensure proper camera configuration
    print("\nConfiguring camera settings...")
    
    # Add CameraDefaults if not present
    if "CameraDefaults" not in settings:
        settings["CameraDefaults"] = {
            "CaptureSettings": [
                {
                    "ImageType": 0,
                    "Width": 1920,
                    "Height": 1080,
                    "FOV_Degrees": 90,
                    "AutoExposureSpeed": 100,
                    "MotionBlurAmount": 0
                }
            ]
        }
        print("  ✅ Added CameraDefaults")
    
    # Ensure vehicle has cameras configured
    if "Vehicles" in settings:
        for vehicle_name, vehicle_config in settings["Vehicles"].items():
            if "Cameras" not in vehicle_config:
                vehicle_config["Cameras"] = {
                    "0": {
                        "CaptureSettings": [
                            {
                                "ImageType": 0,
                                "Width": 1920,
                                "Height": 1080,
                                "FOV_Degrees": 90
                            }
                        ],
                        "X": 0.5,
                        "Y": 0,
                        "Z": -0.5,
                        "Pitch": 0,
                        "Roll": 0,
                        "Yaw": 0
                    }
                }
                print(f"  ✅ Added camera to vehicle: {vehicle_name}")
    
    # Add recording settings for better performance
    if "Recording" not in settings:
        settings["Recording"] = {
            "RecordInterval": 0.05,
            "Cameras": [
                {
                    "CameraName": "0",
                    "ImageType": 0,
                    "PixelsAsFloat": False,
                    "Compress": True
                }
            ]
        }
        print("  ✅ Added recording settings")
    
    # Save updated settings
    with open(settings_path, 'w') as f:
        json.dump(settings, f, indent=2)
    
    print(f"\n✅ Settings saved to: {settings_path}")
    
    # Display the camera configuration
    print("\n" + "=" * 60)
    print("CAMERA CONFIGURATION:")
    print("=" * 60)
    print(json.dumps(settings.get("CameraDefaults", {}), indent=2))
    
    if "Vehicles" in settings:
        for vehicle_name, vehicle_config in settings["Vehicles"].items():
            if "Cameras" in vehicle_config:
                print(f"\nVehicle '{vehicle_name}' cameras:")
                print(json.dumps(vehicle_config["Cameras"], indent=2))
    
    print("\n" + "=" * 60)
    print("✅ SETTINGS FIXED!")
    print("=" * 60)
    print("\nNext steps:")
    print("1. Restart Unreal Engine")
    print("2. Press PLAY")
    print("3. Run: python3 vision/camera_test.py")
    print("\nThe camera should now work properly!")

if __name__ == "__main__":
    fix_settings()