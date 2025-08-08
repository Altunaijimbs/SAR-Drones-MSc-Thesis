# AirSim Integration Guide for New UE Environment

## Method 1: Adding AirSim to Existing UE Project

### Step 1: Prepare Your UE Project
1. Open your new UE5 project
2. Make sure it's a C++ project (not Blueprint-only)
   - If Blueprint-only: File → New C++ Class → None → Create Class
3. Close Unreal Editor

### Step 2: Copy AirSim Plugin
```bash
# Navigate to your new UE project
cd /path/to/your/NewUEProject

# Create Plugins directory if it doesn't exist
mkdir -p Plugins

# Copy AirSim plugin from your current working project
cp -r /path/to/Cosys-AirSim/Unreal/Plugins/AirSim Plugins/

# OR if you built from source
cp -r ~/Desktop/airsim/Cosys-AirSim/Unreal/Plugins/AirSim Plugins/
```

### Step 3: Update Project Files
1. Right-click your `.uproject` file → Generate Visual Studio/Xcode project files
2. Open the project in your IDE
3. Build the project

### Step 4: Configure in Editor
1. Open project in UE5
2. Edit → Plugins → Search "AirSim" → Enable
3. Restart editor when prompted
4. Window → World Settings → Game Mode Override → AirSimGameMode

### Step 5: Add Drone Spawn Point
1. In Content Browser: AirSim Content → Blueprints
2. Drag `BP_FlyingPawn` into your scene
3. Position it where you want drone to spawn
4. Set it as "Auto Possess Player: Player 0"

## Method 2: Using AirSim Blocks Environment as Template

### Step 1: Copy and Modify Blocks
```bash
# Copy Blocks environment
cp -r ~/Desktop/airsim/Cosys-AirSim/Unreal/Environments/Blocks /path/to/NewEnvironment

# Rename project files
cd /path/to/NewEnvironment
mv Blocks.uproject YourProjectName.uproject
```

### Step 2: Replace Content
1. Open in UE5
2. Delete existing Blocks geometry
3. Import your new environment assets
4. Keep AirSim PlayerStart and game mode settings

## Important Configuration

### Update settings.json
```json
{
    "SettingsVersion": 2.0,
    "SimMode": "Multirotor",
    "ClockType": "SteppableClock",
    "Vehicles": {
        "PX4": {
            "VehicleType": "PX4Multirotor",
            "UseSerial": false,
            "LockStep": true,
            "UseTcp": true,
            "TcpPort": 4560,
            "ControlPortLocal": 14540,
            "ControlPortRemote": 14580,
            "X": 0, "Y": 0, "Z": 0,  // Adjust spawn position
            "Yaw": 0,
            "Cameras": {
                "front_center": {
                    "CaptureSettings": [{
                        "ImageType": 0,
                        "Width": 1920,
                        "Height": 1080,
                        "FOV_Degrees": 90,
                        "TargetFPS": 30
                    }],
                    "X": 0.5, "Y": 0, "Z": 0.1,
                    "Pitch": -10, "Roll": 0, "Yaw": 0
                }
            }
        }
    }
}
```

## Testing Your New Environment

### Quick Test Commands
```bash
# Terminal 1: Launch your SAR system
cd /home/mbs/SAR-Drones-MSc-Thesis
./launch_with_vision.sh

# Terminal 2: Test basic control
ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once
ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once
```

## Troubleshooting

### Common Issues:
1. **"AirSim plugin not found"**
   - Ensure Plugins/AirSim exists in project
   - Regenerate project files

2. **"Vehicle doesn't spawn"**
   - Check PlayerStart location
   - Verify AirSimGameMode is set

3. **"Can't connect to PX4"**
   - Ensure only ONE UE project is running
   - Check settings.json is in ~/Documents/AirSim/

### Environment-Specific Considerations:
- **Large environments**: Increase draw distance in Project Settings
- **Indoor environments**: Adjust drone spawn height
- **Complex geometry**: May need to adjust collision settings

## Recommended Test Environments for SAR:
1. **Forest/Mountain**: Good for wilderness SAR
2. **Urban**: Building searches
3. **Disaster Site**: Rubble and obstacles
4. **Maritime**: Coastal/water rescue