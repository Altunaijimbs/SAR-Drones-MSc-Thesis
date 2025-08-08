#!/bin/bash
# Switch between PX4 and SimpleFlight modes

echo "╔══════════════════════════════════════════════════════╗"
echo "║         AIRSIM MODE SWITCHER                         ║"
echo "╚══════════════════════════════════════════════════════╝"
echo ""

SETTINGS_DIR="$HOME/Documents/AirSim"
CURRENT_SETTINGS="$SETTINGS_DIR/settings.json"
PX4_SETTINGS="$SETTINGS_DIR/settings_px4.json"
SIMPLE_SETTINGS="$SETTINGS_DIR/settings_simpleflight.json"

# Check current mode
if [ -f "$CURRENT_SETTINGS" ]; then
    if grep -q "PX4Multirotor" "$CURRENT_SETTINGS"; then
        CURRENT_MODE="PX4"
    elif grep -q "SimpleFlight" "$CURRENT_SETTINGS"; then
        CURRENT_MODE="SimpleFlight"
    else
        CURRENT_MODE="Unknown"
    fi
else
    CURRENT_MODE="No settings file"
fi

echo "Current mode: $CURRENT_MODE"
echo ""
echo "Select mode:"
echo "1. PX4 Mode (Realistic - requires PX4 SITL)"
echo "2. SimpleFlight Mode (Simple - native AirSim)"
echo "3. View current settings"
echo "4. Exit"
echo ""
echo -n "Choice (1-4): "
read choice

case $choice in
    1)
        echo ""
        echo "Switching to PX4 mode..."
        
        # Backup current if exists
        if [ -f "$CURRENT_SETTINGS" ]; then
            cp "$CURRENT_SETTINGS" "$CURRENT_SETTINGS.backup"
        fi
        
        # Check if PX4 settings exist
        if [ -f "$PX4_SETTINGS" ]; then
            cp "$PX4_SETTINGS" "$CURRENT_SETTINGS"
        else
            # Use the user's original PX4 settings
            cat > "$CURRENT_SETTINGS" << 'EOF'
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
            "Cameras": {
                "front_center": {
                    "CaptureSettings": [
                        {
                            "ImageType": 0,
                            "Width": 1920,
                            "Height": 1080,
                            "FOV_Degrees": 90,
                            "TargetFPS": 30
                        }
                    ],
                    "X": 0.5,
                    "Y": 0,
                    "Z": 0.1,
                    "Pitch": -10,
                    "Roll": 0,
                    "Yaw": 0
                }
            },
            "Sensors":{
                "Imu": {
                    "SensorType": 2,
                    "Enabled": true
                },
                "Gps": {
                    "SensorType": 3,
                    "Enabled": true
                },
                "Magnetometer": {
                    "SensorType": 4,
                    "Enabled": true
                },
                "Barometer":{
                    "SensorType": 1,
                    "Enabled": true,
                    "PressureFactorSigma": 0.0001825
                },
                "LidarSensor1": {
                    "SensorType": 6,             
                    "Enabled": true,
                    "NumberOfChannels": 32,
                    "RotationsPerSecond": 10,
                    "PointsPerSecond": 50000,
                    "X": 0, "Y": 0, "Z": -0.1,    
                    "Roll": 0, "Pitch": 0, "Yaw": 0,
                    "Range": 20,
                    "VerticalFOVUpper": 10,
                    "VerticalFOVLower": -10
                }
            },
            "Parameters": {
                "NAV_RCL_ACT": 0,
                "NAV_DLL_ACT": 0,
                "COM_OBL_ACT": 1,
                "COM_RC_IN_MODE": 4,
                "COM_DISARM_LAND": -1,
                "LPE_LAT": 47.641468,
                "LPE_LON": -122.140165
            }
        }
    }
}
EOF
            # Save for future use
            cp "$CURRENT_SETTINGS" "$PX4_SETTINGS"
        fi
        
        echo "✅ Switched to PX4 mode"
        echo ""
        echo "Requirements:"
        echo "  • Start PX4 SITL first"
        echo "  • Use launch_hybrid_system.sh"
        echo "  • Camera: 'front_center'"
        echo "  • Vehicle: 'PX4'"
        ;;
        
    2)
        echo ""
        echo "Switching to SimpleFlight mode..."
        
        # Backup current if exists
        if [ -f "$CURRENT_SETTINGS" ]; then
            cp "$CURRENT_SETTINGS" "$CURRENT_SETTINGS.backup"
        fi
        
        # Check if SimpleFlight settings exist
        if [ -f "$SIMPLE_SETTINGS" ]; then
            cp "$SIMPLE_SETTINGS" "$CURRENT_SETTINGS"
        else
            # Copy from our workspace
            cp "settings_simpleflight.json" "$CURRENT_SETTINGS"
            # Save for future use
            cp "$CURRENT_SETTINGS" "$SIMPLE_SETTINGS"
        fi
        
        echo "✅ Switched to SimpleFlight mode"
        echo ""
        echo "Benefits:"
        echo "  • No PX4 needed!"
        echo "  • Direct control with moveOnPathAsync"
        echo "  • Camera: '0'"
        echo "  • Vehicle: 'Drone1'"
        echo ""
        echo "Run: python3 patterns/smooth_square.py"
        ;;
        
    3)
        echo ""
        echo "Current settings.json:"
        echo "─────────────────────"
        if [ -f "$CURRENT_SETTINGS" ]; then
            # Show just the key parts
            grep -E "VehicleType|SimMode|Cameras|front_center" "$CURRENT_SETTINGS" | head -20
        else
            echo "No settings file found"
        fi
        ;;
        
    4)
        echo "Exiting..."
        exit 0
        ;;
        
    *)
        echo "Invalid choice"
        ;;
esac

echo ""
echo "IMPORTANT: Restart Unreal Engine after switching modes!"