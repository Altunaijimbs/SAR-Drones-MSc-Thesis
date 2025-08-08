#!/bin/bash

echo "========================================="
echo "SimpleFlight Web Interface Launcher"
echo "========================================="
echo ""

# Check if Flask is installed
if ! python3 -c "import flask" 2>/dev/null; then
    echo "Flask not found. Installing Flask and dependencies..."
    pip3 install flask flask-cors opencv-python-headless
fi

# Check if AirSim is installed
if ! python3 -c "import airsim" 2>/dev/null; then
    echo "Error: AirSim Python package not found!"
    echo "Please install with: pip3 install airsim"
    exit 1
fi

# Check if settings are in SimpleFlight mode
SETTINGS_FILE="$HOME/Documents/AirSim/settings.json"
if [ -f "$SETTINGS_FILE" ]; then
    if grep -q "PX4Multirotor" "$SETTINGS_FILE"; then
        echo "⚠️  WARNING: AirSim is configured for PX4 mode!"
        echo "The web interface works best with SimpleFlight mode."
        echo ""
        echo "Would you like to switch to SimpleFlight mode? (y/n)"
        read -r response
        if [[ "$response" == "y" ]]; then
            cd "$(dirname "$0")"
            ./switch_mode.sh
            echo ""
            echo "Please restart UE5 and press PLAY, then run this script again."
            exit 0
        fi
    fi
fi

echo "Starting SimpleFlight Web Interface..."
echo "========================================="
echo "Access the interface at: http://localhost:5001"
echo ""
echo "Keyboard Controls (when connected):"
echo "  W/↑ - Forward     S/↓ - Backward"
echo "  A/← - Left        D/→ - Right"
echo "  Q - Up            E - Down"
echo "  Z - Rotate Left   C - Rotate Right"
echo "  Space - Stop"
echo "========================================="
echo ""

# Launch the optimized web server
cd "$(dirname "$0")/web_interface"
echo "Launching optimized web server with 60 FPS camera..."
echo "Note: If you see IOLoop errors, they can be safely ignored."
echo "The drone control will still work!"
echo ""
python3 simpleflight_web_server_optimized.py