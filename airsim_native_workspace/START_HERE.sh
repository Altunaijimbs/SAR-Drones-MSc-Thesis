#!/bin/bash
# Quick start script for AirSim Native workspace

echo "╔══════════════════════════════════════════════════════════╗"
echo "║         AIRSIM NATIVE WORKSPACE - QUICK START            ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
echo "This is a SIMPLIFIED system - No PX4/MAVROS needed!"
echo ""
echo "📋 CHECKLIST:"
echo "  [ ] Unreal Engine 5 is running"
echo "  [ ] Your project is loaded (Abandoned_Building_Zen_AirSim)"
echo "  [ ] PLAY button is pressed in UE5"
echo ""
echo "Press Enter when ready..."
read

echo ""
echo "Step 1: Testing connection..."
echo "─────────────────────────────"
python3 scripts/test_connection.py

if [ $? -ne 0 ]; then
    echo ""
    echo "❌ Connection failed!"
    echo ""
    echo "Please:"
    echo "1. Start Unreal Engine 5"
    echo "2. Open your project"
    echo "3. Press PLAY button"
    echo "4. Run this script again"
    exit 1
fi

echo ""
echo "✅ Connection successful!"
echo ""
echo "═══════════════════════════════════════════════════════════"
echo "                    AVAILABLE DEMOS"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "── BASIC DEMOS ──"
echo "1. 🔷 Basic Hover Test"
echo "2. 📦 Smooth Square Pattern" 
echo "3. 🔍 Search & Rescue Pattern"
echo ""
echo "── WITH LIVE TRACKING ──"
echo "4. 📍 Position Tracker Only (monitor drone position)"
echo "5. 🗺️  Square with Live Map"
echo "6. 📊 Any Pattern + Tracker (run both)"
echo ""
echo "── ADVANCED ──"
echo "7. 🎮 Custom Parameters"
echo "8. 📸 Camera/Vision Test"
echo "9. 🛡️  Safe Pattern Runner (guaranteed landing)"
echo ""
echo "── WEB INTERFACE ──"
echo "10. 🌐 Launch Web Control Interface (http://localhost:5001)"
echo ""
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "Which demo would you like to run? (1-10, or 'q' to quit): "
read choice

case $choice in
    1)
        echo "Running hover test..."
        python3 scripts/hover_test.py
        ;;
    2)
        echo "Running smooth square..."
        python3 patterns/smooth_square.py
        ;;
    3)
        echo "Running SAR pattern..."
        python3 patterns/search_rescue_pattern.py
        ;;
    4)
        echo ""
        echo "Starting position tracker..."
        echo "This will show the drone's position in real-time."
        echo "You can move the drone manually in UE5 or run other scripts."
        echo ""
        python3 visualizer/drone_tracker.py
        ;;
    5)
        echo ""
        echo "Square pattern with live tracking..."
        echo "Enter square size (meters, default 20): "
        read size
        size=${size:-20}
        echo "Enter speed (m/s, default 5): "
        read speed
        speed=${speed:-5}
        python3 patterns/square_with_tracker.py --size $size --speed $speed
        ;;
    6)
        echo ""
        echo "Starting tracker in background..."
        python3 visualizer/drone_tracker.py &
        TRACKER_PID=$!
        sleep 2
        echo ""
        echo "Tracker is running! Now choose a pattern:"
        echo "1) Hover Test"
        echo "2) Smooth Square"
        echo "3) Search & Rescue"
        read pattern_choice
        case $pattern_choice in
            1) python3 scripts/hover_test.py ;;
            2) python3 patterns/smooth_square.py ;;
            3) python3 patterns/search_rescue_pattern.py ;;
            *) echo "Invalid pattern" ;;
        esac
        echo ""
        echo "Pattern complete! Tracker is still running."
        echo "Press Ctrl+C to close tracker..."
        wait $TRACKER_PID
        ;;
    7)
        echo "Enter square size (meters): "
        read size
        echo "Enter speed (m/s): "
        read speed
        python3 patterns/smooth_square.py --size $size --speed $speed
        ;;
    8)
        echo ""
        echo "Camera/Vision Test Options:"
        echo "1) Basic camera feed (fixed version)"
        echo "2) YOLO object detection"
        read vision_choice
        case $vision_choice in
            1) python3 vision/camera_test_fixed.py ;;
            2) python3 vision/camera_feed_yolo.py ;;
            *) echo "Invalid choice" ;;
        esac
        ;;
    9)
        echo ""
        echo "Safe Pattern Runner - Guaranteed Landing!"
        echo "Choose pattern:"
        echo "1) Square"
        echo "2) Circle"
        read pattern_choice
        pattern="square"
        case $pattern_choice in
            1) pattern="square" ;;
            2) pattern="circle" ;;
            *) pattern="square" ;;
        esac
        echo "Enter size/radius (meters, default 20): "
        read size
        size=${size:-20}
        echo "Enter speed (m/s, default 5): "
        read speed
        speed=${speed:-5}
        python3 patterns/safe_pattern_runner.py --pattern $pattern --size $size --speed $speed
        ;;
    10)
        echo ""
        echo "Launching Web Control Interface..."
        echo "═══════════════════════════════════════════════════════════"
        ./launch_web_interface.sh
        ;;
    q)
        echo "Exiting..."
        exit 0
        ;;
    *)
        echo "Invalid choice"
        ;;
esac

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "                    NEXT STEPS"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "📚 Documentation: STARTUP_GUIDE.md"
echo "🔧 More scripts: scripts/ and patterns/ directories"
echo "🤖 Add LLM: llm_integration/ (coming soon)"
echo "👁️ Add Vision: vision/ (coming soon)"
echo ""
echo "Remember: This is for SIMULATION ONLY!"
echo "For real drones, use the PX4/MAVROS system."