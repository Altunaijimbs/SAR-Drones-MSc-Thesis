#!/bin/bash
# Launch SimpleFlight demos with real-time position tracking

echo "╔══════════════════════════════════════════════════════════╗"
echo "║        SIMPLEFLIGHT WITH REAL-TIME TRACKING             ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""
echo "This will show a live map of your drone's position!"
echo ""
echo "📋 Make sure:"
echo "  ✓ Unreal Engine 5 is running"
echo "  ✓ Your project is loaded"  
echo "  ✓ PLAY button is pressed"
echo ""
echo "═══════════════════════════════════════════════════════════"
echo "                    TRACKING OPTIONS"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "1. 📍 Track Only (no flight - just monitor position)"
echo "2. 🔷 Square Pattern with Tracking"
echo "3. 🔍 Search Pattern with Tracking"
echo "4. 🚁 Manual Flight with Tracking"
echo ""
echo "Which option? (1-4): "
read choice

case $choice in
    1)
        echo ""
        echo "Starting tracker only..."
        echo "Move the drone manually in UE5 or with other scripts"
        python3 visualizer/drone_tracker.py
        ;;
    2)
        echo ""
        echo "Enter square size in meters (default 20): "
        read size
        size=${size:-20}
        echo "Enter speed in m/s (default 5): "
        read speed
        speed=${speed:-5}
        python3 patterns/square_with_tracker.py --size $size --speed $speed
        ;;
    3)
        echo ""
        echo "Starting search pattern with tracking..."
        echo "Starting tracker in background..."
        python3 visualizer/drone_tracker.py &
        TRACKER_PID=$!
        sleep 2
        python3 patterns/search_rescue_pattern.py
        echo ""
        echo "Pattern complete! Tracker is still running."
        echo "Press Ctrl+C to close tracker..."
        wait $TRACKER_PID
        ;;
    4)
        echo ""
        echo "Starting tracker for manual flight..."
        echo "Tracker will show drone position in real-time"
        echo ""
        echo "In another terminal, run any pattern script or control manually"
        echo "Examples:"
        echo "  python3 scripts/hover_test.py"
        echo "  python3 patterns/smooth_square.py"
        echo ""
        python3 visualizer/drone_tracker.py
        ;;
    *)
        echo "Invalid option"
        exit 1
        ;;
esac

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "Session complete!"