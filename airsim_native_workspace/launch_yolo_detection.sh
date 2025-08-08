#!/bin/bash

echo "╔══════════════════════════════════════════════════════════╗"
echo "║        YOLO + AIRSIM DETECTION FOR SAR                   ║"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# Check if ultralytics is installed
if ! python3 -c "import ultralytics" 2>/dev/null; then
    echo "Installing ultralytics (YOLO)..."
    pip3 install ultralytics
fi

# Check if cosysairsim is installed
if ! python3 -c "import cosysairsim" 2>/dev/null; then
    echo "Error: cosysairsim not found!"
    echo "Please run: pip3 install cosysairsim"
    exit 1
fi

echo "📋 Prerequisites:"
echo "  ✓ UE5 running with AirSim"
echo "  ✓ SimpleFlight mode enabled"
echo "  ✓ PLAY button pressed"
echo ""

# Check for YOLO models
if [ -f "../../yolov8m.pt" ]; then
    echo "✅ Found YOLOv8m model"
    MODEL="../../yolov8m.pt"
elif [ -f "../../yolov8s.pt" ]; then
    echo "✅ Found YOLOv8s model"
    MODEL="../../yolov8s.pt"
else
    echo "📥 YOLO model will be downloaded on first run"
    MODEL=""
fi

echo ""
echo "Starting YOLO detection..."
echo "════════════════════════════════════════════════════════════"
echo ""

cd "$(dirname "$0")/vision"
if [ -n "$MODEL" ]; then
    python3 yolo_airsim_integration.py "$MODEL"
else
    python3 yolo_airsim_integration.py
fi