#!/bin/bash
# Fix Unreal Engine Zen Storage Server Port Conflict

echo "=== Fixing Unreal Engine Zen Storage Server ==="
echo ""

# Default Zen port is 1337
ZEN_PORT=1337

echo "1. Checking what's using port $ZEN_PORT..."
sudo lsof -i :$ZEN_PORT

echo ""
echo "2. Killing any process using port $ZEN_PORT..."
sudo fuser -k $ZEN_PORT/tcp

echo ""
echo "3. Looking for UnrealZenServer processes..."
ps aux | grep -i "UnrealZenServer" | grep -v grep

echo ""
echo "4. Killing any UnrealZenServer processes..."
pkill -f "UnrealZenServer"

echo ""
echo "5. Clearing UE5 cache and temp files..."
# Clear common UE5 cache locations
rm -rf ~/.config/Epic/UnrealEngine/Common/DerivedDataCache/*
rm -rf ~/.config/Epic/UnrealEngine/*/Saved/Config/CrashReportClient/*

echo ""
echo "6. Final check - port should be free now:"
sudo lsof -i :$ZEN_PORT

echo ""
echo "=== Done! ==="
echo ""
echo "You can now start Unreal Engine 5."
echo "If the issue persists, try:"
echo "  1. Restart your computer"
echo "  2. In UE5: Edit > Editor Preferences > General > Global"
echo "     - Disable 'Use Zen Store Server' temporarily"
echo ""