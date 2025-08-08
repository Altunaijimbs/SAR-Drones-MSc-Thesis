# SAR Web Control Platform

A modern web-based control interface for the SAR drone system, providing real-time monitoring and control capabilities.

## Features

- **Real-time Status Monitoring**: Live updates of drone position, mode, and system status
- **Video Streaming**: Live camera feed with object detection overlay
- **Mission Control**: Start/stop search patterns, return to home, emergency stop
- **Manual Control**: Arrow key navigation and altitude control
- **Natural Language Commands**: Send commands in plain English
- **System Logging**: Real-time activity log
- **Responsive Design**: Works on desktop and tablet devices

## Installation

1. Install Python dependencies:
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/src/sar_web_platform
pip install -r requirements.txt
```

2. Build the ROS2 package:
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws
colcon build --packages-select sar_web_platform
source install/setup.bash
```

## Usage

1. Make sure all drone systems are running (PX4, AirSim, MAVROS, etc.)

2. Launch the web server:
```bash
ros2 run sar_web_platform web_server
```

3. Open your web browser and navigate to:
```
http://localhost:5000
```

## Interface Overview

### Quick Actions
- **Arm/Disarm**: Enable or disable drone motors
- **Takeoff/Land**: Automated takeoff to safe altitude and landing

### Mission Control
- **Start Search**: Begin grid search pattern
- **Stop Search**: Halt current search and hover
- **Return Home**: Return to saved home position
- **Emergency Stop**: Immediate stop all operations
- **Set Home**: Save current position as home

### Manual Control
- Use arrow keys or on-screen buttons for movement
- Space bar for stop/hover
- Altitude controls for up/down movement

### Natural Language Control
Type commands like:
- "Search for people in the area"
- "Fly forward 10 meters"
- "Return to base"
- "Circle around current position"

## Keyboard Shortcuts

- **Arrow Keys**: Manual directional control
- **Space**: Stop/hover
- **Ctrl+E**: Emergency stop

## System Architecture

The platform consists of:
- **Flask Backend**: ROS2 node that bridges web requests to ROS2 topics
- **HTML/CSS/JS Frontend**: Modern responsive interface
- **Real-time Updates**: Status polling at 2Hz
- **Video Streaming**: MJPEG stream from drone camera

## API Endpoints

- `GET /api/status` - Get current drone status
- `POST /api/command` - Send control commands
- `GET /api/video_feed` - MJPEG video stream
- `POST /api/launch` - System launch control

## Troubleshooting

1. **Connection Failed**: Ensure all ROS2 nodes are running
2. **No Video Feed**: Check AirSim camera topic is publishing
3. **Commands Not Working**: Verify velocity coordinator is running

## Future Enhancements

- Multi-drone support
- Mission planning interface
- Telemetry recording
- 3D visualization
- Mobile app version