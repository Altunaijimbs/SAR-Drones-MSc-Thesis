# SAR-Drones MSc Thesis Project
**Autonomous Search and Rescue Drone System using ROS2**

## 🎯 Project Overview
This repository contains the implementation of an autonomous Search and Rescue (SAR) drone system developed as part of an MSc thesis. The system uses ROS2, PX4 autopilot, AirSim simulator, and Unreal Engine 5 to create a comprehensive SAR solution with pattern-based search capabilities.

### Key Features
- 🚁 Autonomous pattern-based search (square, spiral, zigzag)
- 🎮 Multiple control modes (manual, semi-autonomous, fully autonomous)
- 👁️ Computer vision integration with YOLO for victim detection
- 🗺️ Real-time position visualization and tracking
- 🏠 Return-to-home (RTH) functionality
- 🌐 Web-based control interface
- 🤖 Natural language control via LLM integration

## 📁 Repository Structure
```
SAR-Drones-MSc-Thesis/
├── ros2_ws/                    # Main ROS2 workspace
│   └── src/
│       ├── search_patterns/    # Pattern generation and execution
│       ├── llm_controller/     # Natural language control
│       └── sar_web_platform/   # Web interface
├── airsim_native_workspace/    # SimpleFlight mode scripts
├── Updates/                     # Progress reports and documentation
├── launch_*.sh                 # System launch scripts
└── test_*.sh                   # Testing scripts
```

## 🚀 Quick Start

### Prerequisites
- Ubuntu 22.04 LTS
- ROS2 Humble
- PX4-Autopilot
- AirSim (Cosys fork)
- Unreal Engine 5
- Python 3.10+

### Installation
```bash
# Clone the repository
git clone https://github.com/altunaijimbs/SAR-Drones-MSc-Thesis.git
cd SAR-Drones-MSc-Thesis

# Build ROS2 workspace
cd ros2_ws
colcon build
source install/setup.bash
```

### Running the System

#### Option 1: Hybrid System (Recommended)
```bash
# Terminal 1: Launch complete system
./launch_hybrid_system.sh

# Terminal 2: Arm and takeoff
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once

# Terminal 3: Run pattern
./pattern_control_center.sh
```

#### Option 2: SimpleFlight Mode (No PX4 Required)
```bash
cd airsim_native_workspace
./START_HERE.sh
# Select desired option from menu
```

## 🎮 Control Modes

### Manual Control
```bash
ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once
ros2 topic pub /simple_command std_msgs/msg/String "data: 'left'" --once
ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once
```

### Pattern Execution
```bash
# Load pattern
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:10,2'" --once

# Start execution
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
```

### Available Patterns
- `expanding_square:SIZE,LAYERS` - Square search pattern
- `spiral:RADIUS,SPACING` - Spiral search pattern
- `zigzag:WIDTH,LENGTH,SPACING` - Zigzag coverage pattern

## 📊 Visualization
```bash
# Launch standalone visualizer
./launch_visualizer.sh

# Or quick map
./quick_map.sh
```

## 🌐 Web Interface
Access at `http://localhost:5000` after launching the system.

Features:
- Live camera feed
- Manual control buttons
- Pattern selection
- RTH activation
- System status

## 🤖 Pattern Executors

### Available Executors
1. **fixed_movement_executor** - No yaw control, pure translation
2. **precision_pattern_executor** - Variable speed control
3. **coordinated_banking_executor** - Works with velocity coordinator
4. **improved_pattern_executor** - Enhanced turn handling

### Testing Executors
```bash
./executor_comparison_fixed.sh
```

## 📚 Documentation
- `CLAUDE.md` - Project memory and progress tracking
- `PATTERN_EXECUTION_PROBLEM.txt` - Solution to rotation vs movement issue
- `MANUAL_PATTERN_TESTING.md` - Complete testing guide
- `Updates/` - Weekly progress reports

## 🔧 Troubleshooting

### Drone not moving?
```bash
# Check system status
./diagnose_movement.sh

# Monitor velocity commands
ros2 topic echo /search_pattern/velocity_command
```

### Pattern not executing?
```bash
# Check waypoints
ros2 topic echo /pattern_waypoints

# Check status
ros2 topic echo /pattern_status
```

## 🎯 Key Achievements
- ✅ Autonomous pattern-based search
- ✅ Web-based control interface
- ✅ Computer vision integration
- ✅ Multiple executor implementations
- ✅ Real-time visualization
- ✅ Natural language control

## 📝 Citation
If you use this work in your research, please cite:
```
Mohamed Alteneiji. (2025). Autonomous Search and Rescue Drone System. 
MSc Thesis, University of Birmingham.
```

## 📄 License
This project is part of an MSc thesis. Please contact the author for usage permissions.

👤 Author
Mohamed Alteneiji

MSc AI and CS Student

University Of Birmingham

Mohamed.alteneijiii@gmail.com

🙏 Acknowledgments
Supervisor: Sauleh Eetemadi
Institution: University of Birmingham
Framework: ROS2, PX4, AirSim, UE5
Thesis Submission Deadline: Sep 8, 2025
Status: Final testing and documentation phase
