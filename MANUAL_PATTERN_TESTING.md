# Manual Pattern Testing Guide
*Complete step-by-step instructions for testing patterns*

## 🚀 Quick Start (3 Terminal Method)

### Terminal 1: Launch System
```bash
cd /home/mbs/SAR-Drones-MSc-Thesis
./launch_hybrid_system.sh
# Wait for all nodes to start (about 30 seconds)
```

### Terminal 2: Pattern System
```bash
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Start pattern generator
ros2 run search_patterns pattern_generator &

# Start your chosen executor (pick ONE):
ros2 run search_patterns coordinated_banking_executor    # Best for hybrid system
# OR
ros2 run search_patterns precision_pattern_executor      # Your previous best (5/10)
# OR  
ros2 run search_patterns improved_pattern_executor       # Basic improved version
```

### Terminal 3: Control Terminal
```bash
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash

# Arm and takeoff
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once

# Wait for drone to reach ~5m altitude, then run patterns...
```

---

## 📊 Available Patterns

### 1. Expanding Square
```bash
# Small square (5m sides, 1 layer)
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:5,1'" --once

# Medium square (10m sides, 2 layers)
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:10,2'" --once

# Large square (15m sides, 3 layers)
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:15,3'" --once
```

### 2. Spiral
```bash
# Small spiral (8m radius, 2m spacing)
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:8,2'" --once

# Medium spiral (12m radius, 3m spacing)
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:12,3'" --once

# Large spiral (20m radius, 5m spacing)
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:20,5'" --once
```

### 3. Zigzag
```bash
# Small zigzag (10m width, 10m length, 3m spacing)
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:10,10,3'" --once

# Medium zigzag (15m width, 15m length, 5m spacing)
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:15,15,5'" --once

# Large zigzag (20m width, 20m length, 5m spacing)
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:20,20,5'" --once
```

### 4. Lawnmower (if available)
```bash
# Small area (10m x 10m, 3m spacing)
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'lawnmower:10,10,3'" --once

# Medium area (15m x 15m, 4m spacing)
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'lawnmower:15,15,4'" --once
```

---

## 🎮 Pattern Control Commands

### Start Pattern
```bash
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
```

### Pause Pattern
```bash
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'pause'" --once
```

### Resume Pattern
```bash
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'resume'" --once
```

### Stop Pattern
```bash
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
```

---

## 📺 Add Visualization (Optional 4th Terminal)

### Option 1: Simple Grid Visualizer
```bash
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
ros2 run search_patterns simple_grid_visualizer
```

### Option 2: Background Visualizer
```bash
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
ros2 run search_patterns background_visualizer
```

### Option 3: Terminal Map (No GUI)
```bash
source /home/mbs/SAR-Drones-MSc-Thesis/ros2_ws/install/setup.bash
ros2 run search_patterns terminal_map
```

---

## 🔍 Monitoring Commands

### Check Pattern Status
```bash
ros2 topic echo /pattern_status
```

### Watch Current Waypoints
```bash
ros2 topic echo /pattern_waypoints
```

### Monitor Velocity Coordinator (if using hybrid system)
```bash
ros2 topic echo /velocity_coordinator/active_source
```

### Check Drone Position
```bash
ros2 topic echo /mavros/local_position/pose --once
```

---

## 📝 Complete Test Sequence Example

```bash
# Terminal 3: Full test sequence

# 1. Takeoff
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once
sleep 5  # Wait for altitude

# 2. Test small square
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:5,1'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
sleep 30  # Let it run
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
sleep 3

# 3. Test spiral
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:10,3'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
sleep 40  # Let it run
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once
sleep 3

# 4. Test zigzag
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:12,12,4'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
sleep 35  # Let it run
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'stop'" --once

# 5. Return and land
ros2 topic pub /simple_command std_msgs/msg/String "data: 'stop'" --once
sleep 2
ros2 topic pub /simple_command std_msgs/msg/String "data: 'down'" --once
```

---

## 🎯 Testing Different Executors

### Compare Executors Side-by-Side

1. **Test precision_pattern_executor** (your baseline - 5/10)
```bash
pkill -f pattern_executor
ros2 run search_patterns precision_pattern_executor &
# Run test pattern
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:8,1'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
# Observe performance
```

2. **Test coordinated_banking_executor** (new, works with hybrid)
```bash
pkill -f pattern_executor
ros2 run search_patterns coordinated_banking_executor &
# Run same pattern
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:8,1'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
# Compare performance
```

3. **Test improved_pattern_executor** (4/10 in your test)
```bash
pkill -f pattern_executor
ros2 run search_patterns improved_pattern_executor &
# Run same pattern
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:8,1'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
# Compare performance
```

---

## 🚨 Troubleshooting

### Drone not moving?
```bash
# Check velocity coordinator status
ros2 topic echo /velocity_coordinator/active_source --once

# Check if commands are being sent
ros2 topic echo /search_pattern/velocity_command --once

# Try manual control
ros2 topic pub /simple_command std_msgs/msg/String "data: 'forward'" --once
```

### Pattern not starting?
```bash
# Check if waypoints were received
ros2 topic echo /pattern_waypoints --once

# Check pattern status
ros2 topic echo /pattern_status --once

# Restart pattern generator
pkill -f pattern_generator
ros2 run search_patterns pattern_generator &
```

### Jittering or oscillation?
```bash
# Make sure using coordinated_banking_executor with hybrid system
# OR use precision_pattern_executor without hybrid system
pkill -f hybrid_position_controller  # Stop hybrid if not needed
```

---

## 💡 Pro Tips

1. **Start with small patterns** (5m) to test safely
2. **Watch the visualizer** to understand movement
3. **Use coordinated_banking_executor** when running hybrid system
4. **Use precision_pattern_executor** when running standalone
5. **Always stop pattern before sending new one**
6. **Monitor /pattern_status** for completion

---

## 🎬 Demo Sequence for Thesis

```bash
# Professional demo sequence
# 1. Start with hover demonstration
ros2 topic pub /simple_command std_msgs/msg/String "data: 'up'" --once
sleep 5

# 2. Show precision square pattern
echo "Demonstrating precision square search pattern..."
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'expanding_square:10,2'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
# Wait for completion

# 3. Show smooth spiral search
echo "Demonstrating spiral search pattern..."
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'spiral:15,3'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
# Wait for completion

# 4. Show systematic area coverage
echo "Demonstrating systematic area coverage..."
ros2 topic pub /pattern_command std_msgs/msg/String "data: 'zigzag:20,20,5'" --once
ros2 topic pub /pattern_control std_msgs/msg/String "data: 'start'" --once
# Wait for completion

# 5. Return to home
echo "Returning to home position..."
ros2 topic pub /rth_command std_msgs/msg/String "data: 'rth'" --once
```