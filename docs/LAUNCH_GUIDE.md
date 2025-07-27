# Launch Guide

This guide provides detailed instructions for launching different modes of the autonomous vehicle system.

## Prerequisites

```bash
# Source the workspace
cd /path/to/ros2_ws
source install/setup.bash
```

## Launch Modes

### 1. Basic Robot Operation

Use this mode for standard autonomous navigation with pre-built maps.

#### Terminal Setup (4 terminals required)

**Terminal 1: Robot Base**
```bash
source install/setup.bash
ros2 launch articubot_one launch_robot.launch.py
```

**Terminal 2: LIDAR Sensor**
```bash
source install/setup.bash
ros2 launch articubot_one rplidar_launch.py
```

**Terminal 3: Navigation Stack**
```bash
source install/setup.bash
ros2 launch navigation navigation.launch.py
```

**Terminal 4: Visualization**
```bash
source install/setup.bash
rviz2
```

### 2. SLAM Mapping Mode

Use this mode to create new maps of unknown environments.

#### Terminal Setup (6 terminals required)

**Terminal 1: Robot Base**
```bash
source install/setup.bash
ros2 launch articubot_one launch_robot.launch.py
```

**Terminal 2: Manual Control**
```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Terminal 3: Map-Odom Transform**
```bash
source install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

**Terminal 4: Cartographer SLAM**
```bash
source install/setup.bash
ros2 launch cartographer_slam cartographer.launch.py
```

**Terminal 5: Base-Laser Transform**
```bash
source install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame
```

**Terminal 6: Visualization**
```bash
source install/setup.bash
rviz2
```

### 3. Autonomous Exploration Mode

Use this mode for autonomous exploration and mapping of unknown environments.

#### Terminal Setup (5 terminals required)

**Terminal 1: Robot Base**
```bash
source install/setup.bash
ros2 launch articubot_one launch_robot.launch.py
```

**Terminal 2: SLAM**
```bash
source install/setup.bash
ros2 launch cartographer_slam cartographer.launch.py
```

**Terminal 3: Navigation**
```bash
source install/setup.bash
ros2 launch navigation navigation.launch.py
```

**Terminal 4: Autonomous Exploration**
```bash
source install/setup.bash
ros2 run autonomous_exploration control
```

**Terminal 5: Visualization**
```bash
source install/setup.bash
rviz2
```

## Launch Scripts

For convenience, you can create shell scripts to automate the launch process:

### Basic Operation Script

Create `scripts/launch_basic.sh`:
```bash
#!/bin/bash
cd /path/to/ros2_ws
source install/setup.bash

# Launch in separate terminals using gnome-terminal
gnome-terminal --tab --title="Robot Base" -- bash -c "ros2 launch articubot_one launch_robot.launch.py; exec bash"
sleep 2
gnome-terminal --tab --title="LIDAR" -- bash -c "ros2 launch articubot_one rplidar_launch.py; exec bash"
sleep 2
gnome-terminal --tab --title="Navigation" -- bash -c "ros2 launch navigation navigation.launch.py; exec bash"
sleep 2
gnome-terminal --tab --title="RViz" -- bash -c "rviz2; exec bash"
```

### SLAM Mapping Script

Create `scripts/launch_slam.sh`:
```bash
#!/bin/bash
cd /path/to/ros2_ws
source install/setup.bash

# Launch SLAM mapping mode
gnome-terminal --tab --title="Robot Base" -- bash -c "ros2 launch articubot_one launch_robot.launch.py; exec bash"
sleep 2
gnome-terminal --tab --title="Teleop" -- bash -c "ros2 run teleop_twist_keyboard teleop_twist_keyboard; exec bash"
sleep 1
gnome-terminal --tab --title="Map-Odom TF" -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom; exec bash"
sleep 1
gnome-terminal --tab --title="Cartographer" -- bash -c "ros2 launch cartographer_slam cartographer.launch.py; exec bash"
sleep 2
gnome-terminal --tab --title="Base-Laser TF" -- bash -c "ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame; exec bash"
sleep 1
gnome-terminal --tab --title="RViz" -- bash -c "rviz2; exec bash"
```

Make scripts executable:
```bash
chmod +x scripts/launch_basic.sh
chmod +x scripts/launch_slam.sh
```

## Troubleshooting Launch Issues

### Common Problems

1. **"Package not found" errors**
   ```bash
   # Rebuild workspace
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Transform errors**
   ```bash
   # Check if static transforms are running
   ros2 topic echo /tf_static
   
   # View transform tree
   ros2 run tf2_tools view_frames
   ```

3. **LIDAR not detected**
   ```bash
   # Check USB permissions
   sudo chmod 666 /dev/ttyUSB0
   
   # Or create udev rules
   cd src/rplidar_ros/scripts
   sudo ./create_udev_rules.sh
   ```

4. **Navigation not working**
   - Ensure map is loaded correctly
   - Check that localization is working
   - Verify costmap configurations

### Debug Commands

```bash
# Check running nodes
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /scan

# Check transforms
ros2 run tf2_ros tf2_echo map base_link

# View computation graph
rqt_graph
```

## Performance Tips

1. **System Resources**: Monitor CPU and memory usage during operation
2. **Network**: Use `ROS_DOMAIN_ID` to avoid interference from other ROS systems
3. **Timing**: Allow 2-3 seconds between launching different components
4. **Hardware**: Ensure adequate power supply for all components

## Safety Reminders

- Always test in a safe, controlled environment
- Keep emergency stop accessible
- Monitor battery levels during operation
- Ensure proper hardware connections before launching