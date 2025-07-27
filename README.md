# Autonomous Vehicle System - ROS 2 Workspace

## Overview

This repository contains a complete ROS 2 workspace for an autonomous vehicle system featuring SLAM (Simultaneous Localization and Mapping), navigation, and hardware control capabilities. The system is designed to work with a Raspberry Pi-based robot equipped with RPLIDAR sensor, motor drivers, and various sensors for autonomous navigation and mapping.

## System Architecture

### Hardware Components
- **Raspberry Pi**: Main computing unit
- **RPLIDAR**: Laser scanner for environment perception
- **LiFePO₄ Battery**: Power supply for all components
- **L298N Motor Driver**: Controls robot movement
- **Various Sensors**: Additional sensors for enhanced perception

### Software Stack
- **ROS 2**: Robot Operating System framework
- **Cartographer SLAM**: Real-time SLAM implementation
- **NAV2**: Navigation stack for autonomous movement
- **RViz2**: 3D visualization tool
- **Custom Packages**: Robot-specific implementations

## Package Structure

```
src/
├── articubot_one/              # Main robot package with launch files and configurations
├── articubot_gpio_driver/      # GPIO driver for hardware control
├── autonomous_exploration/     # Autonomous exploration algorithms
├── cartographer_slam/          # Cartographer SLAM configuration
├── navigation/                 # Navigation stack configuration
├── rplidar_ros/               # RPLIDAR driver package
├── sllidar_ros2/              # Alternative LIDAR driver
├── tf_broadcaster_pkg/         # Transform broadcasting utilities
└── my_slam_toolbox/           # SLAM toolbox configuration
```

## Prerequisites

### System Requirements
- Ubuntu 20.04 or 22.04
- ROS 2 Humble or Foxy
- Python 3.8+
- Colcon build tools

### Dependencies Installation

```bash
# Install ROS 2 (if not already installed)
sudo apt update
sudo apt install ros-humble-desktop-full

# Install additional ROS 2 packages
sudo apt install ros-humble-navigation2 \
                 ros-humble-nav2-bringup \
                 ros-humble-cartographer \
                 ros-humble-cartographer-ros \
                 ros-humble-teleop-twist-keyboard \
                 ros-humble-tf2-tools \
                 ros-humble-robot-state-publisher \
                 ros-humble-joint-state-publisher

# Install build tools
sudo apt install python3-colcon-common-extensions
```

## Installation and Setup

### 1. Clone and Build the Workspace

```bash
# Navigate to the workspace directory
cd /path/to/ros2_ws

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

### 2. Hardware Setup

1. **Power Connections**:
   - Connect the LiFePO₄ battery to power all components
   - Ensure proper voltage levels for each component

2. **Sensor Connections**:
   - Connect RPLIDAR to USB port
   - Verify GPIO connections for motor driver (L298N)
   - Check all sensor connections

3. **Verification**:
   - Confirm operational status via indicator LEDs
   - Test LIDAR rotation and data output
   - Verify motor driver functionality

## Usage

### Basic Robot Operation

#### Terminal 1: Launch Robot Base
```bash
source install/setup.bash
ros2 launch articubot_one launch_robot.launch.py
```

#### Terminal 2: Launch RPLIDAR
```bash
source install/setup.bash
ros2 launch articubot_one rplidar_launch.py
```

#### Terminal 3: Launch Navigation Stack
```bash
source install/setup.bash
ros2 launch navigation navigation.launch.py
```

#### Terminal 4: Launch Visualization
```bash
source install/setup.bash
rviz2
```

### SLAM Mapping Mode

For creating maps using Cartographer SLAM:

#### Terminal 1: Robot Base
```bash
source install/setup.bash
ros2 launch articubot_one launch_robot.launch.py
```

#### Terminal 2: Keyboard Teleoperation
```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Terminal 3: Static Transform (map → odom)
```bash
source install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
```

#### Terminal 4: Cartographer SLAM
```bash
source install/setup.bash
ros2 launch cartographer_slam cartographer.launch.py
```

#### Terminal 5: Static Transform (base_link → laser_frame)
```bash
source install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser_frame
```

#### Terminal 6: Visualization
```bash
source install/setup.bash
rviz2
```

### Mapping Process

1. **Start SLAM Mode**: Follow the SLAM mapping setup above
2. **Manual Control**: Use keyboard teleoperation to drive the robot
3. **Map Building**: Drive around the environment to build a complete map
4. **Save Map**: Use the map server to save the generated map

```bash
# Save the map
ros2 run nav2_map_server map_saver_cli -f my_map
```

### Autonomous Navigation

1. **Load Saved Map**: Ensure you have a previously saved map
2. **Launch Navigation**: Use the basic robot operation setup
3. **Set Navigation Goals**: Use RViz2 to set navigation goals
4. **Monitor Progress**: Watch the robot navigate autonomously

## Configuration

### Robot Parameters
- Modify robot-specific parameters in `src/articubot_one/config/`
- Adjust LIDAR parameters in the respective launch files
- Configure navigation parameters in `src/navigation/config/`

### SLAM Configuration
- Cartographer settings: `src/cartographer_slam/config/`
- SLAM Toolbox settings: `src/my_slam_toolbox/config/`

### Navigation Tuning
- Costmap parameters: `src/navigation/config/nav2_params.yaml`
- Planner configurations: Modify planner-specific parameters
- Controller settings: Adjust for robot dynamics

## Troubleshooting

### Common Issues

1. **LIDAR Not Detected**:
   ```bash
   # Check USB permissions
   sudo chmod 666 /dev/ttyUSB0
   
   # Or create udev rules
   cd src/rplidar_ros/scripts
   sudo ./create_udev_rules.sh
   ```

2. **Transform Errors**:
   ```bash
   # Check transform tree
   ros2 run tf2_tools view_frames
   
   # Verify static transforms are running
   ros2 topic echo /tf_static
   ```

3. **Navigation Issues**:
   - Verify map is loaded correctly
   - Check costmap configurations
   - Ensure proper localization

4. **Build Errors**:
   ```bash
   # Clean build
   rm -rf build/ install/ log/
   colcon build --symlink-install
   ```

### Debug Commands

```bash
# Check node status
ros2 node list

# Monitor topics
ros2 topic list
ros2 topic echo /scan

# Check transforms
ros2 run tf2_ros tf2_echo map base_link

# View computation graph
rqt_graph
```

## Development

### Adding New Features

1. **Create New Package**:
   ```bash
   cd src/
   ros2 pkg create --build-type ament_python my_new_package
   ```

2. **Modify Existing Packages**:
   - Follow ROS 2 package structure
   - Update `package.xml` and `setup.py`/`CMakeLists.txt`
   - Rebuild workspace after changes

3. **Testing**:
   ```bash
   # Run tests
   colcon test
   
   # Check test results
   colcon test-result --verbose
   ```

## Performance Optimization

### System Performance
- Monitor CPU and memory usage during operation
- Adjust node priorities if needed
- Consider distributed computing for complex scenarios

### Navigation Performance
- Tune costmap resolution based on environment
- Optimize planner parameters for specific use cases
- Adjust controller gains for smooth movement

## Safety Considerations

- Always test in a safe, controlled environment
- Implement emergency stop mechanisms
- Monitor battery levels during operation
- Ensure proper hardware connections before operation

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Acknowledgments

- ROS 2 community for the excellent framework
- Cartographer team for SLAM implementation
- Navigation2 team for the navigation stack
- RPLIDAR manufacturers for sensor drivers

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review ROS 2 documentation
3. Open an issue in the repository
4. Contact the maintainers

---

**Note**: This system is designed for educational and research purposes. Always follow safety protocols when operating autonomous vehicles.