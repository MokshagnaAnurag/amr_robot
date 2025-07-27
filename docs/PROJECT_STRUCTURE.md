# Project Structure

This document outlines the organization of the Autonomous Vehicle ROS 2 workspace.

## Directory Structure

```
ros2_ws/
├── README.md                    # Main project documentation
├── .gitignore                   # Git ignore rules
├── rviz.rviz                   # RViz configuration file
├── config/                     # Global configuration files
├── docs/                       # Project documentation
│   └── PROJECT_STRUCTURE.md    # This file
├── maps/                       # Generated and saved maps
└── src/                        # ROS 2 source packages
    ├── articubot_one/          # Main robot package
    ├── articubot_gpio_driver/  # GPIO hardware driver
    ├── autonomous_exploration/ # Autonomous exploration algorithms
    ├── cartographer_slam/      # Cartographer SLAM configuration
    ├── my_slam_toolbox/        # SLAM Toolbox configuration
    ├── navigation/             # Navigation stack configuration
    ├── rplidar_ros/           # RPLIDAR driver package
    ├── sllidar_ros2/          # Alternative LIDAR driver
    └── tf_broadcaster_pkg/     # Transform broadcasting utilities
```

## Package Descriptions

### Core Robot Packages

#### `articubot_one/`
- **Type**: Main robot package (ament_cmake)
- **Purpose**: Central robot configuration, launch files, and robot description
- **Key Files**:
  - `launch/launch_robot.launch.py` - Main robot launcher
  - `launch/rplidar_launch.py` - LIDAR launcher
  - `description/` - Robot URDF/XACRO files
  - `config/` - Robot-specific configurations

#### `articubot_gpio_driver/`
- **Type**: Hardware driver package (ament_python)
- **Purpose**: GPIO control for motors and sensors
- **Key Files**:
  - `articubot_gpio_driver/` - Python driver modules
  - `plugin_description.xml` - Plugin configuration

### SLAM Packages

#### `cartographer_slam/`
- **Type**: SLAM package (ament_python)
- **Purpose**: Cartographer SLAM implementation and configuration
- **Key Files**:
  - `launch/cartographer.launch.py` - Cartographer launcher
  - `config/` - Cartographer configuration files

#### `my_slam_toolbox/`
- **Type**: SLAM package (ament_cmake)
- **Purpose**: SLAM Toolbox configuration and utilities
- **Key Files**:
  - `launch/` - SLAM Toolbox launch files
  - `config/` - SLAM Toolbox parameters

### Navigation Package

#### `navigation/`
- **Type**: Navigation package (ament_cmake)
- **Purpose**: NAV2 navigation stack configuration
- **Key Files**:
  - `launch/navigation.launch.py` - Navigation launcher
  - `config/nav2_params.yaml` - Navigation parameters
  - `map/` - Map files for navigation
  - `worlds/` - Gazebo world files

### Sensor Packages

#### `rplidar_ros/`
- **Type**: Sensor driver (ament_cmake)
- **Purpose**: RPLIDAR sensor driver and utilities
- **Key Files**:
  - `launch/rplidar_launch.py` - RPLIDAR launcher
  - `src/` - C++ driver source code
  - `scripts/` - Utility scripts

#### `sllidar_ros2/`
- **Type**: Alternative sensor driver (ament_cmake)
- **Purpose**: Alternative SLAMTEC LIDAR driver
- **Key Files**:
  - `launch/` - Various LIDAR model launchers
  - `src/` - C++ driver source code

### Utility Packages

#### `autonomous_exploration/`
- **Type**: Exploration package (ament_python)
- **Purpose**: Frontier-based autonomous exploration
- **Key Files**:
  - `autonomous_exploration/` - Python exploration modules
  - `config/params.yaml` - Exploration parameters

#### `tf_broadcaster_pkg/`
- **Type**: Transform utility (ament_python)
- **Purpose**: Transform broadcasting utilities
- **Key Files**:
  - `tf_broadcaster_pkg/fake_odom_tf.py` - Odometry transform broadcaster

## Build System

- **Build Tool**: colcon
- **Package Types**:
  - `ament_cmake`: C++ packages (articubot_one, navigation, rplidar_ros, sllidar_ros2, my_slam_toolbox)
  - `ament_python`: Python packages (cartographer_slam, autonomous_exploration, tf_broadcaster_pkg, articubot_gpio_driver)

## Configuration Management

### Global Configurations
- `config/` - Workspace-level configurations
- `rviz.rviz` - Default RViz configuration

### Package-Specific Configurations
- Each package maintains its own `config/` directory
- Launch files reference appropriate configuration files

### Maps
- `maps/` - Centralized location for generated maps
- Individual packages may have their own map directories for specific use cases

## Development Guidelines

1. **Package Organization**: Keep packages focused on single responsibilities
2. **Configuration**: Use YAML files for parameters, avoid hardcoding
3. **Launch Files**: Create modular launch files for different system components
4. **Documentation**: Maintain README files in each package
5. **Dependencies**: Clearly specify dependencies in package.xml files

## Build Instructions

```bash
# From workspace root
colcon build --symlink-install
source install/setup.bash
```

## Clean Build

```bash
# Remove build artifacts
rm -rf build/ install/ log/
colcon build --symlink-install
```