# RB-Y1 Robot Implementation

ROS 2 Jazzy implementation of the Rainbow Robotics RB-Y1 humanoid robot with Gazebo simulation and control capabilities.

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
  - [Gazebo Simulation](#gazebo-simulation)
  - [URDF Visualization](#urdf-visualization)
  - [Trajectory Publishing](#trajectory-publishing)
- [Robot Specifications](#robot-specifications)
- [Configuration Files](#configuration-files)
- [Attribution](#attribution)
- [License](#license)

## Features

- **URDF Model**: Full robot description with original RB-Y1 meshes
- **Dual Arm Manipulation**: 7-DOF arms with gripper control
- **Mobile Base**: Differential drive system with odometry
- **Sensors**:
  - 2D LIDAR (360° coverage, 10m range)
- **Torso Control**: 6-DOF articulated torso
- **Head Control**: 2-DOF pan-tilt head
- **Gazebo Integration**: Simulation with Gazebo Harmonic
- **ROS 2 Control**: Joint trajectory controllers for all joints

## Prerequisites

- ROS 2 Jazzy
- Gazebo Harmonic
- Python 3.12+
- Required ROS 2 packages:
```bash
  sudo apt install ros-jazzy-ros-gz-sim \
                   ros-jazzy-ros-gz-bridge \
                   ros-jazzy-ros2-control \
                   ros-jazzy-ros2-controllers \
                   ros-jazzy-joint-state-publisher \
                   ros-jazzy-robot-state-publisher \
                   ros-jazzy-xacro \
                   ros-jazzy-gz-ros2-control \
                   ros-jazzy-ros-gz-bridge
```

## Installation

1. **Clone the repository**:
```bash
   mkdir -p ~/rby_ws/src
   cd ~/rby_ws/src
   git clone https://github.com/angysof16/rby1_robot_implementation.git rby
```

2. **Build the workspace**:
```bash
   cd ~/rby_ws
   colcon build
   source install/setup.bash
```

## Usage

### Gazebo Simulation

Launch the complete robot in Gazebo with all controllers:
```bash
ros2 launch rby_gazebo gazebo.launch.py
```

### URDF Visualization

View the robot model in RViz without Gazebo:
```bash
ros2 launch rby_description display.launch.py
```

### Trajectory Publishing

Execute predefined trajectories for the robot arms:
```bash
ros2 launch rby_gazebo rby_trayectory_publish.launch.py
```

Positions defined in `rby_position_publisher.yaml`.

## Robot Specifications

### Degrees of Freedom
- **Torso**: 6 DOF
- **Right Arm**: 7 DOF + 2 DOF gripper
- **Left Arm**: 7 DOF + 2 DOF gripper
- **Head**: 2 DOF (pan-tilt)
- **Mobile Base**: Differential drive
- **Total**: 26 controllable joints

### Joint Controllers
All joints are controlled via `JointTrajectoryController`:
- `joint_trajectory_controller_head`
- `joint_trajectory_controller_torso`
- `joint_trajectory_controller_left_arm`
- `joint_trajectory_controller_right_arm`
- `joint_trajectory_controller_left_gripper`
- `joint_trajectory_controller_right_gripper`

### Sensors
- **LIDAR**: 360° scan, 10Hz update rate, 0.12-10m range

### Base Control
- **Topic**: `/cmd_vel` (geometry_msgs/TwistStamped)
- **Odometry**: `/odom` (nav_msgs/Odometry)
- **Wheel Separation**: 0.48m
- **Wheel Radius**: 0.1m

## Configuration Files

### Controller Manager
`rby_gazebo/config/rby_controller_manager.yaml`
- Defines all joint trajectory controllers
- Update rate: 1000 Hz
- Command interfaces: position
- State interfaces: position, velocity

### ROS-Gazebo Bridge
`rby_gazebo/config/ros_gz_bridge.yaml`
- Bridges topics between ROS 2 and Gazebo
- Handles sensor data, joint states, and odometry

### Position Publisher
`rby_gazebo/config/rby_position_publisher.yaml`
- Defines named poses for trajectory execution
- Configurable timing between waypoints
- Currently configured for right arm wave motion

## Attribution

This project uses meshes and resources from Rainbow Robotics RB-Y1 robot.

**Original Work**: Copyright 2024-2025 Rainbow Robotics  
**License**: Apache License 2.0

**Includes**:
- DynamixelSDK under Apache License 2.0
- Original RB-Y1 CAD meshes and visual assets

## License

This project is licensed under the **Apache License 2.0** - see the [LICENSE](LICENSE) file for details.