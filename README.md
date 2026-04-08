# Zaybot - Autonomous Mobile Robot Navigation with ROS 2

> **ROS 2 Humble · Gazebo Harmonic · SLAM Toolbox · Nav2 · Differential Drive**
>
> A fully autonomous navigation system built from scratch: SLAM mapping, AMCL localization, and Nav2 path planning - all running in a physics-accurate Gazebo simulation. Designed for industrial material transport use cases with a clear path toward real-hardware deployment.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Robot Specifications](#robot-specifications)
- [Requirements](#requirements)
- [Installation](#installation)
- [Build](#build)
- [Usage](#usage)
  - [SLAM Mapping](#slam-mapping)
  - [Autonomous Navigation](#autonomous-navigation)
  - [Useful Commands](#useful-commands)
- [Project Structure](#project-structure)
- [Key Design Decisions & Debugging Notes](#key-design-decisions--debugging-notes)
- [Future Work](#future-work)

---

## Overview

Zaybot implements a complete autonomous navigation stack on a differential-drive mobile robot. The system solves the core robotics challenge of **simultaneous localization and mapping (SLAM)**: robot explores an unknown environment, builds a 2D occupancy grid in real time, then uses that map to plan and execute collision-free paths to goal poses.

**What the system does end-to-end:**

1. **Mapping** - SLAM Toolbox builds a 2D occupancy grid from LiDAR scans, running pose-graph optimization and loop closure to keep the map consistent over time.
2. **Localization** - Once a map is saved, AMCL estimates the robot's pose within that map using a particle filter fused with odometry.
3. **Planning & Control** - Nav2's NavFn global planner (Dijkstra) computes a collision-free path; DWB local controller executes it in real time, reacting to dynamic obstacles via layered costmaps.
4. **Simulation** - Gazebo Harmonic simulates realistic physics (friction, inertia, sensor noise) so that tuned parameters translate meaningfully to real hardware.

This project was completed as part of a remote internship at **Autobonics (Bonic.ai)**, December 2025 – April 2026.

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        ROS 2 Humble                             │
│                                                                 │
│  Gazebo Harmonic  ──►  LiDAR + Odometry                        │
│                               │                                 │
│                    ┌──────────▼──────────┐                      │
│                    │    SLAM Toolbox     │  (mapping mode)      │
│                    │  async graph-based  │                      │
│                    │  2D occupancy grid  │                      │
│                    └──────────┬──────────┘                      │
│                               │  /map                           │
│         ┌─────────────────────▼──────────────────────┐         │
│         │               Nav2 Stack                   │         │
│         │  AMCL localization  │  NavFn global planner│         │
│         │  DWB local planner  │  Layered costmaps    │         │
│         │  Recovery behaviors │  MPC smoothing       │         │
│         └─────────────────────┬──────────────────────┘         │
│                               │  /cmd_vel                       │
│                         Robot actuators                         │
└─────────────────────────────────────────────────────────────────┘
```

### Transform Tree (TF)

```
map → odom → base_footprint → base_link → { LiDAR, IMU, left_wheel, right_wheel, caster_wheel }
```

- `odom → base_link` - published by the `odom_to_tf` node from Gazebo ground-truth odometry
- `map → odom` - published by AMCL to correct accumulated odometric drift

Getting this TF tree right is critical. Any incorrect frame breaks the entire navigation stack.

---

### SLAM

SLAM Toolbox runs in **asynchronous mode**, processing:
- LiDAR scans at **5.5 Hz**
- Odometry at **50 Hz**

It performs pose-graph optimization with loop closure detection to produce a consistent map even after the robot revisits areas.

### Navigation (Nav2)

| Component | Implementation |
|-----------|---------------|
| Global planner | NavFn (Dijkstra) |
| Local controller | DWB (Dynamic Window Approach) + custom MPC plugin |
| Costmaps | Obstacle layer + inflation layer |
| Recovery behaviors | Spin, backup, wait |

---

## Robot Specifications

| Parameter | Value |
|-----------|-------|
| Wheel radius | 0.0381 m |
| Wheel separation | 0.1725 m |
| Total mass | 5.525 kg |
| Max linear velocity | 0.26 m/s |
| Max angular velocity | 1.0 rad/s |
| Max linear acceleration | 2.5 m/s² |
| Max angular acceleration | 3.2 rad/s² |

**Differential drive kinematics:**

$$v = \frac{r(\omega_R + \omega_L)}{2}, \qquad \omega = \frac{r(\omega_R - \omega_L)}{W}$$

where $r$ is wheel radius, $W$ is wheel separation, and $\omega_{R/L}$ are the right/left wheel angular velocities.

---

## Requirements

### Software

| Tool | Version |
|------|---------|
| OS | Ubuntu 22.04 LTS |
| ROS 2 | Humble Hawksbill |
| Gazebo | Harmonic (gz-sim 8) |
| Python | 3.8+ |
| CMake | 3.16+ |

### ROS 2 Packages

```
ros-humble-ros-gz                   # Gazebo Harmonic bridge
ros-humble-navigation2              # Full Nav2 stack
ros-humble-nav2-bringup
ros-humble-slam-toolbox             # SLAM mapping
ros-humble-rviz2                    # Visualization
ros-humble-teleop-twist-keyboard    # Keyboard teleoperation
ros-humble-robot-state-publisher    # URDF → TF publishing
ros-humble-joint-state-publisher
ros-humble-xacro                    # URDF macro processing
ros-humble-tf2-tools                # TF debugging
```

---

## Installation

### 1. Install ROS 2 Humble

```bash
# Configure locale
sudo apt update && sudo apt install locales curl
sudo locale-gen en_US en_US.UTF-8

# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install
sudo apt update && sudo apt upgrade
sudo apt install ros-humble-desktop
```

### 2. Install Gazebo Harmonic

```bash
sudo apt install gz-harmonic ros-humble-ros-gz
```

### 3. Install Project Dependencies

```bash
sudo apt install -y \
  python3-colcon-common-extensions python3-rosdep python3-argcomplete \
  ros-humble-navigation2 ros-humble-nav2-bringup \
  ros-humble-slam-toolbox ros-humble-rviz2 \
  ros-humble-teleop-twist-keyboard ros-humble-joy \
  ros-humble-robot-state-publisher ros-humble-joint-state-publisher \
  ros-humble-xacro ros-humble-tf2-tools

sudo rosdep init && rosdep update
```

### 4. Clone Repository

```bash
mkdir -p ~/ros2/zaybot_ws/src
cd ~/ros2/zaybot_ws/src
git clone https://github.com/zaya1d3/Zaybot_robot.git .
cd ~/ros2/zaybot_ws
```

---

## Build

```bash
colcon build --symlink-install
source install/setup.bash
```

To automatically source the workspace in every new terminal:

```bash
echo "source ~/ros2/zaybot_ws/install/setup.bash" >> ~/.bashrc
```

---

## Usage

### SLAM Mapping

**1. Launch the simulation with SLAM Toolbox and RViz:**

```bash
ros2 launch zaybot_bringup slam_bringup.launch.py
```

**2. Drive the robot to explore the environment (choose one):**

```bash
# Keyboard teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or graphical GUI
ros2 launch zaybot_teleop_gui teleop_gui.launch.py
```

**3. Save the map once the environment has been fully explored:**

```bash
ros2 launch zaybot_slam save_map.launch.py
```

The saved map files (`.pgm` + `.yaml`) will appear in `zaybot_navigation/maps/`.

### Autonomous Navigation

> **Requires a previously saved map.**

**1. Launch the navigation stack:**

```bash
ros2 launch zaybot_bringup navigation_bringup.launch.py
```

**2. In RViz2:**
- Click **2D Pose Estimate** → click on the map to set the robot's initial pose (this initializes AMCL)
- Click **2D Goal Pose** → click anywhere on free map space to send a navigation goal
- Watch the global path, local trajectory, and AMCL particle cloud update in real time

### Useful Commands

```bash
# System inspection
ros2 node list                            # All active nodes
ros2 topic list                           # All active topics
ros2 topic hz /scan                       # LiDAR publish rate
ros2 topic echo /cmd_vel                  # Live velocity commands

# TF debugging
ros2 run tf2_ros tf2_echo map base_link   # Transform lookup
ros2 run tf2_tools view_frames            # Generate TF tree PDF

# Node introspection
ros2 node info /slam_toolbox
ros2 param list /controller_server

# Data recording
ros2 bag record -a -o navigation_data     # Record all topics
```

---

## Project Structure

```
Zaybot_robot/
├── src/
│   ├── zaybot_bringup/            # Top-level launch orchestrators
│   │   └── launch/
│   │       ├── slam_bringup.launch.py
│   │       └── navigation_bringup.launch.py
│   │
│   ├── zaybot_description/        # URDF model, meshes, RViz configs
│   │   ├── urdf/
│   │   ├── meshes/
│   │   ├── rviz/
│   │   └── launch/
│   │
│   ├── zaybot_gazebo/             # Gazebo Harmonic simulation
│   │   ├── zaybot_gazebo/
│   │   │   └── odom_to_tf.py      # Odometry → TF broadcaster
│   │   ├── models/zaybot_v2/      # SDF model + meshes
│   │   ├── worlds/                # Simulation world files
│   │   └── launch/
│   │       └── simulation.launch.py
│   │
│   ├── zaybot_slam/               # SLAM Toolbox configuration
│   │   ├── config/
│   │   │   └── slam_params.yaml
│   │   ├── rviz/
│   │   └── launch/
│   │       ├── slam.launch.py
│   │       └── save_map.launch.py
│   │
│   ├── zaybot_navigation/         # Nav2 configuration and maps
│   │   ├── config/
│   │   │   └── nav2_params.yaml
│   │   ├── maps/
│   │   ├── rviz/
│   │   └── launch/
│   │       └── navigation.launch.py
│   │
│   └── zaybot_teleop_gui/         # PyQt5 teleoperation GUI
│       ├── zaybot_teleop_gui/
│       │   ├── main.py
│       │   ├── main_window.py
│       │   ├── ros_node.py
│       │   └── widgets/
│       │       ├── keyboard_mode.py
│       │       ├── joystick_mode.py
│       │       └── slider_mode.py
│       └── launch/
│           └── teleop_gui.launch.py
│
├── documentacion/
│   └── modelo-matematico/         # Kinematic and control derivations
├── images/                        # Documentation images
└── README.md
```

---

## Key Design Decisions & Debugging Notes

This section documents non-obvious configuration choices and bugs encountered - useful if you're adapting this stack.

### TF Tree Configuration

The single most common source of breakage in this stack is an incorrect TF tree. If the robot spins in circles or the map doesn't align, check the `odom → base_link` transform first. Use `ros2 run tf2_tools view_frames` to generate a live diagram and verify the full chain `map → odom → base_link → sensors`.

### Costmap Inflation Radius

```yaml
inflation_layer:
  plugin: "nav2_costmap_2d::InflationLayer"
  inflation_radius: 1.0       # Tune carefully - too high and planner sees phantom blockages
  cost_scaling_factor: 3.0
```

An inflation radius that's too large causes the global planner to treat navigable corridors as blocked. Start with `0.3`–`0.5` m and increase only as needed for your environment's geometry.

### DWB Local Planner Tuning

```yaml
FollowPath:
  plugin: "dwb_core::DWBLocalPlanner"
  max_vel_x: 0.22
  max_vel_theta: 1.0
  acc_lim_x: 2.5
  acc_lim_theta: 3.2
  sim_time: 1.5
```

High acceleration limits with a short `sim_time` cause jerky overcorrections. Increasing `sim_time` to `1.5`+ gives the planner enough lookahead to smooth out the trajectory. A custom MPC plugin was also integrated to predict future states for smoother motion.

### SLAM Loop Closure Consistency

If the map drifts after revisiting areas, the scan-matching parameters need tightening. In `slam_params.yaml`, reduce `minimum_travel_distance` and `minimum_travel_heading` thresholds so that the toolbox re-evaluates the pose graph more aggressively on revisit.

### Gazebo Rendering Issues

On machines without discrete GPU support, Gazebo may freeze or RViz may display a blank map. Fix: set the `LIBGL_ALWAYS_SOFTWARE=1` environment variable before launching to force software rendering. Slightly slower but stable.

---

## Future Work

- **Hardware deployment** - Port the full stack to the physical Zaybot platform, replacing Gazebo sensors with real LiDAR and IMU hardware
- **Computer vision** - Integrate a depth camera and object detection pipeline so the robot can semantically label its environment, not just geometrically avoid it
- **3D navigation** - Upgrade from a 2D occupancy grid to a 3D voxel map (e.g. OctoMap) for environments with ramps or multi-level structures
- **Fleet coordination** - Extend toward multi-robot task allocation for full industrial logistics coverage
- **Real-world mobility** - Apply this navigation stack to autonomous scooters and personal mobility devices operating in crowded pedestrian environments

---

## Acknowledgements

Built during a remote internship at **Autobonics (Bonic.ai)**.

Developed independently using ROS 2 documentation, Nav2 documentation, SLAM Toolbox wiki, and community forums.
