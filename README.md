# Autonomous Target-Based Docking System

An autonomous docking system developed for the TurtleBot3 Waffle Pi using ROS 2 Humble, OpenCV, Gazebo Classic, and 2D LiDAR.

## Overview

This project was developed as part of the **Robotic Systems** course at PES University.

The objective is to enable a TurtleBot3 Waffle Pi robot to autonomously detect a target, align itself using visual feedback, and safely dock without human intervention.

The system combines computer vision and LiDAR-based sensing to achieve reliable target detection and collision-free docking in a simulated Gazebo environment.

---

## Repository Contents

This repository contains only the custom docking node developed for this project.

The TurtleBot3 simulation environment, ROS 2 workspace, and Gazebo packages are **not included** in this repository and must be installed separately.

To run this project, users are expected to:

- Install ROS 2 Humble
- Install TurtleBot3 packages
- Create and build a ROS 2 workspace
- Create a ROS 2 Python package
- Place the provided docking script inside the package
- Build the workspace using `colcon build`

This approach keeps the repository lightweight while focusing on the autonomous docking logic developed as part of the project.

---

## Features

- Sphere detection using Hough Circle Transform
- Square detection using contour-based filtering
- Aspect Ratio and Extent filtering to reject shadow-induced false detections
- Proportional (P) controller for target alignment
- Lock-State navigation for stable final approach
- LiDAR-based safety stop at 0.45 m
- Implemented in ROS 2 Humble
- Tested in Gazebo Classic with TurtleBot3 Waffle Pi

---

## System Architecture

```text
Camera Feed
     │
     ▼
OpenCV Detection
     │
     ▼
Target Centroid Calculation
     │
     ▼
P-Controller Alignment
     │
     ▼
Lock-State Navigation
     │
     ▼
LiDAR Safety Check
     │
     ▼
Docking Complete
```

---

## Technologies Used

- ROS 2 Humble
- TurtleBot3 Waffle Pi
- Gazebo Classic
- OpenCV
- NumPy
- Python 3
- 2D LiDAR

---

## Usage

### Prerequisites

Before running the project, ensure that:

- ROS 2 Humble is installed
- TurtleBot3 packages are installed
- A ROS 2 workspace has been created and built
- The docking node from this repository has been added to a ROS 2 package

### Terminal 1: Launch TurtleBot3 Simulation

```bash
export TURTLEBOT3_MODEL=waffle_pi

ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Terminal 2: Source the Workspace and Run the Docking Node

```bash
cd ~/your_ros2_workspace

source install/setup.bash

ros2 run <package_name> <executable_name>
```

Example:

```bash
ros2 run autonomous_docking robust_docking
```

---

## How It Works

### 1. Target Detection

The onboard camera continuously captures images from the environment.

- **Sphere Detection:** Hough Circle Transform
- **Square Detection:** Contour-based filtering using:
  - Aspect Ratio
  - Extent (Contour Area / Bounding Box Area)

These filters help eliminate false detections caused by shadows and simulation artifacts.

### 2. Robot Alignment

The detected target centroid is compared with the center of the camera frame.

A Proportional (P) Controller computes the angular velocity:

```text
ω = Kp × error
```

allowing the robot to smoothly align itself with the target.

### 3. Lock State

When the alignment error falls below a predefined threshold, the robot enters a **Lock State** and moves straight toward the docking station, reducing oscillations during the final approach.

### 4. LiDAR Safety Layer

A 2D LiDAR continuously monitors the robot's forward region.

If an obstacle is detected within **0.45 m**, the robot immediately stops, ensuring safe and collision-free docking.

---

## Conclusion

The system successfully:

- Detected spherical targets using Hough Circle Transform
- Detected square targets using geometric filtering
- Eliminated shadow-induced false positives
- Aligned itself using visual feedback
- Executed a stable final approach using Lock State navigation
- Performed collision-free docking using LiDAR-based safety monitoring

---

