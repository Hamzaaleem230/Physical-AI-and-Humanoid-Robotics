# Quickstart Guide

## 1. Install ROS 2 Humble

Follow the instructions on the official ROS 2 website to install ROS 2 Humble on your computer.

## 2. Install Gazebo

Follow the instructions on the official Gazebo website to install Gazebo on your computer.

## 3. Install NVIDIA Isaac Sim

Follow the instructions on the official NVIDIA Isaac Sim website to install Isaac Sim on your computer.

## 4. Set up your environment

```bash
# Create a new ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Clone the repository
git clone https://github.com/your-username/physical-ai-and-humanoid-robotics.git src/physical-ai-and-humanoid-robotics

# Build the workspace
colcon build
```

## 5. Run the simulation

```bash
# Source the workspace
source ~/ros2_ws/install/setup.bash

# Launch the simulation
ros2 launch humanoid_robot simulation.launch.py
```
