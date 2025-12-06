---
id: hardware-requirements
title: Hardware Requirements Guide
slug: /hardware-requirements
---

# Hardware Requirements Guide

To fully engage with the practical examples, simulations, and projects presented in "Physical AI and Humanoid Robotics," certain hardware specifications are highly recommended. While some sections can be explored with more modest setups, especially purely theoretical content, the hands-on nature of the book, particularly for high-fidelity simulations and AI model training, necessitates robust computing resources.

This guide outlines the minimum recommended and ideal hardware configurations.

## 1. Workstation GPU (Primary Development and Simulation)

A powerful workstation GPU is critical for running high-fidelity physics simulations (NVIDIA Isaac Sim, Gazebo with complex models) and for training deep learning models.

| Component      | Minimum Recommended                          | Ideal / High-Performance                     | Reasoning                                                        |
| :------------- | :------------------------------------------- | :------------------------------------------- | :--------------------------------------------------------------- |
| **GPU Model**  | NVIDIA GeForce RTX 4070 Ti (or equivalent)   | NVIDIA GeForce RTX 3090 / 4090 (or equivalent workstation GPU) | Isaac Sim heavily leverages NVIDIA CUDA cores and Tensor Cores. Higher VRAM allows for larger, more complex simulation environments and faster AI model training. RTX series with Tensor Cores are preferred. |
| **VRAM**       | 12 GB                                        | 24 GB+                                       | Directly impacts scene complexity, number of simulated robots, and the size of deep learning models that can be trained locally. Essential for Omniverse-based simulations. |
| **CUDA Cores** | 7680 (RTX 4070 Ti)                           | 10496 (RTX 3090) / 16384 (RTX 4090)          | More CUDA cores accelerate physics, rendering, and AI computation. |
| **Driver**     | Latest NVIDIA Game Ready or Studio Driver    | Latest NVIDIA Game Ready or Studio Driver    | Crucial for optimal performance and compatibility with CUDA, Isaac Sim, and other NVIDIA tools. |

:::info Cloud GPU Fallback
If local hardware constraints prevent meeting these recommendations, consider cloud-based GPU instances. AWS g5 or g6e instances (featuring A10G GPUs with 24GB VRAM) or NVIDIA's Omniverse Cloud services can provide the necessary computational power for demanding tasks like Isaac Sim and large-scale AI training.
:::

## 2. Edge AI Platform (On-Robot Inference and Control)

For deploying AI models directly onto a robot and executing real-time control, an NVIDIA Jetson platform is ideal, especially given its tight integration with Isaac ROS.

| Component     | Minimum Recommended           | Preferred / Advanced        | Reasoning                                                        |
| :------------ | :---------------------------- | :-------------------------- | :--------------------------------------------------------------- |
| **Model**     | NVIDIA Jetson Orin Nano (8GB) | NVIDIA Jetson Orin NX (16GB) | Jetson Orin series offers superior AI performance over older generations (e.g., Xavier, Nano). Higher memory (NX) allows for more complex neural networks and simultaneous processes. |
| **RAM**       | 8 GB (shared with GPU)        | 16 GB (shared with GPU)     | Crucial for running ROS 2 nodes, AI inference, and other system processes concurrently. |
| **Storage**   | 64 GB NVMe SSD (external)     | 128 GB+ NVMe SSD (external) | Fast storage is essential for the operating system, ROS 2 packages, and large AI models. Internal eMMC can be slow; NVMe is highly recommended. |
| **Power Supply** | High-quality 5V/4A (Orin Nano), 19V/4.74A (Orin NX) | Ensure stable power for peak performance, especially under load. |

:::warning Note on Jetson Availability
Jetson devices can sometimes be subject to supply chain fluctuations. Plan accordingly or investigate alternative embedded platforms capable of running ROS 2 and NVIDIA software if Orin series devices are unavailable.
:::

## 3. Sensors (Perception and Environmental Interaction)

Accurate and reliable sensor data is the bedrock of intelligent robotics. This book assumes the availability of specific sensor types for its perception examples.

| Component                   | Recommended Model                     | Type      | Reasoning                                                        |
| :-------------------------- | :------------------------------------ | :-------- | :--------------------------------------------------------------- |
| **Depth Camera**            | Intel RealSense D435i / D455          | Active Stereo Depth | Industry-standard, provides high-quality depth, RGB, and IMU data. ROS 2 drivers are mature. Essential for object detection, SLAM, and obstacle avoidance. |
| **Inertial Measurement Unit (IMU)** | MPU-6050 (integrated into RealSense D435i) or standalone BNO055 (USB) | IMU       | Provides orientation, angular velocity, and linear acceleration. Critical for robot localization, balance control, and motion tracking. |
| **Microphone Array**        | ReSpeaker USB 4-Mic Array             | Audio Input | Required for voice command processing (Whisper integration) and sound source localization in VLA systems. |
| **LiDAR (Optional but useful)** | RPLIDAR A1/A2 (2D)                    | Laser Scanner | While not strictly required for all examples, 2D LiDAR is valuable for robust mapping and navigation, especially in larger environments. |

## 4. Peripherals and Miscellaneous

*   **High-Resolution Monitor:** Essential for comfortable development, especially when working with simulation environments and complex code. Dual monitors are highly recommended.
*   **Keyboard and Mouse:** Standard development peripherals.
*   **Ethernet Cable:** For stable network connectivity, especially important for ROS 2 communication between development machine and Jetson, or between simulated nodes.
*   **USB-C to USB-A Adapter (for Jetson):** If your workstation only has USB-A ports.
*   **External USB Hub:** Useful for managing multiple sensor and peripheral connections.
*   **ROS 2-compatible Humanoid Robot Platform (Optional):** While the book focuses heavily on simulation and assumes a generic humanoid model, having access to a physical platform (e.g., a mini-humanoid research platform) can provide invaluable real-world experience for the final capstone.

By ensuring you have access to hardware meeting these specifications, you will be well-prepared to tackle the challenging and rewarding projects within this book.