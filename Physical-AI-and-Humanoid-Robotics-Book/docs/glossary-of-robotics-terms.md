---
id: glossary
title: Glossary of Robotics Terms
slug: /glossary
---

# Glossary of Robotics Terms

This glossary provides definitions for key terms and concepts frequently used throughout "Physical AI and Humanoid Robotics." Understanding these terms is crucial for effective comprehension of the material.

---

**Action (ROS 2)**
A type of communication in ROS 2 used for long-running, goal-oriented tasks with explicit feedback and cancellability. It is built on top of topics and services and is ideal for complex behaviors like navigation or manipulation sequences.

**Actuator**
A component of a robot responsible for moving or controlling a mechanism or system. Actuators convert energy (e.g., electrical, hydraulic, pneumatic) into mechanical force, torque, or displacement. Examples include motors, hydraulic cylinders, and pneumatic pistons.

**AI-Native**
An approach to system design where artificial intelligence is fundamental to the architecture and operation from the ground up, rather than being an add-on. Emphasizes design for AI principles like learning, adaptivity, and data-driven decision-making.

**APA Style**
(American Psychological Association Style) A widely used academic formatting and citation style for publications. In this book, it refers to the standard for in-text citations and reference lists.

**Autonomous Robot**
A robot that can perform tasks in an environment without continuous human guidance, using its sensors to perceive the environment and its AI to make decisions.

**Backmatter**
The sections of a book that come after the main content, typically including appendices, glossaries, references, and indices.

**Bipedal Locomotion**
The act of moving using two legs, common in humanoids. It involves complex balance and control mechanisms.

**Capstone Project**
A culminating project that integrates knowledge and skills acquired throughout a course or book, demonstrating mastery of the subject matter.

**CI/CD (Continuous Integration/Continuous Deployment)**
A set of practices that enable rapid and reliable software delivery. Continuous Integration involves frequently merging code changes into a central repository, and Continuous Deployment automates the release of software to various environments.

**Computer Vision**
A field of artificial intelligence that trains computers to interpret and understand the visual world from digital images or videos. It involves tasks like object detection, image segmentation, and pose estimation.

**Control Loop**
A system that continuously monitors a variable, compares it to a desired setpoint, and adjusts an output to minimize the difference. In robotics, it's used to control joint positions, velocities, or forces.

**CUDA (Compute Unified Device Architecture)**
A parallel computing platform and application programming interface (API) model developed by NVIDIA for its GPUs. CUDA allows software developers and engineers to use a CUDA-enabled graphics processing unit for general purpose processing – an approach known as GPGPU.

**Deep Learning (DL)**
A subfield of machine learning that uses artificial neural networks with multiple layers (deep neural networks) to learn representations of data with multiple levels of abstraction. Particularly effective for complex pattern recognition tasks like image and speech processing.

**Digital Twin**
A virtual replica of a physical object, system, or process that serves as a real-time digital counterpart. In robotics, it allows for simulation, testing, and monitoring of robots in a virtual environment.

**Docusaurus**
An open-source static site generator that helps you build, deploy, and maintain documentation websites with ease. It uses Markdown for content and React for dynamic features.

**Domain Randomization**
A technique used in sim-to-real transfer where parameters of the simulated environment (e.g., textures, lighting, object positions, physics properties) are randomized during training. This forces the learning agent to learn policies robust to variations, making them more transferable to the real world.

**Frontmatter**
The introductory sections of a book that appear before the main content, typically including the title page, preface, table of contents, and introduction.

**Gazebo**
A powerful open-source 3D physics simulator for robotics. It provides robust physics engines, high-quality rendering, and a convenient interface for creating and managing complex robot simulations.

**GitHub Actions**
A CI/CD platform that allows you to automate your build, test, and deployment pipeline directly from GitHub.

**Humanoid Robot**
A robot designed to resemble the human body, typically featuring a torso, head, two arms, and two legs. They are engineered to operate in environments designed for humans.

**IMU (Inertial Measurement Unit)**
An electronic device that measures and reports a body's specific force, angular rate, and sometimes the orientation of the body, using a combination of accelerometers, gyroscopes, and magnetometers. Critical for robot localization and balance.

**Isaac ROS**
A collection of hardware-accelerated ROS 2 packages developed by NVIDIA that leverage NVIDIA GPUs and Jetson platforms to provide high-performance solutions for common robotics tasks like perception, navigation, and manipulation.

**Isaac Sim (NVIDIA)**
A scalable robotics simulation platform built on NVIDIA Omniverse, designed for developing, testing, and managing AI-driven robots. It offers physically accurate simulation, synthetic data generation, and advanced tools for reinforcement learning.

**Jetson (NVIDIA)**
A series of embedded computing boards from NVIDIA designed for AI and deep learning applications at the edge. Often used for on-robot inference due to their power efficiency and GPU acceleration.

**Kinematics**
The branch of mechanics that describes the motion of points, bodies (objects), or systems of bodies without considering the forces that cause them to move. In robotics, it deals with the mathematical description of the robot's movement and position.

**LiDAR (Light Detection and Ranging)**
A remote sensing method that uses light in the form of a pulsed laser to measure ranges (variable distances) to the Earth. Used in robotics for mapping, localization, and obstacle detection.

**LLM (Large Language Model)**
An artificial intelligence program capable of generating human-like text, understanding context, and performing various language-based tasks. Used in VLA systems for high-level planning and instruction interpretation.

**Multimodal Interaction**
Refers to interfaces that allow users to interact with a system using multiple modes of communication, such as speech, gestures, and visual cues. In robotics, it enables more natural human-robot interaction.

**Nav2 (Navigation2)**
The ROS 2 navigation stack, providing tools for autonomous mobile robot navigation. It includes components for path planning, obstacle avoidance, and localization.

**Node (ROS 2)**
An executable process in ROS 2 that performs computation (e.g., a camera driver, a motor controller, an algorithm for path planning). Multiple nodes can run concurrently and communicate with each other.

**Omniverse (NVIDIA)**
An open platform for virtual collaboration and real-time physically accurate simulation. It is built on Pixar's Universal Scene Description (USD) and leverages NVIDIA's RTX and AI technologies.

**Physical AI**
The field of artificial intelligence focused on developing intelligent systems that understand, interact with, and operate within the physical world. It emphasizes embodiment, perception, action, and learning in real-world environments.

**`rclpy`**
The ROS Client Library for Python in ROS 2. It provides the Python API for creating ROS 2 nodes and interacting with the ROS 2 ecosystem.

**Reinforcement Learning (RL)**
A type of machine learning where an agent learns to make decisions by performing actions in an environment to maximize a cumulative reward. Often used to train robots to learn complex behaviors.

**Robot Operating System 2 (ROS 2)**
An open-source set of software libraries and tools that help you build robot applications. It provides middleware, development tools, and libraries for communication, hardware abstraction, and package management.

**Sensor**
A device that detects and responds to some type of input from the physical environment (e.g., light, heat, motion, pressure). In robotics, sensors provide the robot's perception of its surroundings.

**Service (ROS 2)**
A type of communication in ROS 2 used for synchronous request-response interactions. A client sends a request, and a server processes it and sends back a response.

**SDF (Simulation Description Format)**
An XML format for describing objects and environments for robot simulators like Gazebo. It is a more comprehensive format than URDF, supporting physics properties, lights, and sensors directly.

**Sim-to-Real Transfer**
The process of transferring policies or models learned in a simulation environment to a real-world robot. Techniques often involve domain randomization, domain adaptation, and reality gap mitigation.

**SLAM (Simultaneous Localization and Mapping)**
The computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it. Visual SLAM (VSLAM) uses camera data for this.

**Synthetic Data Generation (SDG)**
The process of creating artificial data, often using simulation, that mimics the characteristics of real-world data. Crucial for training AI models, especially when real data is scarce or difficult to acquire.

**`tf2` (ROS 2 Transformations)**
A ROS 2 library that allows you to keep track of multiple coordinate frames and transform points, vectors, etc., between any two coordinate frames at any time. Essential for robot kinematics and multi-sensor fusion.

**Topic (ROS 2)**
An asynchronous, publish-subscribe communication channel in ROS 2. Nodes publish messages to topics, and other nodes subscribe to those topics to receive messages.

**Unified Robot Description Format (URDF)**
An XML format for representing a robot model in ROS. It describes the robot's kinematic and dynamic properties, visual appearance, and collision properties.

**Unity**
A cross-platform real-time 3D development platform used for creating games, simulations, and other interactive experiences. Valued for its high-fidelity rendering and rich asset ecosystem.

**Vision-Language-Action (VLA) Systems**
AI systems that integrate visual perception, natural language understanding, and physical action capabilities, typically driven by large language models, to enable robots to interpret and execute complex human commands.

**Whisper (OpenAI)**
An automatic speech recognition (ASR) system developed by OpenAI that can transcribe human speech into text. Used in VLA systems for processing voice commands.