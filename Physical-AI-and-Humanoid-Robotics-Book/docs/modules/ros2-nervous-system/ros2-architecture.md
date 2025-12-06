---
id: ros2-architecture
title: "ROS 2 Architecture: Nodes, Topics, Services, Actions"
slug: /modules/ros2-nervous-system/ros2-architecture
---

# ROS 2 Architecture: Nodes, Topics, Services, Actions

The Robot Operating System 2 (ROS 2) is the de facto standard for robotic application development. It provides a flexible framework for writing robot software, offering a collection of tools, libraries, and conventions that simplify the complex task of building robust robotic systems. For a humanoid robot, ROS 2 acts as the central nervous system, coordinating everything from low-level motor control and sensor data processing to high-level AI decision-making.

This chapter delves into the fundamental architectural components of ROS 2: Nodes, Topics, Services, and Actions. Understanding these concepts is paramount to developing modular, scalable, and maintainable robotic applications.

## 1. The ROS 2 Graph: A Distributed System

At its core, ROS 2 is a distributed system. A robot application built with ROS 2 is not a single monolithic program but rather a collection of small, specialized, and independent executable programs (nodes) that communicate with each other. This distributed architecture offers several key advantages:

*   **Modularity:** Each component (e.g., a camera driver, a motor controller, a navigation algorithm) can be developed, tested, and maintained independently.
*   **Reusability:** Nodes can be reused across different robot platforms or applications.
*   **Fault Tolerance:** If one node crashes, other nodes can continue operating.
*   **Scalability:** Processing can be distributed across multiple machines or even cloud resources.

These nodes communicate asynchronously or synchronously over a network, forming what is conceptually known as the "ROS 2 Graph."

### A. ROS 1 vs. ROS 2: Key Differences

While ROS 2 retains many core concepts from ROS 1, it introduces significant improvements, especially relevant for real-world Physical AI applications:

| Feature           | ROS 1                                         | ROS 2                                                             | Impact for Physical AI & Humanoids                                                                                                                  |
| :---------------- | :-------------------------------------------- | :---------------------------------------------------------------- | :-------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Communication** | ROS Master (single point of failure)          | DDS (Data Distribution Service) - decentralized                   | Enhanced robustness and real-time capabilities. No single point of failure means more reliable operation in critical humanoid tasks.                  |
| **Real-time**     | Best-effort communication                     | Hard real-time support (via specific RMW implementations)         | Crucial for precise, low-latency control loops for humanoid balance and manipulation.                                                               |
| **Security**      | None built-in                                 | DDS-Security (authentication, encryption, access control)         | Essential for safe and secure operation of humanoid robots, especially in human-shared environments.                                                |
| **Multi-robot**   | Challenging with single ROS Master            | Native multi-robot communication                                  | Enables coordination of multiple humanoids or a humanoid with other robotic systems.                                                                |
| **Language Support** | C++, Python (primary)                         | C++, Python, Java, C#, Rust, etc. (more robust client libraries)  | Broader development options, better integration with diverse AI frameworks often written in Python.                                                 |
| **Platform Support** | Linux-centric                                 | Linux, Windows, macOS, RTOS                                       | Greater flexibility for development and deployment environments, including embedded systems like Jetson.                                            |

## 2. Nodes: The Computational Units

A **Node** is the fundamental computational unit in ROS 2. It is an executable program that performs a specific task. In a humanoid robot, examples of nodes include:

*   **`camera_driver_node`**: Reads images from a camera.
*   **`motor_controller_node`**: Sends commands to joint motors and reads encoder feedback.
*   **`vslam_node`**: Processes camera images to estimate robot pose and build a map.
*   **`path_planner_node`**: Computes a collision-free path for the robot.
*   **`voice_command_node`**: Processes audio input for natural language understanding.

Each node is independent and can be started, stopped, and restarted without affecting other nodes, provided their communication protocols are honored.

### Verbal Diagram: Example ROS 2 Nodes
```
+------------------+    +------------------+    +------------------+
|  camera_driver   |    | motor_controller |    |   path_planner   |
|     (Node A)     |    |     (Node B)     |    |     (Node C)     |
+------------------+    +------------------+    +------------------+
         |                       |                       |
         +-----------------------+-----------------------+
         |                                               |
         |         DDS (Data Distribution Service)         |
         |          (Middleware for Communication)         |
         |                                               |
         +-----------------------+-----------------------+
         |                       |                       |
+------------------+    +------------------+    +------------------+
|    vslam_node    |    | voice_command_node |    |    robot_model   |
|     (Node D)     |    |     (Node E)     |    |     (Node F)     |
+------------------+    +------------------+    +------------------+
```
In this diagram, each box represents a node, and the lines indicate potential communication channels facilitated by the DDS middleware.

## 3. Communication Mechanisms: Topics, Services, and Actions

Nodes communicate using different mechanisms, each suited for a particular type of data exchange.

### A. Topics: Asynchronous, Publish-Subscribe Streaming

**Purpose:** Topics are used for streaming data asynchronously. One node (the "publisher") sends messages to a named topic, and any other node (the "subscriber") interested in that data can receive those messages. This is a many-to-many, one-way communication pattern.

**Analogy:** A radio station broadcasting music. Many listeners can tune in, but the radio station doesn't know who is listening or if they received the message.

**Use Cases for Humanoids:**
*   **Sensor Data:** Publishing camera images (`/camera/image_raw`), LiDAR scans (`/scan`), IMU data (`/imu/data`).
*   **Odometry:** Publishing robot's estimated pose (`/odom`).
*   **Joint States:** Publishing the current angles of all robot joints (`/joint_states`).
*   **Control Commands:** Publishing desired joint velocities or torques.

**Example: Publisher-Subscriber Interaction**
```
+-------------------+      /image_raw (std_msgs/msg/Image)    +-------------------+
|  camera_driver    | --------------------------------------> |   vslam_node      |
|  (Publisher)      |                                         |   (Subscriber)    |
+-------------------+                                         +-------------------+
         |                                                              |
         |                                                              |
         |       /joint_states (sensor_msgs/msg/JointState)             |
         | ------------------------------------------------------------> | motor_controller  |
         |                                                              | (Subscriber)      |
         |                                                              +-------------------+
         |
         | ROS 2 Middleware (DDS)
         |
```

**Key Characteristics:**
*   **Asynchronous:** Publishers don't wait for subscribers.
*   **One-way:** Information flows from publisher to subscriber.
*   **Decoupled:** Publishers and subscribers don't need to know about each other directly.
*   **Streaming:** Ideal for continuous data flows.

### B. Services: Synchronous, Request-Response

**Purpose:** Services are used for synchronous, request-response communication. A "client" node sends a request to a "server" node, and the client blocks (waits) until the server processes the request and sends back a response. This is a one-to-one communication pattern.

**Analogy:** Calling a customer service hotline. You make a request, wait for a specific answer, and then hang up.

**Use Cases for Humanoids:**
*   **Robot Configuration:** Setting a new PID gain for a joint (`/set_pid_gain`).
*   **Triggering Actions:** Taking a single snapshot from a camera (`/camera/take_snapshot`).
*   **Kinematics Calculations:** Requesting inverse kinematics solution for a desired end-effector pose.
*   **Diagnostic Queries:** Querying the status of a specific sensor or actuator.

**Example: Client-Server Interaction**
```
+-------------------+                           +-------------------+
| path_planner_node | -- Request (MoveGoal) --> | motion_control_node |
|   (Client)        | <-- Response (Success) -- |   (Server)        |
+-------------------+                           +-------------------+
         ^                                               ^
         |                                               |
         | ROS 2 Middleware (DDS)                        |
         |                                               |
```

**Key Characteristics:**
*   **Synchronous:** Client waits for the server.
*   **Two-way:** Request followed by a response.
*   **One-time:** Best for single, discrete operations.
*   **Reliable:** Guaranteed delivery of request and response.

### C. Actions: Asynchronous, Goal-Oriented with Feedback

**Purpose:** Actions are designed for long-running, goal-oriented tasks that provide periodic feedback and can be preempted (canceled). They combine aspects of topics (feedback stream) and services (goal/result).

**Analogy:** Ordering a pizza. You send an order (goal), get updates on its status (feedback: "dough stretched," "in the oven," "out for delivery"), and eventually receive the pizza (result). You can also call to cancel the order.

**Use Cases for Humanoids:**
*   **Navigation:** Commanding the robot to move to a specific location (`/navigate_to_pose`). Feedback includes current position, remaining distance.
*   **Complex Manipulation:** Instructing the robot to pick up an object (`/pick_object`). Feedback includes progress of grasping, object stability.
*   **Locomotion Behaviors:** Commanding a specific walking gait or sequence of steps.
*   **Voice Control:** Processing a complex natural language command that requires multiple steps (e.g., "Go to the kitchen, find the coffee cup, and bring it here").

**Example: Action Client-Server Interaction**
```
+-------------------+
|  voice_command_node | --- Goal (PickObject) --> +-------------------+
|   (Action Client) |                             | manipulation_node |
+-------------------+                             | (Action Server)   |
          ^                                       +-------------------+
          |                                       |
          | <--- Feedback (Progress%) ----------- |
          |                                       |
          | <--- Result (Success/Failure) --------|
          |                                       |
          | ROS 2 Middleware (DDS)                |
          |                                       |
```

**Key Characteristics:**
*   **Asynchronous:** Client doesn't block entirely; can receive feedback and send cancel requests.
*   **Goal-oriented:** Defines a goal, gets a result.
*   **Feedback:** Provides updates on progress.
*   **Preemptable:** Client can cancel an ongoing action.
*   **Reliable:** Ensures delivery of goal, feedback, and result.

## 4. The ROS 2 Client Libraries (RCL)

To interact with these ROS 2 communication mechanisms, developers use Client Libraries (RCLs). The most common are `rclcpp` (for C++) and `rclpy` (for Python). This book primarily uses `rclpy` due to its integration with Python's extensive AI/ML ecosystem.

### `rclpy` Node Structure

A basic `rclpy` node typically involves:
1.  **Initialization:** `rclpy.init(args=args)`
2.  **Node Creation:** `node = rclpy.create_node('my_node_name')`
3.  **Spinning:** `rclpy.spin(node)` (keeps the node alive, allowing callbacks to be executed)
4.  **Shutdown:** `node.destroy_node(); rclpy.shutdown()`

This modular and distributed architecture, facilitated by nodes and their communication via topics, services, and actions, allows for the complex integration of hardware control, sensor processing, and advanced AI algorithms necessary for a sophisticated humanoid robot.