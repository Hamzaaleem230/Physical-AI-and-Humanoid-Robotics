---
id: gazebo-setup-physics-engine
title: Gazebo Setup & Physics Engine
slug: /modules/digital-twin/gazebo-setup-physics-engine
---

# Gazebo Setup & Physics Engine

The journey from a conceptual humanoid robot to a physical machine capable of complex tasks is fraught with challenges. Debugging hardware can be time-consuming, expensive, and even dangerous. This is where **simulation** becomes an indispensable tool. A digital twin—a virtual replica—allows for rapid prototyping, safe experimentation, and systematic testing of robot designs, control algorithms, and AI policies. Among the most widely used open-source robotics simulators, **Gazebo** stands out as a robust platform for physics-based simulation.

This chapter will guide you through the process of setting up Gazebo, understanding the fundamentals of its physics engine, and configuring it to accurately simulate humanoid robots.

## 1. Installing Gazebo

Gazebo is closely integrated with ROS 2. For ROS 2 Humble (Ubuntu 22.04), the recommended Gazebo version is Gazebo Garden (formerly Gazebo Classic). However, the ROS 2 documentation often points to specific versions of Gazebo. Always refer to the official ROS 2 documentation for the exact Gazebo version compatible with your ROS 2 distribution. For our purposes, we will assume a typical installation suitable for ROS 2 Humble.

### A. Recommended Installation (Ubuntu 22.04)

First, ensure your system is up-to-date:

```bash
sudo apt update && sudo apt upgrade -y
```

Then, install Gazebo (Gazebo Garden):

```bash
# Add the Gazebo Garden repository
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list'
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

# Update package list and install Gazebo Garden
sudo apt update
sudo apt install gazebo
```

Verify the installation by launching a simple world:

```bash
gazebo # or gzclient for older versions
```
You should see a blank Gazebo window with a ground plane and a sun.

### B. Installing ROS 2 Gazebo Packages

To integrate Gazebo with ROS 2, you need specific bridge packages. For Gazebo Garden:

```bash
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros-control
```
*   `gazebo_ros_pkgs`: Provides the core ROS 2 integration for Gazebo.
*   `gazebo_ros_control`: Enables the use of `ros2_control` (a hardware abstraction framework) with simulated robots in Gazebo.

## 2. Gazebo's Physics Engine: The Heart of Simulation

At the core of Gazebo is its physics engine, responsible for simulating the realistic interactions between robot links and the environment. Gazebo supports multiple physics engines, including:

*   **ODE (Open Dynamics Engine):** The default and most commonly used engine, known for its robustness.
*   **Bullet:** Another popular choice, often used for soft-body dynamics and advanced collision detection.
*   **DART (Dynamic Animation and Robotics Toolkit):** Emphasizes stability and accuracy for complex multi-body systems.
*   **Simbody:** High-performance dynamics for biomechanical and rigid-body systems.

You can select the physics engine in your Gazebo world file.

### A. Physics Engine Configuration (Conceptual)

A Gazebo world file (`.world` extension) is an XML file that describes the entire simulation environment. It includes information about light sources, ground planes, models, and crucial physics parameters.

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="default">
    <!-- ... (light, ground plane includes) ... -->

    <physics name="default_physics" type="ode"> <!-- Specifies ODE as the physics engine -->
      <ode>
        <solver>
          <type>quick</type> <!-- Iterative solver type -->
          <iters>50</iters> <!-- Number of iterations for solver (higher = more accurate, slower) -->
          <tolerance>0.0001</tolerance>
        </solver>
        <constraints>
          <cfm>0.00001</cfm> <!-- Constraint Force Mixing (softness of constraints) -->
          <erp>0.2</erp>   <!-- Error Reduction Parameter (how quickly constraints are met) -->
          <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.001</max_step_size> <!-- Simulation time step (e.g., 1ms) -->
      <real_time_factor>1.0</real_time_factor> <!-- Real-time factor (1.0 = simulation runs at real-time speed) -->
      <real_time_update_rate>1000</real_time_update_rate> <!-- How many physics updates per second -->
      <max_contacts>20</max_contacts>
    </physics>

    <!-- ... (robot model and other environment elements) ... -->

  </world>
</sdf>
```

**Key Parameters for Humanoid Simulation:**

*   **`max_step_size`**: The duration of each physics simulation step. A smaller step size leads to more accurate (but slower) simulation. For stable humanoid locomotion, often `0.001`s or less.
*   **`real_time_factor`**: Controls the speed of simulation relative to real-time. A value of `1.0` means the simulation tries to run at real-time speed. If the simulation falls behind, this factor drops below 1.0. For real-time control, it's desirable to maintain a factor close to 1.0.
*   **`real_time_update_rate`**: The number of physics updates per second. Directly related to `max_step_size` (`real_time_update_rate = 1 / max_step_size`).
*   **Solver iterations (`iters`):** For iterative solvers (like `quick`), more iterations generally lead to more accurate force calculations and constraint resolution, crucial for stable contacts (e.g., feet on ground, grasping objects).

### B. Understanding Physics Engine Parameters

The tuning of physics parameters is a critical and often challenging aspect of achieving realistic robot behavior in simulation, especially for humanoids which involve complex contacts and balance.

**Comparison Table: Impact of Physics Parameters**

| Parameter             | Description                                                                 | Low Value Impact                                                                                                                                                                                                                                                             | High Value Impact                                                                                                                                                                                                                                                            |
| :-------------------- | :-------------------------------------------------------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | :--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `max_step_size`       | Duration of each physics step (seconds).                                    | More accurate simulation, better for contact/balance. Slower simulation speed. Higher CPU usage.                                                                                                                                                                                             | Less accurate simulation. May lead to instability, objects passing through each other (tunneling). Faster simulation speed. Lower CPU usage.                                                                                                                                             |
| `iters` (solver)      | Number of iterations for the physics solver.                                | Less accurate constraint resolution. May result in "bouncy" contacts or joints not behaving as expected.                                                                                                                                                                                     | More accurate constraint resolution. Better for stable contacts (humanoid feet on ground). Slower simulation speed. Higher CPU usage.                                                                                                                                                     |
| `cfm` (Constraint Force Mixing) | How "soft" constraints are. Higher value means constraints are softer. | Constraints are very stiff, leading to high-frequency oscillations if not perfectly met. Can cause instability with small `max_step_size` if not enough solver `iters`.                                                                                                                             | Constraints are very soft, allowing for slight penetration or "squishiness" in contacts. Can stabilize simulation but may feel less rigid.                                                                                                                                                            |
| `erp` (Error Reduction Parameter) | How quickly constraint errors are corrected.                        | Constraints errors are corrected slowly, leading to visible penetration or "lag" in joint limits.                                                                                                                                                                                          | Constraints errors are corrected quickly, leading to more rigid contacts. Can cause instability if `erp` is too high with large `max_step_size` or low `iters`.                                                                                                                              |
| `real_time_factor`    | Speed of simulation relative to real-time.                                  | Simulation runs slower than real-time. Useful for debugging complex behavior.                                                                                                                                                                                               | Simulation tries to run faster than real-time (if possible). Not ideal for real-time control loops, which expect a `real_time_factor` near 1.0.                                                                                                                                              |

## 3. Integrating Robot Models

Gazebo uses models to represent everything from robots to furniture. These models are typically defined in SDF. However, as we learned, robots are often described in URDF. Gazebo can natively import URDF files, converting them internally to SDF.

### A. Loading a URDF into Gazebo (via ROS 2 Launch)

The most common way to get your URDF-defined humanoid robot into Gazebo is via a ROS 2 launch file. This usually involves:

1.  **`robot_state_publisher`:** A ROS 2 node that reads the URDF and publishes the robot's joint state transformations to `/tf` (TF2).
2.  **`spawn_entity.py`:** A Gazebo ROS 2 utility to spawn models into the simulation.
3.  **Gazebo Launch:** Launching Gazebo itself.

Here's an example `spawn_humanoid.launch.py` to get our `humanoid.urdf` into Gazebo:

```python
# humanoid_sim/launch/spawn_humanoid.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to your package's share directory
    humanoid_description_package = get_package_share_directory('humanoid_description') # Assuming you have a package for your robot's description

    # Path to the URDF file
    urdf_file = os.path.join(humanoid_description_package, 'urdf', 'humanoid.urdf')

    # Start Gazebo simulation
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={'world': os.path.join(get_package_share_directory('humanoid_sim'), 'worlds', 'empty.world')}.items()
    )

    # Robot State Publisher node
    # This node reads the URDF and publishes the robot's joint transforms to /tf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_file).read()}],
        arguments=[urdf_file] # Passing URDF as an argument
    )

    # Spawn the robot entity in Gazebo
    # This uses the 'spawn_entity.py' script from gazebo_ros
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'humanoid'],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node
    ])
```

To run this, you would need to:
1.  Create a `humanoid_description` package with your URDF.
2.  Create a `humanoid_sim` package for your launch files and world files.
3.  Install `ros-humble-robot-state-publisher`.
4.  Build both packages.
5.  Then, `ros2 launch humanoid_sim spawn_humanoid.launch.py`.

This will open Gazebo with your humanoid robot model, floating in the air. To make it interact with the ground, you need to add a controller and potentially define plugins for its base or feet to simulate contact.

### B. Gazebo Plugins for Advanced Simulation

Gazebo's functionality can be extended through plugins. For humanoid robots, critical plugins include:

*   **`libgazebo_ros_force_system.so`:** Applies forces to links, useful for testing balance or external disturbances.
*   **`libgazebo_ros_imu_sensor.so`:** Simulates IMU data, publishing to `sensor_msgs/msg/Imu`.
*   **`libgazebo_ros_ray_sensor.so`:** Simulates LiDAR or range finder data.
*   **`libgazebo_ros_joint_state_publisher.so`:** Publishes `sensor_msgs/msg/JointState` messages for the robot's joints.
*   **`libgazebo_ros_control.so`:** Integrates with `ros2_control` to allow ROS 2 controllers to command simulated joints.

These plugins are typically embedded directly into your robot's URDF or SDF model definition.

## 4. Case Study: Simulating Humanoid Balance

Simulating humanoid balance is a prime example of where physics engine tuning becomes critical.

**Scenario:** A humanoid robot is standing on a flat surface. We want it to maintain balance when subjected to small external perturbations.

**Challenges:**
*   **Stable Contacts:** The interaction between the robot's feet and the ground must be stable. "Jittering" or "bouncing" feet indicate poor physics solver settings.
*   **Accurate Inertia:** The `inertial` properties of each link in the URDF must be accurate to reflect the robot's true center of mass and rotational dynamics.
*   **Controller Latency:** The balance controller (implemented in ROS 2) needs to send commands to the simulated joints with very low latency.

**Physics Tuning Steps:**
1.  **`max_step_size`**: Start with a small value (e.g., 0.001s or 1ms) for stability.
2.  **Solver `iters`**: Increase `iters` (e.g., 50-100) until contacts are stable and the robot doesn't "fall through" the floor.
3.  **`erp` and `cfm`**: Adjust these values carefully to achieve a balance between rigidity and stability. Small `erp` values can prevent jitter, while small `cfm` values make constraints stiffer.
4.  **`real_time_factor`**: Monitor this. If it drops significantly below 1.0, your simulation is running slower than real-time, which can degrade the performance of real-time controllers. You may need to simplify your model or reduce `max_step_size` if accuracy is more important.

**Verbal Diagram: Gazebo Simulation Stack**
```
+-------------------------------------------------------+
|                 ROS 2 Nodes                           |
|  +---------------------+   +---------------------+  |
|  | Balance Controller  |   |    IMU Subscriber   |  |
|  +---------+-----------+   +---------+-----------+  |
|            |                         |                |
|            v                         v                |
|  (Joint Commands)            (IMU Data)              |
|                                                       |
+-------------------------------------------------------+
            ^                             ^
            |                             |
            | ROS 2 Topics                | ROS 2 Topics
            v                             v
+-------------------------------------------------------+
|                 Gazebo Simulator                      |
|  +---------------------+   +---------------------+  |
|  |  ROS 2 Control      |   |   Gazebo Plugins    |  |
|  |   (Controller Mgr)  |<--|   (IMU, JointState) |  |
|  +---------+-----------+   +---------------------+  |
|            |                         ^                |
|            v                         |                |
|  +-------------------------------------------------+  |
|  |           Physics Engine (ODE/Bullet/DART)      |  |
|  |   - Robot Model (URDF/SDF)                      |  |
|  |   - Environment                                 |  |
|  +-------------------------------------------------+  |
+-------------------------------------------------------+
```

## 5. Comparison: Gazebo Physics vs. Isaac Sim Physics (Conceptual)

While both Gazebo and Isaac Sim offer physics-based simulation, there are differences in their underlying engines and capabilities:

| Feature           | Gazebo (Default ODE)                                      | NVIDIA Isaac Sim (PhysX 5)                                             |
| :---------------- | :-------------------------------------------------------- | :--------------------------------------------------------------------- |
| **Physics Engine** | ODE (default), Bullet, DART, Simbody. CPU-centric solvers. | PhysX 5. GPU-accelerated, highly stable for complex contacts and stacks. |
| **Accuracy**      | Good for general robotics; can struggle with very complex, high-friction contacts. | Excellent, especially for contact-rich manipulation and dynamic locomotion.                                                          |
| **Performance**   | CPU-bound for physics calculations; scales well with multiple models. | GPU-accelerated; excels with many bodies and complex interactions.                                                                  |
| **Real-time**     | Achievable but sensitive to `max_step_size` and `iters`.   | Designed for real-time and faster-than-real-time simulation, especially with many agents.                                            |
| **Determinism**   | Can be challenging to ensure exact determinism across runs. | High degree of determinism, crucial for reinforcement learning.                                                                      |
| **ROS 2 Integration** | `gazebo_ros_pkgs` (well-established).                     | `OmniGraph`, `ROS 2 Bridge` (native integration with Omniverse).                                                                     |
| **Use Case**      | General-purpose simulation, wide community support, older projects. | AI-driven robotics, synthetic data generation, RL, high-fidelity visualization, large-scale multi-robot simulation.                      |

## Conclusion

Setting up and configuring Gazebo's physics engine is a critical first step in developing robust digital twins for humanoid robots. By carefully selecting and tuning physics parameters, you can create a stable and accurate simulation environment that will enable effective testing of your robot's control algorithms and AI behaviors. This foundational understanding paves the way for more advanced simulation techniques and the eventual transfer of learned policies to real hardware.