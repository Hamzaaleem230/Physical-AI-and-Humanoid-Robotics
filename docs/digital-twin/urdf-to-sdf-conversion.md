---
id: urdf-to-sdf-conversion
title: URDF to SDF Conversion
slug: /modules/digital-twin/urdf-to-sdf-conversion
---

# URDF to SDF Conversion

In the previous chapter on URDF, we established that it is the standard format for describing a single robot's kinematics, dynamics, and visual properties within the ROS ecosystem. However, when we move into full-fledged physics simulations like Gazebo, a more comprehensive format is needed—the **Simulation Description Format (SDF)**. SDF is designed to describe everything in a simulation environment, including robots, static objects, lights, sensors, and even physics engine parameters.

This chapter will clarify the differences between URDF and SDF and guide you through the process of converting a URDF model of a humanoid robot into an SDF model suitable for Gazebo.

## 1. URDF vs. SDF: A Comparison

While both URDF and SDF use XML to describe robots, their scope and capabilities differ significantly.

| Feature                 | URDF (Unified Robot Description Format)                                   | SDF (Simulation Description Format)                                            |
| :---------------------- | :------------------------------------------------------------------------ | :----------------------------------------------------------------------------- |
| **Primary Use**         | Robot description (kinematics, dynamics, visuals, collisions)             | Full simulation environment description (robots, worlds, lights, sensors)        |
| **Scope**               | Single robot                                                              | Multiple robots, static objects, environment                                   |
| **Kinematic Structure** | Tree structure (one root, children can't link back to parents)            | Graph structure (allows closed loops, e.g., parallel manipulators)             |
| **Physics Properties**  | Defined per link (`inertial`), per joint (`limit`, `dynamics`)            | More extensive physics properties (e.g., friction, restitution, contact parameters) defined directly at link/collision level |
| **Sensors**             | Not directly defined; typically added via ROS nodes                       | Can define sensors directly as Gazebo plugins within the model                   |
| **Plugins**             | No native plugin mechanism                                                | Supports Gazebo plugins directly within the model                              |
| **Lights, Terrain**     | Cannot describe                                                           | Can describe ambient light, sun, point lights, and terrain                       |
| **ROS Integration**     | Native to ROS (`robot_state_publisher`, `tf2`)                            | Used with `gazebo_ros` packages for ROS integration                           |
| **Common Use**          | ROS robot description, `rviz2` visualization, `ros2_control` hardware interface | Gazebo simulations, broader physics simulation contexts                      |

**Verbal Diagram: SDF World Structure**
```
+-------------------------------------------------------+
|                 SDF World File (.world)               |
|  +-------------------------------------------------+  |
|  |                <world name="...">             |  |
|  |   +-------------------------------------------+  |
|  |   |             <light name="...">            |  |
|  |   |             <model name="ground_plane">   |  |
|  |   |             <model name="humanoid_robot"> |  |
|  |   |               +-------------------------+  |
|  |   |               |         <link>          |  |
|  |   |               +-------------------------+  |
|  |   |               +-------------------------+  |
|  |   |               |         <joint>         |  |
|  |   |               +-------------------------+  |
|  |   |               +-------------------------+  |
|  |   |               |       <sensor>          |  |
|  |   |               +-------------------------+  |
|  |   |               +-------------------------+  |
|  |   |               |       <plugin>          |  |
|  |   |               +-------------------------+  |
|  |   +-------------------------------------------+  |
|  +-------------------------------------------------+  |
+-------------------------------------------------------+
```

## 2. The Conversion Process: `urdf_to_sdf`

Gazebo can directly read URDF files. When you load a URDF into Gazebo, it implicitly performs a conversion to an internal SDF representation. This process involves:

*   **Links → SDF `<link>`:** URDF links map directly to SDF links. Inertial, visual, and collision properties are translated.
*   **Joints → SDF `<joint>`:** URDF joints map directly to SDF joints. Joint types, origins, axes, and limits are preserved.
*   **Plugins:** URDF does not have a native way to define Gazebo plugins. These often need to be manually added to the URDF (using `<gazebo>` tags within the URDF, which are then interpreted by Gazebo) or after conversion to SDF.

### A. Converting via ROS 2 Launch (Implicit)

As seen in the previous chapter, using `spawn_entity.py` with a URDF file essentially tells Gazebo to load and convert that URDF.

The `robot_description` parameter, which holds the entire URDF XML, is passed to Gazebo, which then handles the conversion.

### B. Explicit Conversion with `urdfdom_py` or `sdformat_wrapper` (Programmatic)

For more control or to inspect the generated SDF, you can use Python libraries.

*   **`urdf_parser_py` (ROS 1, now `urdfdom_py` in ROS 2 context):** This library allows you to parse URDF files in Python. You can then use the parsed structure to manually construct an SDF.
*   **`sdformat_wrapper`:** A Python wrapper for the `sdformat` C++ library, allowing programmatic creation and manipulation of SDF XML.

A command-line tool `urdf_to_sdf` also exists (part of the `sdformat_urdf` package in Gazebo's development tools), but it's often more convenient to leverage the implicit conversion in ROS 2 launch files.

### C. Adding Gazebo-Specific Extensions to URDF

Since URDF lacks native support for Gazebo plugins and some specific physics parameters, a common practice is to embed Gazebo-specific XML tags directly within the URDF. Gazebo's URDF parser knows how to interpret these custom tags.

For example, to add an IMU sensor plugin to a link:

```xml
<link name="base_link">
  <!-- ... existing link properties ... -->
  <gazebo reference="base_link">
    <gravity>true</gravity>
    <self_collide>true</self_collide>
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100</update_rate>
      <visualize>true</visualize>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <ros>
          <namespace>humanoid</namespace>
          <argument>~/out</argument>
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
        <frame_name>base_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
</link>
```

These `<gazebo>` tags are ignored by standard ROS tools (like `rviz2`) but are parsed and utilized by Gazebo when the URDF is loaded. This allows you to keep a single robot description file while still leveraging Gazebo's advanced simulation features.

## 3. Practical Conversion and Augmentation for Humanoids

Consider our `humanoid.urdf` from the previous chapter. To make it truly useful in Gazebo, we need to:

1.  **Add `ros2_control` integration:** This involves adding `<ros2_control>` tags to the URDF, specifying the hardware interfaces (e.g., `PositionJointInterface`) for each joint. This is crucial for controlling the robot via ROS 2 controllers.
2.  **Add Gazebo plugins:** For sensors (IMU, camera, contact sensors for feet) and potentially for the base link to simulate its interaction with the world.
3.  **Define materials:** For visual realism and accurate friction properties.

### A. Example: `ros2_control` Integration in URDF

```xml
<?xml version="1.0"?>
<robot name="humanoid">
  <!-- ... existing links and joints ... -->

  <ros2_control name="humanoid_robot_controller" type="system">
    <hardware>
      <plugin>gazebo_ros2_control/GazeboSystem</plugin>
    </hardware>
    <joint name="left_shoulder_pitch_joint"> <!-- Example joint from previous chapter -->
      <command_interface name="position">
        <param name="min">-1.57</param>
        <param name="max">1.57</param>
      </command_interface>
      <state_interface name="position"/>
      <state_interface name="velocity"/>
    </joint>
    <!-- ... define other joints like this ... -->
  </ros2_control>

  <!-- ... Gazebo plugins for individual links (e.g., IMU on torso) ... -->

  <gazebo>
    <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
      <parameters>$(find humanoid_controller)/config/humanoid_controller.yaml</parameters>
    </plugin>
  </gazebo>
</robot>
```
This `<ros2_control>` block tells Gazebo how to interface with the robot's joints using the `gazebo_ros2_control` plugin, which bridges between Gazebo's physics and ROS 2 controllers. The `parameters` tag within the `<plugin>` refers to a YAML configuration file for the controllers.

### B. The Controller Configuration YAML

This YAML file (e.g., `humanoid_controller.yaml`) would specify which controllers to load and their PID gains:

```yaml
# humanoid_controller/config/humanoid_controller.yaml
controller_manager:
  ros__parameters:
    update_rate: 100 # Hz

    left_shoulder_pitch_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    # ... other controllers ...

left_shoulder_pitch_controller:
  ros__parameters:
    joint: left_shoulder_pitch_joint
    pid_gains:
      left_shoulder_pitch_joint:
        p: 100.0
        i: 0.01
        d: 1.0
    # ... other controller specific parameters ...
```

This configuration, combined with the augmented URDF, allows `ros2_control` to manage the simulated humanoid's joints, enabling external ROS 2 nodes to command positions, velocities, or efforts.

## 4. SDF as the Native Gazebo World Format

While URDF can be loaded, creating a `.world` file directly in SDF gives you the most control over the entire simulation environment. This is where you define not just the robot, but also:
*   **Ground Plane:** Essential for contact.
*   **Lights:** Ambient, directional (sun), point lights.
*   **Static Objects:** Walls, furniture, obstacles.
*   **Sensors:** Defined directly within models or as standalone entities.

**Example: Simple SDF World with a Humanoid Model**

```xml
<?xml version="1.0" ?>
<sdf version="1.5">
  <world name="humanoid_world">
    <include>
      <uri>model://sun</uri>
    </include>
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <model name="humanoid_robot">
      <include>
        <uri>model://humanoid_model_description</uri> <!-- Reference to our humanoid model -->
        <name>humanoid</name>
        <pose>0 0 0.9 0 0 0</pose> <!-- Initial pose -->
      </include>
      <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
        <parameters>$(find humanoid_controller)/config/humanoid_controller.yaml</parameters>
      </plugin>
    </model>

    <!-- Optionally add other static models or sensors -->
    <model name="table">
      <pose>1 0 0.7 0 0 0</pose>
      <link name="link">
        <visual>
          <geometry><box><size>1 0.5 0.7</size></box></geometry>
        </visual>
        <collision>
          <geometry><box><size>1 0.5 0.7</size></box></geometry>
        </collision>
      </link>
    </model>

  </world>
</sdf>
```
In this SDF world file, `model://humanoid_model_description` would typically refer to a directory structure within Gazebo's model path, containing the SDF of your humanoid robot (which might have been generated from an augmented URDF).

## Conclusion

While URDF is excellent for defining the robot itself, SDF provides the necessary richness to describe an entire simulation environment in Gazebo. The implicit conversion of URDF to SDF is often sufficient for basic robot loading, but for advanced features like `ros2_control` integration and custom Gazebo plugins, augmenting the URDF with `<gazebo>` tags or manually creating SDF files becomes necessary. Mastering this interplay is key to creating robust and realistic digital twins for humanoid robots.