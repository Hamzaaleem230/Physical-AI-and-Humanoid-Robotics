---
id: urdf-for-humanoids
title: "URDF for Humanoids (Links, Joints, Transmissions)"
slug: /modules/ros2-nervous-system/urdf-for-humanoids
---

# URDF for Humanoids (Links, Joints, Transmissions)

This chapter delves into the Unified Robot Description Format (URDF) for modeling humanoid robots, covering links, joints, and transmissions.

## 1. The Core of URDF: Links and Joints

A URDF file essentially defines a tree-like structure of rigid bodies connected by joints.

### A. Links: The Robot's Body Segments

A `link` element describes a single rigid body of the robot. This could be a torso, a thigh, a forearm, a hand, or a sensor housing. Each link has properties defining its physical and visual characteristics.

**Key `link` properties:**

*   **`inertial`**: Describes the link's mass, center of mass, and inertia matrix. These are crucial for accurate physics simulation.
    *   `mass`: Total mass of the link in kilograms.
    *   `origin`: The offset of the center of mass relative to the link's own frame.
    *   `inertia`: A 3x3 rotational inertia matrix (Ixx, Iyy, Izz, Ixy, Ixz, Iyz).
*   **`visual`**: Describes the appearance of the link, primarily for visualization in tools like `rviz2`.
    *   `origin`: The pose (position and orientation) of the visual element relative to the link's frame.
    *   `geometry`: The shape of the visual element (e.g., `box`, `cylinder`, `sphere`, or `mesh` for complex 3D models like `.dae` or `.stl`).
    *   `material`: Color and texture properties.
*   **`collision`**: Describes the shape of the link used for collision detection in simulations.
    *   `origin`: The pose of the collision element relative to the link's frame.
    *   `geometry`: The shape of the collision element (often simplified compared to visual geometry to reduce computation, e.g., a simple box instead of a detailed mesh).

**Example: Humanoid Torso Link**

```xml
<link name="torso_link">
  <inertial>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <mass value="5.0"/>
    <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.05"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.4 0.4"/> <!-- Example: a rectangular torso -->
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <geometry>
      <box size="0.2 0.4 0.4"/>
    </geometry>
  </collision>
</link>
```

### B. Joints: Connecting the Links

A `joint` element describes how two links are connected and their relative motion. Each joint connects a `parent` link to a `child` link.

**Key `joint` properties:**

*   **`name`**: A unique identifier for the joint.
*   **`type`**: Defines the degrees of freedom (DOF) and motion of the joint.
    *   `revolute`: Rotational joint with a limited range (e.g., elbow, knee).
    *   `continuous`: Rotational joint with unlimited range (e.g., spinning wheel).
    *   `prismatic`: Translational joint with a limited range (e.g., linear actuator).
    *   `fixed`: No motion (used for rigidly attaching sensors or end-effectors).
    *   `planar`: Motion in a plane (2 DOF).
    *   `floating`: Full 6 DOF motion (used for the base_link if the robot is not fixed to the world).
*   **`origin`**: The pose of the joint frame relative to the parent link's frame. This defines *where* the joint physically is on the parent link.
*   **`parent`**: The name of the parent link.
*   **`child`**: The name of the child link.
*   **`axis`**: The axis of rotation or translation for revolute, continuous, and prismatic joints (e.g., `xyz="0 0 1"` for rotation around the Z-axis).
*   **`limit`**: For `revolute` and `prismatic` joints, defines the lower and upper position limits, velocity limits, and effort limits.
*   **`dynamics`**: (Optional) Friction and damping coefficients for the joint, used in simulation.

**Example: Torso to Head Joint**

```xml
<joint name="neck_joint" type="revolute">
  <origin xyz="0 0 0.4" rpy="0 0 0"/> <!-- Joint located 0.4m above torso_link origin -->
  <parent link="torso_link"/>
  <child link="head_link"/>
  <axis xyz="0 0 1"/> <!-- Rotation around Z-axis (yaw) -->
  <limit lower="-1.57" upper="1.57" effort="10" velocity="0.5"/>
  <dynamics damping="0.1" friction="0.01"/>
</joint>
```

**Verbal Diagram: Link-Joint-Link Relationship**
```
+----------------+        +-------------+        +------------+
|  parent_link   |------->|    joint    |------->| child_link |
| (rigid body 1) |        | (articulation)|        | (rigid body 2) |
+----------------+        +-------------+        +------------+
        ^
        |
        |  Defines physical/    | Defines relative     | Is moved by the
        |  visual/collision     | motion between       | parent via the joint
        |  properties           | parent and child     |
```

## 2. Humanoid Robot Kinematic Tree Structure

Humanoid robots typically have a complex kinematic chain, resembling a tree structure with the `base_link` (often the torso or pelvis) as the root.

**Figure 2.1: Simplified Humanoid URDF Tree**
```
          world (fixed reference frame)
            |
            v
        base_link (e.g., torso_link or pelvis_link)
       /    |     \
      v     v       v
 left_hip  neck_joint  right_hip
    |       |       |
    v       v       v
left_thigh head_link right_thigh
    |
    v
left_knee
    |
    v
left_calf
    |
    v
left_ankle
    |
    v
left_foot
```

In addition to the main body, arms, hands, and various sensors would branch off from appropriate links (e.g., a camera attached to the `head_link`).

## 3. Transmissions: Connecting Joints to Actuators

While links and joints describe the robot's kinematics, `transmission` elements provide the crucial link between the mechanical joints and the actuators (motors) that drive them. This is especially important for control.

A `transmission` typically defines the gearing ratio and mechanical properties between a *joint* (which describes motion) and an *actuator* (which exerts effort).

### A. Transmission Element Structure

```xml
<transmission name="neck_yaw_transmission">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="neck_joint">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="neck_yaw_motor">
    <mechanicalReduction>100</mechanicalReduction> <!-- Gearing ratio -->
  </actuator>
</transmission>
```

**Key Elements:**

*   **`name`**: Unique identifier for the transmission.
*   **`type`**: Specifies the type of transmission. `SimpleTransmission` is common for a direct gear reduction between one joint and one actuator. Others exist for more complex setups (e.g., differential, fourbar).
*   **`joint`**: References the `name` of the joint it's connected to.
    *   `hardwareInterface`: Specifies the type of interface this joint expects from the robot's hardware/control system (e.g., `PositionJointInterface`, `VelocityJointInterface`, `EffortJointInterface`). This tells the controller how to interact with the joint.
*   **`actuator`**: Describes the physical actuator.
    *   `name`: Unique identifier for the actuator.
    *   `mechanicalReduction`: The gearing ratio between the actuator and the joint. For every unit of rotation/movement of the actuator, the joint moves by `1/mechanicalReduction`.

### B. Importance for Humanoid Control

Transmissions are vital for:
*   **Control System Design:** They inform the control system about how to translate desired joint commands (e.g., move neck_joint to X radians) into raw actuator commands (e.g., send Y current to neck_yaw_motor).
*   **Physics Simulation:** Accurately models the mechanical properties for physics engines, influencing how forces and torques propagate.
*   **Real-world Hardware Abstraction:** Allows a common control interface (e.g., `PositionJointInterface`) to work across different physical motor/gearbox combinations.

## 4. Building a Complete URDF for a Humanoid (Conceptual Example)

A full humanoid URDF would involve hundreds of lines, connecting each link (pelvis, upper_leg, lower_leg, foot, torso, upper_arm, lower_arm, hand, head, etc.) with appropriate joints (revolute for most human-like rotations, fixed for sensors).

**Example: Snippet for a Simple Leg**

```xml
<!-- Example of a Leg Segment -->
<link name="upper_leg_left">
  <!-- inertial, visual, collision definitions -->
</link>

<joint name="hip_pitch_left" type="revolute">
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <parent link="pelvis_link"/>
  <child link="upper_leg_left"/>
  <axis xyz="0 1 0"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1.0"/>
</joint>

<transmission name="hip_pitch_left_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="hip_pitch_left">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="hip_pitch_left_motor">
    <mechanicalReduction>50</mechanicalReduction>
  </actuator>
</transmission>

<link name="lower_leg_left">
  <!-- inertial, visual, collision definitions -->
</link>

<joint name="knee_pitch_left" type="revolute">
  <origin xyz="0 0 -0.4" rpy="0 0 0"/> <!-- Offset from upper_leg_left origin -->
  <parent link="upper_leg_left"/>
  <child link="lower_leg_left"/>
  <axis xyz="0 1 0"/>
  <limit lower="-2.0" upper="0.0" effort="100" velocity="1.0"/> <!-- Knee only bends backwards -->
</joint>

<transmission name="knee_pitch_left_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="knee_pitch_left">
    <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
  </joint>
  <actuator name="knee_pitch_left_motor">
    <mechanicalReduction>50</mechanicalReduction>
  </actuator>
</transmission>
```

This snippet illustrates how individual links and joints are defined and then linked to their respective actuators via transmissions. The entire robot model is assembled by chaining these elements.

## 5. Limitations of URDF and Alternatives

While powerful, URDF has some limitations:

*   **Single Robot Description:** URDF is designed to describe a single robot. It cannot describe an entire world (e.g., multiple robots, static objects, terrain). For this, SDF (Simulation Description Format, used by Gazebo) is preferred.
*   **No Loop Joints:** URDF describes a kinematic tree, not a graph. Robots with closed kinematic chains (e.g., parallel manipulators, grippers where fingers meet) are difficult to represent directly in URDF.
*   **Limited Dynamics:** Only basic dynamics properties are covered.
*   **No Non-Robot Elements:** Cannot describe lights, cameras, or other static elements of a simulation environment.

For complex simulation environments or for platforms like NVIDIA Isaac Sim (which uses USD - Universal Scene Description), URDF models are often converted or integrated into a richer scene description format. Despite these limitations, URDF remains the cornerstone for defining the basic kinematics and dynamics of a single robot within the ROS ecosystem.

## Conclusion

Mastering URDF is a fundamental step in humanoid robotics. By precisely defining the links, joints, and transmissions of your robot, you establish the foundation for accurate kinematics, dynamics, and control. This detailed description is consumed by a multitude of ROS 2 tools, from visualization in `rviz2` to physics simulation in Gazebo, and is the starting point for bringing your digital humanoid to life.