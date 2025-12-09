---
id: sensor-simulation
title: "Sensor Simulation (LiDAR, IMU, Depth Camera)"
slug: /modules/digital-twin/sensor-simulation
---

# Sensor Simulation (LiDAR, IMU, Depth Camera)

For a humanoid robot to intelligently interact with its environment, it needs accurate and timely information about the world around it and its own internal state. This information is provided by an array of sensors. In a digital twin, these sensors must also be simulated to reflect their real-world counterparts as closely as possible, providing realistic data streams for testing perception algorithms, control loops, and AI models.

This chapter delves into the simulation of key sensor types for humanoid robots within Gazebo: LiDAR, IMU, and Depth Cameras. We will explore how to configure these simulated sensors and access their data streams via ROS 2.

## 1. The Importance of Realistic Sensor Simulation

The "reality gap" between simulation and the real world is often most pronounced in sensor data. Poorly simulated sensor noise, resolution, or field of view can lead to algorithms that work perfectly in simulation but fail catastrophically on hardware. Realistic sensor simulation is vital for:

*   **Algorithm Development:** Testing SLAM, object detection, navigation, and control algorithms.
*   **Synthetic Data Generation:** Creating large, labeled datasets for training deep learning models (especially relevant when real-world data is scarce or expensive).
*   **System Integration Testing:** Ensuring that different robot components (e.g., perception, planning, control) can correctly interpret and utilize sensor information.

Gazebo provides a powerful plugin architecture to simulate various sensors with configurable parameters.

## 2. Simulating an Inertial Measurement Unit (IMU)

An IMU is crucial for a humanoid robot's balance, localization, and motion tracking. It typically consists of accelerometers and gyroscopes, and often magnetometers. Gazebo can simulate an IMU by attaching a sensor plugin to a robot link.

### A. IMU Sensor Plugin Configuration

The `libgazebo_ros_imu_sensor.so` plugin is commonly used for IMU simulation. It needs to be embedded within the `<gazebo>` tag associated with the link where the IMU is mounted (e.g., `base_link` or `torso_link`).

```xml
<!-- Example IMU configuration within humanoid URDF (or SDF) -->
<link name="base_link">
  <!-- ... existing link definitions (inertial, visual, collision) ... -->

  <gazebo reference="base_link">
    <sensor name="imu_sensor" type="imu">
      <always_on>true</always_on>
      <update_rate>100.0</update_rate> <!-- Publish IMU data at 100 Hz -->
      <visualize>true</visualize>
      <imu>
        <noise>
          <type>gaussian</type>
          <!-- Gyroscope noise parameters -->
          <rate>
            <mean>0.0</mean>
            <stddev>0.0002</stddev> <!-- Standard deviation of noise -->
            <bias_mean>0.0000075</bias_mean>
            <bias_stddev>0.0000008</bias_stddev>
          </rate>
          <!-- Accelerometer noise parameters -->
          <accel>
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </accel>
        </noise>
      </imu>
      <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
        <ros>
          <namespace>humanoid</namespace>
          <remapping>~/out:=imu_data</remapping> <!-- Remap Gazebo's internal topic to a user-friendly one -->
        </ros>
        <initial_orientation_as_reference>false</initial_orientation_as_reference>
        <frame_name>base_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
</link>
```

**Key Parameters Explained:**
*   `always_on`: Sensor is active when the simulation starts.
*   `update_rate`: Frequency at which sensor data is generated and published (in Hz). Crucial for real-time control.
*   `noise`: Configures Gaussian noise for gyroscope (rate) and accelerometer (accel) readings. This is essential for realistic simulation.
*   `remapping`: Allows renaming the ROS 2 topic published by the plugin. `~/out:=imu_data` means the internal Gazebo topic `/imu_sensor/out` becomes `/humanoid/imu_data` in ROS 2.
*   `frame_name`: The TF frame from which the IMU data is measured.

### B. Accessing IMU Data in ROS 2

After launching Gazebo with the IMU plugin, you can verify the topic and message type:

```bash
ros2 topic list # You should see /humanoid/imu_data
ros2 interface show sensor_msgs/msg/Imu
```
You can then subscribe to this topic in your ROS 2 nodes, as demonstrated in the "Motor Commands, Sensors, Control Loops" chapter.

## 3. Simulating a Depth Camera

Depth cameras (e.g., Intel RealSense) are vital for perception tasks like object detection, 3D reconstruction, and navigation. They typically provide an RGB image, a depth image, and an infrared image, along with camera intrinsic parameters.

### A. Depth Camera Plugin Configuration

The `libgazebo_ros_depth_camera.so` plugin (or `libgazebo_ros_camera.so` for RGB only) is used for camera simulation. It's attached to the link where the camera is mounted (e.g., `head_link`).

```xml
<!-- Example Depth Camera configuration within humanoid URDF (or SDF) -->
<link name="head_link">
  <!-- ... existing head link definitions ... -->

  <gazebo reference="head_link">
    <sensor name="head_camera" type="depth">
      <always_on>true</always_on>
      <update_rate>30.0</update_rate> <!-- 30 FPS -->
      <camera name="head_camera_sensor">
        <horizontal_fov>1.089</horizontal_fov> <!-- ~62 degrees -->
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>8.0</far>
        </clip>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.007</stddev>
        </noise>
      </camera>
      <plugin name="depth_camera_plugin" filename="libgazebo_ros_depth_camera.so">
        <ros>
          <namespace>humanoid</namespace>
          <argument>head_camera</argument> <!-- Base topic name -->
        </ros>
        <always_on>true</always_on>
        <update_rate>30.0</update_rate>
        <camera_name>head_camera</camera_name>
        <frame_name>head_camera_optical_frame</frame_name>
        <point_cloud_cut_off_max_depth>5.0</point_cloud_cut_off_max_depth>
        <point_cloud_cut_off_min_depth>0.4</point_cloud_cut_off_min_depth>
        <distortion_k1>0.0</distortion_k1> <!-- Example distortion parameters -->
        <distortion_k2>0.0</distortion_k2>
        <distortion_k3>0.0</distortion_k3>
        <distortion_t1>0.0</distortion_t1>
        <distortion_t2>0.0</distortion_t2>
      </plugin>
    </sensor>
  </gazebo>
</link>
```

**Key Parameters Explained:**
*   `type="depth"`: Specifies a depth camera.
*   `camera/image`: Image resolution and format.
*   `camera/clip`: Near and far clipping planes for depth sensing.
*   `camera/noise`: Gaussian noise can be added to depth readings for realism.
*   `plugin/argument`: Defines the base topic name, leading to `/humanoid/head_camera/image_raw`, `/humanoid/head_camera/depth/image_raw`, `/humanoid/head_camera/points` (point cloud), etc.
*   `plugin/frame_name`: The TF frame to which the sensor data is relative.

### B. Accessing Depth Camera Data in ROS 2

The plugin will publish to a set of topics. For example:
*   `/humanoid/head_camera/image_raw` (`sensor_msgs/msg/Image`)
*   `/humanoid/head_camera/depth/image_raw` (`sensor_msgs/msg/Image` - encoding typically `32FC1` for float depth)
*   `/humanoid/head_camera/camera_info` (`sensor_msgs/msg/CameraInfo`)
*   `/humanoid/head_camera/points` (`sensor_msgs/msg/PointCloud2`)

You can subscribe to these topics to process the data in your perception nodes. For example, using `cv_bridge` to convert ROS images to OpenCV images.

## 4. Simulating a LiDAR Sensor

LiDAR (Light Detection and Ranging) sensors provide range measurements to surrounding objects, creating a 2D or 3D point cloud. They are widely used for mapping, localization (SLAM), and obstacle avoidance.

### A. LiDAR Sensor Plugin Configuration

The `libgazebo_ros_ray_sensor.so` plugin is used for simulating 2D and 3D LiDAR. It's usually mounted on a prominent link like the `base_link` or `head_link`.

```xml
<!-- Example LiDAR configuration within humanoid URDF (or SDF) -->
<link name="base_link">
  <!-- ... existing link definitions ... -->

  <gazebo reference="base_link">
    <sensor name="laser_sensor" type="ray"> <!-- Type "ray" for LiDAR -->
      <always_on>true</always_on>
      <update_rate>10.0</update_rate> <!-- 10 Hz -->
      <ray>
        <scan>
          <horizontal>
            <samples>360</samples> <!-- Number of laser beams in horizontal plane -->
            <resolution>1</resolution>
            <min_angle>-3.14159</min_angle> <!-- -180 degrees -->
            <max_angle>3.14159</max_angle>   <!-- +180 degrees -->
          </horizontal>
          <!-- <vertical> defines for 3D LiDAR -->
        </scan>
        <range>
          <min>0.1</min>
          <max>10.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="laser_plugin" filename="libgazebo_ros_ray_sensor.so">
        <ros>
          <namespace>humanoid</namespace>
          <argument>laser_scan</argument>
        </ros>
        <output_type>sensor_msgs/LaserScan</output_type> <!-- Publish as LaserScan (2D) -->
        <frame_name>laser_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
</link>
```

**Key Parameters Explained:**
*   `type="ray"`: Specifies a ray sensor (LiDAR).
*   `ray/scan/horizontal/samples`: Number of horizontal rays (beams).
*   `ray/scan/horizontal/min_angle`, `max_angle`: Field of view of the LiDAR.
*   `ray/range`: Minimum, maximum, and resolution of range measurements.
*   `noise`: Gaussian noise for range readings.
*   `plugin/output_type`: Can be `sensor_msgs/LaserScan` for 2D or `sensor_msgs/PointCloud2` for 3D.

### B. Accessing LiDAR Data in ROS 2

The plugin will publish `sensor_msgs/msg/LaserScan` messages to `/humanoid/laser_scan`. This data is commonly used by the Nav2 stack for obstacle avoidance and local path planning.

## 5. Case Study: Sensor Fusion for Humanoid Localization

For robust localization of a humanoid robot, sensor fusion is critical. A common approach is to fuse IMU data with wheel odometry (for wheeled robots) or visual odometry/SLAM (for legged robots). The `robot_localization` package in ROS 2 provides an Extended Kalman Filter (EKF) or Unscented Kalman Filter (UKF) to achieve this.

**Verbal Diagram: Sensor Fusion for Localization**
```
+----------------+      +-----------------+      +-----------------+
|   IMU Data     |----->|                 |      |                 |
| (`/imu_data`)  |      |                 |      |                 |
+----------------+      |                 |      |                 |
                        |   Robot         |      |                 |
+----------------+      |   Localization  |----->|  Fused Odometry |
| Visual Odometry|----->|   Node (EKF/UKF)|      |  (`/odometry/filtered`) |
| (`/vslam/odom`)|      |                 |      |                 |
+----------------+      |                 |      |                 |
                        |                 |      |                 |
+----------------+      |                 |      |                 |
| LiDAR Scan     |----->|                 |      |                 |
| (`/scan`)      |      |                 |      |                 |
+----------------+      +-----------------+      +-----------------+
```
In this setup, the simulated IMU (`/humanoid/imu_data`) and a hypothetical visual odometry or SLAM system would feed data into the `robot_localization` node, which would then output a more accurate and stable estimate of the robot's pose on `/odometry/filtered`.

## Conclusion

Realistic sensor simulation is a cornerstone of effective digital twin development for humanoid robots. By leveraging Gazebo's powerful sensor plugins, you can generate authentic data streams for IMUs, depth cameras, and LiDAR, enabling comprehensive testing and training of your robot's perception, planning, and control algorithms. This lays the groundwork for bridging the simulation-to-reality gap and deploying truly intelligent physical AI systems.