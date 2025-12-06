---
id: motor-commands-sensors-control-loops
title: Motor Commands, Sensors, Control Loops
slug: /modules/ros2-nervous-system/motor-commands-sensors-control-loops
---

# Motor Commands, Sensors, Control Loops

The essence of a physical robot lies in its ability to interact with the world: to move, to feel, and to react. This interaction is fundamentally governed by **motor commands** that drive actuators, **sensor data** that provides perception of the robot's internal and external state, and **control loops** that tirelessly work to bridge the gap between desired behavior and actual performance. In the context of ROS 2, these elements are orchestrated through a modular framework, allowing for flexible and robust robot control.

This chapter will delve into the practical aspects of sending motor commands to a humanoid robot's joints, reading and interpreting various sensor data, and implementing foundational control loops in ROS 2 using `rclpy`.

## 1. Motor Commands: Driving the Actuators

To make a humanoid robot move, we need to send specific instructions to its motors (actuators). In ROS 2, this is typically done by publishing messages to topics that the motor controllers subscribe to. The type of message depends on the level of control desired: position, velocity, or effort (torque).

### A. Joint State and Control Interfaces

Robot motor controllers often expose interfaces to control specific aspects of a joint:
*   **Position Control:** Command a desired joint angle (e.g., "move the elbow to 90 degrees"). The internal controller handles the speed and torque.
*   **Velocity Control:** Command a desired joint speed (e.g., "rotate the shoulder at 1 radian/second").
*   **Effort/Torque Control:** Command a specific force or torque to be applied by the joint (e.g., "apply 5 Nm to the hip"). This offers the most direct control but requires more sophisticated control loops.

For humanoid robots, position and velocity control are very common for high-level tasks, while effort control might be used for specialized force-feedback applications or whole-body impedance control.

### B. Publishing Joint Commands (Example: Position Control)

The standard ROS 2 message type for joint control is often `sensor_msgs/msg/JointState` or a custom message depending on the robot. However, for commanding specific joint positions, a simple `std_msgs/msg/Float64` (for a single joint) or an array of `Float64` (for multiple joints) can be used, if the motor controller is set up to interpret it. More commonly, specialized messages from `ros2_control` are used.

Let's assume a simplified scenario where we publish desired joint positions to a topic that a simulated or real motor controller subscribes to.

```python
# modules/humanoid_ros/humanoid_ros/joint_position_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64 # For a single joint position command
# For more complex multi-joint control, you might use:
# from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
# Or specific messages from ros2_control if a controller is loaded

class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__(\'joint_position_publisher\')
        # Publisher for a specific joint (e.g., 'left_shoulder_joint_position_controller/command')
        # The topic name depends on how your controller is set up
        self.publisher_ = self.create_publisher(Float64, \'/left_shoulder_joint_position_controller/command\', 10)
        self.timer = self.create_timer(1.0, self.timer_callback) # Publish every 1 second
        self.position = 0.0
        self.direction = 1

    def timer_callback(self):
        msg = Float64()
        msg.data = self.position

        self.publisher_.publish(msg)
        self.get_logger().info(f'Commanding left_shoulder to: {msg.data:.2f} radians')

        # Oscillate between -0.5 and 0.5 radians
        self.position += 0.1 * self.direction
        if self.position >= 0.5:
            self.direction = -1
        elif self.position <= -0.5:
            self.direction = 1

def main(args=None):
    rclpy.init(args=args)
    joint_publisher = JointPositionPublisher()
    rclpy.spin(joint_publisher)
    joint_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == \'__main__\':
    main()
```
To make this executable, add it to `setup.py` under `entry_points`:
```python
            \'console_scriptssquo': [
                # ... other entry points ...
                \'joint_position_publisher = humanoid_ros.joint_position_publisher:main\',
            ],
```
Then `colcon build` and `source install/setup.bash`. This node would then publish target positions for a specific joint, assuming a controller is listening on that topic.

## 2. Sensor Data: Perceiving the Environment and Robot State

Sensors provide the robot with its perception of the world and its own internal state. In ROS 2, sensor drivers typically publish data to specific topics. Our humanoid robot will rely on a variety of sensors.

### A. Common Sensor Types for Humanoids

*   **Joint Encoders/Potentiometers:** Provide actual joint angles or positions. Often published via `sensor_msgs/msg/JointState`.
*   **IMU (Inertial Measurement Unit):** Provides orientation (roll, pitch, yaw), angular velocity, and linear acceleration. Published via `sensor_msgs/msg/Imu`. Essential for balance and localization.
*   **Cameras (RGB, Depth):** Provide visual and depth information. Published via `sensor_msgs/msg/Image`, `sensor_msgs/msg/CameraInfo`. Crucial for perception, SLAM, and object recognition.
*   **LiDAR (Light Detection and Ranging):** Provides 2D or 3D point cloud data for mapping and obstacle detection. Published via `sensor_msgs/msg/LaserScan` (2D) or `sensor_msgs/msg/PointCloud2` (3D).
*   **Force/Torque Sensors:** Measure forces/torques applied at end-effectors or feet. Published via `geometry_msgs/msg/WrenchStamped`. Critical for grasping and compliant interaction.
*   **Microphones:** For voice commands and sound source localization. Published via `audio_common_msgs/msg/AudioData` or similar.

### B. Subscribing to Sensor Data (Example: IMU)

```python
# modules/humanoid_ros/humanoid_ros/imu_subscriber.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu # Import the IMU message type
import math

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__(\'imu_subscriber\')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data', # Typical topic for IMU data
            self.imu_callback,
            10)
        self.subscription # prevent unused variable warning
        self.get_logger().info('IMU subscriber node started, waiting for data on /imu/data')

    def imu_callback(self, msg: Imu):
        # Extract orientation (quaternion)
        orientation_q = msg.orientation
        
        # Convert quaternion to Euler angles (for easier human interpretation)
        # Note: tf_transformations package needed for this (e.g., `pip install transformations`)
        # For actual robot control, usually work directly with quaternions or rotation matrices
        
        # This is a conceptual example for logging, actual conversion logic might be in a utility
        # from tf_transformations import euler_from_quaternion
        # (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w])
        # self.get_logger().info(f'Orientation: Roll={math.degrees(roll):.2f}, Pitch={math.degrees(pitch):.2f}, Yaw={math.degrees(yaw):.2f} degrees')

        # Extract angular velocity
        angular_velocity = msg.angular_velocity

        # Extract linear acceleration
        linear_acceleration = msg.linear_acceleration
        
        self.get_logger().info(f'IMU Data - Ang Vel (X,Y,Z): ({angular_velocity.x:.2f}, {angular_velocity.y:.2f}, {angular_velocity.z:.2f}) rad/s')
        self.get_logger().info(f'IMU Data - Lin Acc (X,Y,Z): ({linear_acceleration.x:.2f}, {linear_acceleration.y:.2f}, {linear_acceleration.z:.2f}) m/s^2')

def main(args=None):
    rclpy.init(args=args)
    imu_subscriber = ImuSubscriber()
    rclpy.spin(imu_subscriber)
    imu_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == \'__main__\':
    main()
```
To make this executable, add it to `setup.py` under `entry_points`:
```python
            \'console_scriptssquo': [
                # ... other entry points ...
                \'imu_subscriber = humanoid_ros.imu_subscriber:main\',
            ],
```
Then `colcon build` and `source install/setup.bash`. This node will listen for IMU data and log it.

## 3. Control Loops: Bridging Desired and Actual States

A control loop is the feedback mechanism that enables a robot to achieve and maintain a desired state. For humanoid robots, control loops are fundamental for everything from joint position regulation to maintaining balance. The most common and fundamental control loop is the **PID (Proportional-Integral-Derivative) controller**.

### A. PID Control Theory

A PID controller calculates an "error" value as the difference between a desired setpoint (e.g., target joint angle) and a measured process variable (e.g., actual joint angle from encoder). It then computes a control output (e.g., motor torque) by summing three terms:

$ \text{Output}(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{de(t)}{dt} $

Where:
*   $ e(t) $: Error at time $t$ (setpoint - measured value)
*   $ K_p $: Proportional gain (responds to current error)
*   $ K_i $: Integral gain (responds to accumulated error)
*   $ K_d $: Derivative gain (responds to rate of change of error)

**Figure 3.1: PID Control Loop Diagram**
```
+------------------+     +-----------------+     +-----------------+     +------------------+
|    Setpoint      |--->|     Error       |--->|     PID       |--->|   Process (Robot/ |
| (Desired Value)  |    |   Calculation   |    |  Controller   |    |      Motor)      |
+------------------+ ^   +-----------------+    +-----------------+    +------------------+
                     |
                     |
                     +-------------------------------------------------------------+
                                     Measured Value (Sensor Feedback)
```

### B. Implementing a Basic PID Controller in ROS 2

In a real ROS 2 system, PID controllers are often implemented within the motor controller hardware or by higher-level frameworks like `ros2_control`. However, for understanding, let's conceptualize a simple PID controller node that takes a target position and current position, and publishes an effort command.

```python
# modules/humanoid_ros/humanoid_ros/simple_pid_controller.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64 # For command and feedback
from sensor_msgs.msg import JointState # To get actual joint position

class SimplePIDController(Node):
    def __init__(self):
        super().__init__(\'simple_pid_controller\')
        self.declare_parameter('kp', 5.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.2)
        self.declare_parameter('joint_name', \'left_shoulder_joint\')
        self.declare_parameter('target_position', 0.0)

        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value
        self.joint_name = self.get_parameter('joint_name').get_parameter_value().string_value
        self.target_position = self.get_parameter('target_position').get_parameter_value().double_value

        self.get_logger().info(f'PID for {self.joint_name} initialized with Kp={self.kp}, Ki={self.ki}, Kd={self.kd}, Target={self.target_position}')

        self.effort_publisher_ = self.create_publisher(Float64, f'/{self.joint_name}/effort_controller/command', 10)
        self.joint_state_subscriber_ = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.error_sum = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

        self.update_timer = self.create_timer(0.01, self.update_control) # Control loop runs at 100 Hz

    def joint_state_callback(self, msg: JointState):
        if self.joint_name in msg.name:
            try:
                index = msg.name.index(self.joint_name)
                self.current_position = msg.position[index]
            except ValueError:
                self.get_logger().warn(f'Joint \'{self.joint_name}\' not found in /joint_states message.')
        
    def update_control(self):
        # Ensure we have received current position data
        if not hasattr(self, 'current_position'):
            return

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9 # Convert to seconds

        if dt == 0: # Avoid division by zero
            return

        error = self.target_position - self.current_position
        self.error_sum += error * dt
        error_derivative = (error - self.last_error) / dt

        output_effort = (self.kp * error) + (self.ki * self.error_sum) + (self.kd * error_derivative)

        effort_msg = Float64()
        effort_msg.data = output_effort
        self.effort_publisher_.publish(effort_msg)

        self.get_logger().info(f'Joint \'{self.joint_name}\' - Current: {self.current_position:.2f}, Target: {self.target_position:.2f}, Error: {error:.2f}, Effort: {output_effort:.2f}')

        self.last_error = error
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    pid_controller = SimplePIDController()
    rclpy.spin(pid_controller)
    pid_controller.destroy_node()
    rclpy.shutdown()

if __name__ == \'__main__\':
    main()
```
This simplified PID controller would subscribe to `/joint_states` (published by the robot's state publisher or simulator) and publish desired efforts to a specific joint's command topic. The `target_position` could be set as a parameter or subscribed from another topic.

### C. Tuning PID Gains

Tuning $K_p$, $K_i$, and $K_d$ is often an iterative process, usually done by:
1.  **Start with $K_p$:** Increase $K_p$ until the system oscillates around the target.
2.  **Add $K_d$:** Increase $K_d$ to dampen oscillations.
3.  **Add $K_i$:** Increase $K_i$ to eliminate steady-state error (the difference between target and actual after settling).

Incorrectly tuned PID gains can lead to instability, overshoot, or sluggish response.

## Conclusion

Effective humanoid robot control hinges on the seamless integration of motor commands, sensor data, and well-tuned control loops. ROS 2 provides the necessary communication primitives for these components to interact. By mastering how to publish commands, subscribe to sensor feedback, and implement control strategies like PID, you lay the groundwork for developing dynamic and responsive humanoid behaviors.