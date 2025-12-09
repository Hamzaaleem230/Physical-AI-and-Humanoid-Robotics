---
id: launch-files-parameters-tf2
title: "Launch Files, Parameters, TF2"
slug: /modules/ros2-nervous-system/launch-files-parameters-tf2
---

# Launch Files, Parameters, TF2

In a complex robotic system like a humanoid, numerous ROS 2 nodes need to be started, configured, and managed. Manually launching each node and setting its parameters can be tedious and error-prone. This is where **Launch Files** come in, providing a structured way to orchestrate an entire ROS 2 application. Furthermore, managing configurable values within nodes relies on **Parameters**, and understanding the spatial relationships between different parts of the robot and its environment is handled by the **TF2** (Transform Frame) library.

This chapter will explore these three essential components, crucial for building and operating modular ROS 2 applications for humanoid robots.

## 1. Launch Files: Orchestrating ROS 2 Systems

ROS 2 launch files are XML or Python scripts that allow you to define and execute multiple ROS 2 nodes and other commands simultaneously. They provide a powerful mechanism for system orchestration, configuration, and reproducibility.

### A. Why Use Launch Files?

*   **Automation:** Start a complex set of nodes (e.g., camera driver, IMU driver, VSLAM node, motor controller) with a single command.
*   **Configuration:** Assign parameters to nodes and remap topic/service names.
*   **Reproducibility:** Ensure the same set of nodes and configurations are used every time an application is started.
*   **Debugging:** Easily start nodes in specific debug modes or conditionally.

### B. Launch File Structure (Python API)

While XML launch files exist, Python launch files are generally preferred in ROS 2 due to their greater flexibility, programmatic control, and ease of debugging. A Python launch file defines a `generate_launch_description` function that returns a `LaunchDescription` object.

Here's a basic structure:

```python
# my_robot_bringup/launch/my_robot.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='humanoid_controller',
            executable='simple_publisher',
            name='my_publisher',
            output='screen',
            emulate_tty=True, # For print statements to show in console
            parameters=[
                {'my_parameter': 'hello'},
                {'another_param': 1.23}
            ]
        ),
        Node(
            package='humanoid_controller',
            executable='simple_subscriber',
            name='my_subscriber',
            output='screen',
            emulate_tty=True,
            remappings=[
                ('chatter', 'my_remapped_topic') # Remap 'chatter' topic to 'my_remapped_topic'
            ]
        )
    ])
```

**Key Elements:**
*   `LaunchDescription`: The root element that holds all launch actions.
*   `Node`: Represents a ROS 2 node to be launched.
    *   `package`: The name of the ROS 2 package containing the executable.
    *   `executable`: The name of the executable to run (as defined in `setup.py` for Python nodes).
    *   `name`: (Optional) A unique name for this instance of the node. If not provided, ROS 2 assigns a unique name.
    *   `output`: Where to direct the node's output (`screen` is common for debugging).
    *   `emulate_tty`: `True` ensures that `print()` statements in Python nodes appear correctly.
    *   `parameters`: A list of dictionaries or paths to YAML files for configuring node parameters.
    *   `remappings`: A list of tuples for changing topic, service, or action names.

### C. Running a Launch File

To run the above launch file, you would use:

```bash
ros2 launch my_robot_bringup my_robot.launch.py
```
(Assuming the launch file is in a package named `my_robot_bringup` and installed correctly.)

### D. Advanced Launch Features

Launch files can also include:
*   **`IncludeLaunchDescription`**: Include other launch files, allowing for modularity (e.g., including a sensor driver launch file).
*   **`DeclareLaunchArgument`**: Define arguments that can be passed to the launch file from the command line, making configurations dynamic.
*   **Conditional Actions**: `IfCondition`, `UnlessCondition` allow nodes or actions to be started only if certain conditions are met.
*   **Group Actions**: `GroupAction` to apply common remappings or namespaces to a set of nodes.

These features enable the creation of highly flexible and reusable launch systems for complex humanoid configurations.

## 2. Parameters: Dynamic Node Configuration

Parameters in ROS 2 allow nodes to expose configurable values that can be changed at runtime or specified in launch files. This is essential for tuning robot behavior without recompiling code.

### A. Declaring and Accessing Parameters in Python (`rclpy`)

A ROS 2 node must declare its parameters before accessing them.

```python
# humanoid_controller/humanoid_controller/motor_controller.py
import rclpy
from rclpy.node import Node

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # 1. Declare a parameter with a default value
        self.declare_parameter('kp_gain', 0.5)
        self.declare_parameter('max_velocity', 1.0)

        # 2. Get the current value of the parameter
        self.kp_gain = self.get_parameter('kp_gain').get_parameter_value().double_value
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value

        self.get_logger().info(f'Motor Controller initialized with Kp: {self.kp_gain}, Max Velocity: {self.max_velocity}')

        # Example: Timer to periodically update parameter values
        self.timer = self.create_timer(2.0, self.timer_callback)

    def timer_callback(self):
        # 3. Get parameter value at runtime (if it changed)
        new_kp = self.get_parameter('kp_gain').get_parameter_value().double_value
        if new_kp != self.kp_gain:
            self.kp_gain = new_kp
            self.get_logger().info(f'Kp gain updated to: {self.kp_gain}')

def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```


### B. Setting Parameters

1.  **From Command Line:**
    ```bash
    ros2 run humanoid_controller motor_controller --ros-args -p kp_gain:=0.75 -p max_velocity:=2.0
    ```

2.  **Using a YAML File (recommended for complex configs):**
    ```yaml
    # config/motor_controller.yaml
motor_controller:
  ros__parameters:
    kp_gain: 0.75
    max_velocity: 2.0
    joint_names: ["left_shoulder", "right_shoulder"]
    ```
    And in the launch file:
    ```python
    # ... inside LaunchDescription ...
        Node(
            package='humanoid_controller',
            executable='motor_controller',
            name='motor_controller',
            parameters=[
                '/path/to/your/config/motor_controller.yaml' # Absolute path or relative to launch file
            ]
        ),
    # ... 
    ```

3.  **At Runtime:**
    ```bash
    ros2 param set /motor_controller kp_gain 0.8
    ```
    This changes the parameter value of a running node.

Parameters are vital for the flexible configuration of individual robot components, allowing engineers and researchers to fine-tune performance on the fly without recompiling.

## 3. TF2: Managing Coordinate Transformations

In robotics, understanding spatial relationships is critical. A humanoid robot has many components (joints, sensors, end-effectors), and it operates in a dynamic environment with objects. **TF2** (Transform Frame) is the standard ROS 2 library for tracking coordinate frames and transforming data between them.

### A. Why TF2 is Essential for Humanoids

*   **Kinematics:** Calculating the position of the end-effector given joint angles (forward kinematics) or vice-versa (inverse kinematics) requires transformations.
*   **Sensor Fusion:** Combining data from multiple sensors (e.g., camera in robot's head, LiDAR on torso) requires knowing their positions relative to a common frame.
*   **Navigation:** Relocating objects or goals defined in the robot's frame into the world frame, or vice-versa.
*   **Humanoid Specifics:** Bipedal robots constantly shift their center of mass and base frame, making dynamic transformations indispensable for balance and control.

### B. Core Concepts

*   **Frame:** A coordinate system. Examples: `base_link` (robot's body), `head_camera_link`, `left_foot_link`, `world` (global reference).
*   **Transform:** A mathematical description (translation + rotation) of how to get from one frame to another.
*   **`tf2_ros.TransformBroadcaster`**: Publishes transforms to the ROS 2 graph. Often used by drivers to publish sensor frames relative to robot body, or by robot state publishers to publish joint states.
*   **`tf2_ros.TransformListener`**: Listens for transforms and allows querying for transformations between any two frames at any given time.

### C. Publishing Transforms

Typically, the `robot_state_publisher` node (which we'll cover with URDF) publishes transforms for the robot's internal joints. Sensor drivers publish the transforms for sensors mounted on the robot.

Here's a conceptual Python example of a node publishing a static transform (e.g., a fixed sensor on the robot):

```python
# humanoid_controller/humanoid_controller/static_tf_broadcaster.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import tf_transformations

class StaticTFBroadcaster(Node):
    def __init__(self):
        super().__init__('static_tf_broadcaster')
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # Example: Define a static transform for a camera mounted on the robot's head
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        static_transform_stamped.header.frame_id = 'head_link' # Parent frame
        static_transform_stamped.child_frame_id = 'head_camera_link' # Child frame

        # Translate 10cm forward, 5cm up from head_link origin
        static_transform_stamped.transform.translation.x = 0.1
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.05

        # Rotate 90 degrees around Y-axis (looking down)
        quat = tf_transformations.quaternion_from_euler(-1.57, 0.0, 0.0) # Roll, Pitch, Yaw
        static_transform_stamped.transform.rotation.x = quat[0]
        static_transform_stamped.transform.rotation.y = quat[1]
        static_transform_stamped.transform.rotation.z = quat[2]
        static_transform_stamped.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(static_transform_stamped)
        self.get_logger().info(f'Published static transform from {static_transform_stamped.header.frame_id} to {static_transform_stamped.child_frame_id}')

def main(args=None):
    rclpy.init(args=args)
    node = StaticTFBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### D. Listening for Transforms

Nodes that need to use coordinate frames will create a `tf2_ros.Buffer` and `tf2_ros.TransformListener`.

```python
# humanoid_controller/humanoid_controller/transform_listener.py
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped # For example, a point to transform

class TransformTest(Node):
    def __init__(self):
        super().__init__('transform_test_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self) # Pass node for callbacks

        # Example: Point defined in 'head_camera_link' frame
        self.point_in_camera_frame = PointStamped()
        self.point_in_camera_frame.header.frame_id = 'head_camera_link'
        self.point_in_camera_frame.point.x = 0.5 # 0.5m in front of camera
        self.point_in_camera_frame.point.y = 0.1
        self.point_in_camera_frame.point.z = 0.0

        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            # Query the transform from 'head_camera_link' to 'base_link'
            # Wait up to 1 second for the transform to become available
            transform = self.tf_buffer.lookup_transform(
                'base_link', # Target frame
                self.point_in_camera_frame.header.frame_id, # Source frame
                rclpy.time.Time()) # At the latest available time

            self.get_logger().info(f'Transform from head_camera_link to base_link: {transform.transform.translation}')

            # Transform the point
            point_in_base_link = self.tf_buffer.transform(self.point_in_camera_frame, 'base_link')
            self.get_logger().info(f'Point in base_link frame: {point_in_base_link.point}')

        except Exception as e:
            self.get_logger().error(f'Could not transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TransformTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Figure 3.1: TF2 Tree Structure (Conceptual)**
```
          world
            |
            v
        odom_frame
            |
            v
        base_link  (robot\'s main body)
       /    |    \
      v     v     v
left_arm_link  head_link  right_arm_link
                 |
                 v
           head_camera_link
```
TF2 maintains a tree of coordinate frames, where each transform defines the relationship between a parent and child frame. This allows you to query the transformation between any two frames in the tree.

### E. Visualization with `rviz2`

The most powerful way to debug and visualize TF2 transforms is using `rviz2`. You can add a "TF" display type in `rviz2` to see all published frames and their relationships, which is invaluable for complex robot setups.

```bash
rviz2
```
Then, in `rviz2`, add a "TF" display and select the frames you want to visualize.

## Conclusion

Launch files, parameters, and TF2 are cornerstones of robust ROS 2 development for humanoid robots. Launch files automate system startup, parameters enable flexible runtime configuration, and TF2 provides the crucial framework for managing spatial data. Mastering these tools is essential for orchestrating complex multi-node applications and ensuring accurate robot perception and control.