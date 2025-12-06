# Simple Humanoid ROS Controller Mini-Project

This mini-project guides you through building a basic ROS 2 controller for a simulated humanoid robot. You will learn to publish motor commands to control its joints and subscribe to sensor data to monitor its state. This project solidifies the foundational concepts of ROS 2 nodes, topics, and messages.

## Project Goal

Create a ROS 2 package that can:
1.  Publish position commands to a simulated humanoid joint (e.g., a simple pendulum-like joint or a single leg joint).
2.  Subscribe to the simulated joint's state feedback to confirm movement.
3.  Simulate a humanoid robot (even a very simple one) in Gazebo or Isaac Sim (conceptual, as full simulation setup is in later modules).
4.  Demonstrate basic oscillatory motion of a joint.

## Prerequisites

*   A working ROS 2 Humble installation (as covered in previous chapters).
*   Your `humanoid_ws` workspace set up.
*   Familiarity with creating and building ROS 2 Python packages.

## 1. Project Setup

We will build upon the `humanoid_controller` package created in the "Building ROS 2 Packages" chapter. If you haven't created it, please do so now:

```bash
cd ~/humanoid_ws/src
ros2 pkg create --build-type ament_python humanoid_controller --dependencies rclpy std_msgs sensor_msgs
```

## 2. Defining the Robot Model (Conceptual URDF)

For this mini-project, we'll assume a very basic robot model with at least one revolute joint. The `robots/humanoid.urdf` file created earlier can be extended. Let's conceptually focus on a single joint for this controller.

Suppose we want to control a joint named `left_shoulder_pitch_joint`.

## 3. Implementing the Joint Controller Node

We will create a Python node that publishes desired joint positions. The `joint_position_publisher.py` script from the "Motor Commands, Sensors, Control Loops" chapter is a perfect starting point.

**`~/humanoid_ws/src/humanoid_controller/humanoid_controller/joint_position_publisher.py`**
(If you haven't already, create this file or modify the existing `simple_publisher.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64 # Assuming controller expects Float64 for position
# We'll also subscribe to JointState for feedback
from sensor_msgs.msg import JointState

class JointController(Node):
    def __init__(self):
        super().__init__('humanoid_joint_controller')

        # Declare parameters for joint name and oscillation limits
        self.declare_parameter('joint_to_control', 'left_shoulder_pitch_joint')
        self.declare_parameter('max_angle_rad', 0.5) # Max oscillation angle
        self.declare_parameter('min_angle_rad', -0.5) # Min oscillation angle
        self.declare_parameter('oscillation_speed', 0.1) # Radians per second approximately

        self.joint_name = self.get_parameter('joint_to_control').get_parameter_value().string_value
        self.max_angle = self.get_parameter('max_angle_rad').get_parameter_value().double_value
        self.min_angle = self.get_parameter('min_angle_rad').get_parameter_value().double_value
        self.oscillation_speed = self.get_parameter('oscillation_speed').get_parameter_value().double_value

        # Publisher for the joint position command
        # Topic name should match what the simulated controller expects, e.g., /<joint_name>/commands
        self.command_publisher = self.create_publisher(Float64, f'/{self.joint_name}/commands', 10)

        # Subscriber for joint state feedback
        self.joint_state_subscriber = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.timer_period = 0.05 # 20 Hz update rate for control
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.target_position = 0.0
        self.current_direction = 1 # 1 for increasing, -1 for decreasing

        self.get_logger().info(f'Humanoid Joint Controller for {self.joint_name} started.')
        self.get_logger().info(f'Oscillating between {self.min_angle:.2f} and {self.max_angle:.2f} radians.')

    def joint_state_callback(self, msg: JointState):
        """Callback to process incoming joint state messages."""
        if self.joint_name in msg.name:
            try:
                index = msg.name.index(self.joint_name)
                current_pos = msg.position[index]
                # self.get_logger().info(f'Received current position for {self.joint_name}: {current_pos:.2f} radians')
            except ValueError:
                self.get_logger().warn(f"Joint '{self.joint_name}' not found in /joint_states message.")
        
    def timer_callback(self):
        """Timer callback to publish new target positions."""
        # Update target position
        self.target_position += self.oscillation_speed * self.timer_period * self.current_direction

        # Reverse direction if limits are reached
        if self.target_position >= self.max_angle:
            self.target_position = self.max_angle # Clamp to max
            self.current_direction = -1
        elif self.target_position <= self.min_angle:
            self.target_position = self.min_angle # Clamp to min
            self.current_direction = 1

        # Publish the new target position
        command_msg = Float64()
        command_msg.data = self.target_position
        self.command_publisher.publish(command_msg)
        self.get_logger().info(f'Commanding {self.joint_name} to: {self.target_position:.2f} radians')

def main(args=None):
    rclpy.init(args=args)
    joint_controller_node = JointController()
    rclpy.spin(joint_controller_node)
    joint_controller_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. Updating `setup.py`

Ensure that your `~/humanoid_ws/src/humanoid_controller/setup.py` file includes the entry point for your new node.

```python
# ... (other imports and setup details) ...

setup(
    name=package_name,
    # ...
    entry_points={
        'console_scripts': [
            'humanoid_joint_controller = humanoid_controller.joint_position_publisher:main',
        ],
    },
)
```

## 5. Building and Running the Controller

### A. Build Your Package

Navigate to your workspace root and build the `humanoid_controller` package:

```bash
cd ~/humanoid_ws
colcon build --packages-select humanoid_controller
```

### B. Source Your Workspace

After a successful build, source your workspace to make the new executable available:

```bash
source install/setup.bash
```

### C. Running the Node

You can now run your controller node. In a real scenario, this node would typically interact with a `ros2_control` hardware interface or a simulated joint controller.

```bash
ros2 run humanoid_controller humanoid_joint_controller
```

You should see log messages indicating the controller is publishing target positions for `left_shoulder_pitch_joint`.

### D. Verification (Conceptual with Simulation)

To truly verify this, you would need a simulated robot in Gazebo or Isaac Sim with a `left_shoulder_pitch_joint` that is configured to receive commands on `/left_shoulder_pitch_joint/commands` and publish its state on `/joint_states`.

**Conceptual Simulation Setup (Detailed in Module 2):**
1.  **Robot Model:** A URDF/SDF model of your humanoid with a `left_shoulder_pitch_joint`.
2.  **Controller Configuration:** A `ros2_control` configuration that maps the `left_shoulder_pitch_joint` to a position controller, listening on `/left_shoulder_pitch_joint/commands`.
3.  **Gazebo/Isaac Sim Integration:** The simulation environment would then be launched with this robot and controller, providing `joint_states` feedback and receiving `commands`.

When the `humanoid_joint_controller` node is running and the simulation is active, you would observe the `left_shoulder_pitch_joint` of your simulated humanoid oscillating between -0.5 and 0.5 radians.

## 6. Going Further

*   **Implement a Subscriber:** Extend this project by adding a subscriber in the same node or a separate node that listens to `/joint_states` and logs the *actual* position of `left_shoulder_pitch_joint`.
*   **PID Control:** Integrate the PID controller concept from the previous chapter to achieve more precise control, ensuring the actual position closely tracks the desired position.
*   **Multiple Joints:** Modify the node to control multiple joints simultaneously, using `JointTrajectory` messages for coordinated motion.
*   **Parameters:** Experiment with dynamically changing the `max_angle_rad` or `oscillation_speed` using `ros2 param set` while the node is running.

This mini-project provides a practical foundation for bringing your simulated humanoid robots to life within the ROS 2 ecosystem.