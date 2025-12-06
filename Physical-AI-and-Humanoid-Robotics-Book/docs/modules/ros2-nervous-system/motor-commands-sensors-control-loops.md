---
id: motor-commands-sensors-control-loops
title: 'Motor Commands, Sensors, Control Loops'
slug: /modules/ros2-nervous-system/motor-commands-sensors-control-loops
---

# Motor Commands, Sensors, Control Loops

Robots move by sending motor commands, feel the world through sensors, and maintain stability using control loops. In ROS 2, these systems communicate using topics, publishers, subscribers, and controllers.

---

## 1. Motor Commands (Actuator Control)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class JointPositionPublisher(Node):
    def __init__(self):
        super().__init__("joint_position_publisher")

        self.publisher_ = self.create_publisher(
            Float64,
            "/left_shoulder_joint_position_controller/command",
            10
        )

        self.position = 0.0
        self.direction = 1

        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        msg = Float64()
        msg.data = self.position
        self.publisher_.publish(msg)

        self.position += 0.1 * self.direction

        if self.position >= 0.5:
            self.direction = -1
        if self.position <= -0.5:
            self.direction = 1

def main(args=None):
    rclpy.init(args=args)
    node = JointPositionPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuSubscriber(Node):
    def __init__(self):
        super().__init__("imu_subscriber")
        self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)

    def imu_callback(self, msg):
        ang = msg.angular_velocity
        lin = msg.linear_acceleration

        self.get_logger().info(
            f"Angular: {ang.x:.2f}, {ang.y:.2f}, {ang.z:.2f}"
        )

        self.get_logger().info(
            f"Linear: {lin.x:.2f}, {lin.y:.2f}, {lin.z:.2f}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ImuSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class SimplePIDController(Node):
    def __init__(self):
        super().__init__("simple_pid_controller")

        self.kp = 5.0
        self.ki = 0.1
        self.kd = 0.2

        self.joint_name = "left_shoulder_joint"
        self.target_position = 0.0

        self.error_sum = 0.0
        self.last_error = 0.0
        self.current_position = None
        self.last_time = self.get_clock().now()

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_callback,
            10
        )

        self.publisher_ = self.create_publisher(
            Float64,
            f"/{self.joint_name}/effort_controller/command",
            10
        )

        self.create_timer(0.01, self.update_control)

    def joint_callback(self, msg):
        if self.joint_name in msg.name:
            i = msg.name.index(self.joint_name)
            self.current_position = msg.position[i]

    def update_control(self):
        if self.current_position is None:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return

        error = self.target_position - self.current_position
        self.error_sum += error * dt
        derivative = (error - self.last_error) / dt

        effort = (
            self.kp * error +
            self.ki * self.error_sum +
            self.kd * derivative
        )

        msg = Float64()
        msg.data = effort
        self.publisher_.publish(msg)

        self.last_error = error
        self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = SimplePIDController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```
