# Humanoid Walking + Sensor Visualization Mini-Project

This mini-project guides you through setting up a simulated humanoid robot in Gazebo, implementing a basic walking motion, and visualizing its sensor data (e.g., IMU). This project integrates concepts from URDF, SDF, ROS 2 control, and sensor simulation, creating a functional digital twin.

## Project Goal

1.  Launch a Gazebo simulation with a humanoid robot model.
2.  Implement a basic walking pattern for the humanoid's legs (e.g., oscillatory motion of hip/knee joints).
3.  Visualize the robot's state and sensor data (e.g., IMU orientation) in `rviz2`.

## Prerequisites

*   A working ROS 2 Humble installation.
*   Gazebo installed and configured for ROS 2.
*   Your `humanoid_ws` workspace set up.
*   Familiarity with URDF, ROS 2 packages, launch files, and `ros2_control` concepts.

## 1. Project Setup: `humanoid_sim` Package

We will create a new ROS 2 package called `humanoid_sim` to house our simulation-specific files (launch files, world files, controller configurations).

```bash
cd ~/humanoid_ws/src
ros2 pkg create --build-type ament_python humanoid_sim --dependencies rclpy gazebo_ros ros2_control_test_assets robot_state_publisher
```

### A. Robot Description (`humanoid_description` Package)

For this project, we'll assume your URDF model is in a separate package, `humanoid_description`, for better modularity.

**`~/humanoid_ws/src/humanoid_description/package.xml`**
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>humanoid_description</name>
  <version>0.0.0</version>
  <description>URDF and meshes for the humanoid robot.</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>ament_python</buildtool_depend>

  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>urdf</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>ros2_control</exec_depend>
  <exec_depend>gazebo_ros2_control</exec_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```
**`~/humanoid_ws/src/humanoid_description/setup.py`**
```python
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'humanoid_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.dae')), # If you have meshes
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')), # For controllers
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='URDF and meshes for the humanoid robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
```

**`~/humanoid_ws/src/humanoid_description/urdf/humanoid.urdf`**
*   Create a simple URDF that includes at least two legs with `hip_pitch`, `knee_pitch`, and `ankle_pitch` joints, configured for `ros2_control` as shown in "URDF to SDF Conversion" chapter.
*   Crucially, add an IMU sensor plugin to the `base_link` as shown in "Sensor Simulation" chapter.

### B. Controller Configuration (`humanoid_sim` Package)

**`~/humanoid_ws/src/humanoid_sim/config/humanoid_controller.yaml`**
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100 # Hz

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

    # For simple oscillatory control, we'll use a position controller
    # This assumes your URDF defines 'position' interfaces for the relevant joints
    leg_position_controller:
      type: position_controllers/JointGroupPositionController # Use JointGroupPositionController
                                                             # for controlling multiple joints at once
```

**`~/humanoid_ws/src/humanoid_sim/launch/humanoid_walking_sim.launch.py`**
This launch file will start Gazebo, spawn the humanoid, load `ros2_control`, and load the necessary controllers. Refer to the "ROS 2 Integration with Simulation" chapter for a detailed example.

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    humanoid_description_package_path = get_package_share_directory('humanoid_description')
    humanoid_sim_package_path = get_package_share_directory('humanoid_sim')

    # Path to the augmented URDF file
    urdf_path = os.path.join(humanoid_description_package_path, 'urdf', 'humanoid.urdf')

    # Path to the controller configuration YAML
    controller_config_path = os.path.join(humanoid_sim_package_path, 'config', 'humanoid_controller.yaml')

    # Start Gazebo simulation (empty world for simplicity)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={'world': os.path.join(humanoid_sim_package_path, 'worlds', 'empty.world')}.items()
    )

    # Robot State Publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': open(urdf_path).read()}],
        output='screen'
    )

    # Spawn the robot entity in Gazebo
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'humanoid'],
        output='screen'
    )

    # Load the controller manager and other necessary controllers
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    load_leg_position_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_position_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Simple Python node to generate walking commands
    walking_commander_node = Node(
        package='humanoid_sim', # Assuming this node is in humanoid_sim
        executable='walking_commander',
        name='walking_commander',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        gazebo_launch,
        robot_state_publisher_node,
        spawn_entity_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity_node,
                on_exit=[
                    load_joint_state_broadcaster,
                    load_leg_position_controller,
                    walking_commander_node,
                ]
            )
        )
    ])
```

### C. Walking Commander Node (`humanoid_sim` Package)

We need a Python node to publish commands to the `leg_position_controller`.

**`~/humanoid_ws/src/humanoid_sim/humanoid_sim/walking_commander.py`**
```python
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import Imu # For visualization purposes

import math
import time

class WalkingCommander(Node):
    def __init__(self):
        super().__init__('walking_commander')
        self.declare_parameter('control_frequency', 50) # Hz
        self.control_frequency = self.get_parameter('control_frequency').get_parameter_value().integer_value

        self.joint_names = ['left_hip_pitch_joint', 'left_knee_pitch_joint',
                            'right_hip_pitch_joint', 'right_knee_pitch_joint'] # Example joints
        
        # Publisher for joint trajectory commands
        # Topic name should match your position controller's command topic
        self.trajectory_publisher = self.create_publisher(JointTrajectory, '/leg_position_controller/joint_trajectory', 10)

        # IMU subscriber for visualization
        self.imu_subscriber = self.create_subscription(Imu, '/imu_data', self.imu_callback, 10) # Assuming /imu_data topic

        self.timer_period = 1.0 / self.control_frequency # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Walking Commander node started.')
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def imu_callback(self, msg: Imu):
        # Basic logging for verification
        orientation_q = msg.orientation
        self.get_logger().info(f'IMU Orientation (Q): x={orientation_q.x:.2f}, y={orientation_q.y:.2f}, z={orientation_q.z:.2f}, w={orientation_q.w:.2f}')
        
    def timer_callback(self):
        """Generates a simple oscillatory walking pattern."""
        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time

        # Simple sine wave for walking motion
        amplitude = 0.3 # radians
        frequency = 0.5 # Hz
        
        # Hip pitch oscillates
        left_hip_target = amplitude * math.sin(2 * math.pi * frequency * current_time)
        right_hip_target = -amplitude * math.sin(2 * math.pi * frequency * current_time) # Opposite phase
        
        # Knee pitch oscillates (e.g., in phase with hip for simplicity, or slightly offset)
        left_knee_target = -amplitude * 0.5 * math.sin(2 * math.pi * frequency * current_time)
        right_knee_target = amplitude * 0.5 * math.sin(2 * math.pi * frequency * current_time) # Opposite phase

        # Create JointTrajectory message
        joint_trajectory_msg = JointTrajectory()
        joint_trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        joint_trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = [left_hip_target, left_knee_target, right_hip_target, right_knee_target]
        point.time_from_start.sec = 0 # Execute immediately
        joint_trajectory_msg.points.append(point)

        self.trajectory_publisher.publish(joint_trajectory_msg)
        self.get_logger().info(f'Published joint commands: {point.positions}')

def main(args=None):
    rclpy.init(args=args)
    walking_commander = WalkingCommander()
    rclpy.spin(walking_commander)
    walking_commander.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### D. Update `humanoid_sim/setup.py`

Add `walking_commander` as an entry point:

```python
# ... (imports) ...
package_name = 'humanoid_sim'
setup(
    # ...
    entry_points={
        'console_scripts': [
            'walking_commander = humanoid_sim.walking_commander:main',
        ],
    },
)
```

## 2. Running the Simulation and Visualization

### A. Build Your Workspace

```bash
cd ~/humanoid_ws
colcon build --packages-select humanoid_description humanoid_sim
```

### B. Source Your Workspace

```bash
source install/setup.bash
```

### C. Launch the Simulation

```bash
ros2 launch humanoid_sim humanoid_walking_sim.launch.py
```
This should launch Gazebo with your humanoid. The `walking_commander` node will start publishing joint commands, and you should see the humanoid's legs moving in an oscillatory pattern. The `imu_subscriber` (from a previous chapter's concept) or the `walking_commander` itself will log IMU data.

### D. Visualize in `rviz2`

In a new terminal, source your workspace and launch `rviz2`:

```bash
source install/setup.bash
rviz2
```

In `rviz2`:
1.  **Add `RobotModel` display:** This will show your humanoid robot using the `robot_description` parameter.
2.  **Add `TF` display:** To visualize the coordinate frames.
3.  **Add `IMU` display:** If your `imu_subscriber` is active, you can visualize the IMU data.
4.  **Set Fixed Frame:** Set the `Fixed Frame` to `odom` or `world`.

You should see your humanoid moving in `rviz2`, and if configured correctly, the IMU data will be visualized, showing the robot's orientation changes due to the walking motion.

## Troubleshooting

*   **Robot not moving:**
    *   Check `ros2 topic list` to ensure `/leg_position_controller/joint_trajectory` is being published and subscribed to.
    *   Verify `ros2_control` is loading correctly and the controller is active (`ros2 control list_controllers`).
    *   Ensure joint names in `walking_commander.py` match your URDF.
*   **IMU data not appearing:**
    *   Check `ros2 topic list` for `/imu_data`.
    *   Verify the IMU plugin is correctly configured in your URDF.
*   **Robot falls over:**
    *   This is expected for simple oscillatory control! Humanoid walking is extremely complex. This mini-project aims for basic motion, not stable locomotion.
    *   Review Gazebo physics parameters (Chapter: "Gazebo Setup & Physics Engine") if the robot behaves erratically without any commands.

This mini-project provides a hands-on experience in bringing together ROS 2 control and Gazebo simulation for basic humanoid robot movement and sensor visualization, forming a crucial step in building your digital twin.