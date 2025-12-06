# Autonomous Humanoid Robot Capstone Project

This capstone project is the culmination of your journey through "Physical AI and Humanoid Robotics." It challenges you to integrate all the concepts learned across the modules—ROS 2 communication, digital twin simulation, NVIDIA Isaac AI capabilities, and Vision-Language-Action (VLA) systems—to build a functionally autonomous humanoid robot capable of understanding and executing voice-controlled commands.

The goal is to demonstrate a full end-to-end VLA pipeline where a natural language voice command leads to the robot performing a physical task involving navigation, perception, and manipulation in a simulated environment.

## Project Goal

Implement a simulated humanoid robot that can:
1.  Receive a high-level voice command (e.g., "Robot, pick up the green sphere and place it on the red cube").
2.  Transcribe the voice command to text using a Whisper-like ASR system.
3.  Utilize an LLM to generate a plan from the text command, decomposing it into a sequence of executable robot actions.
4.  Execute the plan using a VLA Control Graph, performing:
    *   **Navigation:** Moving to a target location in Isaac Sim.
    *   **Perception:** Detecting and localizing objects using simulated camera data and Isaac ROS concepts.
    *   **Manipulation:** Grasping and placing objects.
5.  Provide vocal feedback to the human user regarding task progress or completion.

## Prerequisites

*   Completion of all previous modules and mini-projects.
*   A strong understanding of ROS 2, URDF, Gazebo/Isaac Sim.
*   Familiarity with NVIDIA Isaac ROS, VSLAM, and basic perception.
*   Knowledge of LLM planning concepts.
*   A configured Isaac Sim environment with a humanoid model.
*   Your `humanoid_ws` workspace with all necessary packages built.

## 1. System Architecture Overview

The capstone project will integrate the following key components, most of which you've developed or conceptually explored in previous chapters:

**Figure 1.1: Capstone Project Full VLA System Diagram**
```
+------------------------------------------------------------------------------------------------------------------+
|                                Human Operator                                                                    |
|                                (Voice Command)                                                                    |
+------------------------------------------------------------------------------------------------------------------+
          |                                                                                                  ^
          | (1) Audio Input                                                                                  | (6) Spoken Feedback
          v                                                                                                  |
+------------------------------------------------------------------------------------------------------------------+
|                                ROS 2 COMMUNICATION LAYER (Middleware: DDS)                                      |
|  +-------------------------------------------------------------------------------------------------------------+  |
|  |    (A) ASR (Whisper Node)     /humanoid/voice_commands (String)      (B) LLM Planning Node                  |  |
|  |    (Audio -> Text)           <-----------------------------------------> (Text -> Structured Plan)        |  |
|  +-----------------------------+                                          +-----------------------------+      |  |
|                                                                                ^                              |  |
|                                                                                | /humanoid/robot_state (JSON String) |  |
|                                                                                v                              |  |
|  +-------------------------------------------------------------------------------------------------------------+  |
|  | (C) VLA Control Graph Executive                                                                               |  |
|  |    (Plan -> Low-Level Actions, Manages Flow, Error Handling)                                                  |  |
|  +-------------------------------------------------------------------------------------------------------------+  |
|                                  |                                                                                |
|                                  v (ROS 2 Calls: /navigate_to_pose, /pick_object, /detect_object, /speak)         |
|  +-------------------------------------------------------------------------------------------------------------+  |
|  | (D) NAVIGATION             (E) PERCEPTION                  (F) MANIPULATION                 (G) SPEECH SYNTHESIS |  |
|  |    (Nav2 Adapted)          (Isaac ROS VSLAM/Detect)        (ros2_control, IK)            (Text -> Audio)       |  |
|  +-------------------------------------------------------------------------------------------------------------+  |
|                                  |                                                                                |
|                                  v (ROS 2 Control Layer / Hardware Interfaces)                                  |
+------------------------------------------------------------------------------------------------------------------+
          |                                                                                                  ^
          | (Actuator Commands)                                                                              | (Simulated Sensor Data)
          v                                                                                                  |
+------------------------------------------------------------------------------------------------------------------+
|                                SIMULATED HUMANOID ROBOT IN ISAAC SIM                                            |
|                                (Joints, Sensors: Camera, IMU, LiDAR)                                            |
+------------------------------------------------------------------------------------------------------------------+
```

## 2. Setting up the Simulated Environment in Isaac Sim

You will use Isaac Sim as your primary simulation environment.

### A. Humanoid Robot Model in Isaac Sim

Ensure your Isaac Sim stage contains your humanoid robot model. This model should be:
*   Articulated and configured with `ros2_control` interfaces.
*   Equipped with a stereo camera (for VSLAM and object detection) and an IMU.
*   Positioned in a simple environment (e.g., a room with a table, a shelf, and some known objects).

### B. Example Scene Configuration (`capstone_scene.py` in Isaac Sim)

This script would be an extension of `humanoid_vslam_sim.py` from the "Isaac VSLAM + Navigation Pipeline" mini-project.

```python
# humanoid_vla/scripts/capstone_scene.py (run within Isaac Sim)
import os
import carb
from omni.isaac.kit import SimulationApp

# Configuration as before, ensure ROS Bridge is active
CONFIG = {
    "headless": False,
    "width": 1280,
    "height": 720,
    "renderer": "RayTracing",
    "experience": f"{os.environ['ISAAC_ROS_PATH']}/_isaac_ros_common/exts/omni.isaac.ros_bridge/omni.isaac.ros_bridge.python.kit",
}
simulation_app = SimulationApp(CONFIG)

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.sensor import Camera, IMUSensor
from omni.isaac.ros_bridge.ros_bridge_extension import RosBridgeExtension
from pxr import Gf, UsdLux, UsdGeom

class CapstoneSim:
    def __init__(self):
        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            carb.log_error("Nucleus server not configured.")
            simulation_app.close()
            exit()

        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        # Add light
        UsdLux.DistantLight.Define(self.world.stage, "/World/distantLight")
        light_prim = self.world.stage.GetPrimAtPath("/World/distantLight")
        light_prim.GetAttribute("intensity").Set(20000.0)

        # Load your humanoid robot model (replace with your actual USD/URDF)
        # For this capstone, assume a more advanced humanoid model.
        # Placeholder using a simple cuboid for robot base and sensors
        humanoid_prim_path = "/World/humanoid"
        self.humanoid = self.world.scene.add(
            Articulation(
                prim_path=humanoid_prim_path,
                name="humanoid",
                # This should be your full humanoid USD or URDF converted to USD
                usd_path=self.assets_root_path + "/Isaac/Robots/Franka/franka_alt_fingers.usd", # Placeholder
                position=Gf.Vec3d(0.0, 0.0, 0.9),
            )
        )
        self.humanoid.disable_gravity() # For initial setup stability
        
        # Add a stereo camera and IMU to the humanoid (similar to VSLAM project)
        # Assuming '/World/humanoid/head_link' or similar exists for attachment
        self.stereo_camera = Camera(prim_path=humanoid_prim_path + "/head_link/stereo_camera", ...)
        self.imu_sensor = IMUSensor(prim_path=humanoid_prim_path + "/base_link/imu_sensor", ...)

        # Define and load known objects (e.g., "green sphere", "red cube")
        # Ensure these are distinct colors/shapes for easy detection
        self.green_sphere = self.world.scene.add_default_sphere(
            prim_path="/World/green_sphere",
            name="green_sphere",
            position=Gf.Vec3d(1.0, 0.5, 0.1),
            scale=Gf.Vec3d(0.1, 0.1, 0.1),
            color=Gf.Vec3f(0.0, 1.0, 0.0)
        )
        self.red_cube = self.world.scene.add_default_cube(
            prim_path="/World/red_cube",
            name="red_cube",
            position=Gf.Vec3d(1.0, -0.5, 0.1),
            scale=Gf.Vec3d(0.1, 0.1, 0.1),
            color=Gf.Vec3f(1.0, 0.0, 0.0)
        )
        # Add a table or shelf
        self.table = self.world.scene.add_default_cube(
            prim_path="/World/table",
            name="table",
            position=Gf.Vec3d(1.0, 0.0, 0.0),
            scale=Gf.Vec3d(0.8, 1.5, 0.02),
            color=Gf.Vec3f(0.5, 0.5, 0.5)
        )
        self.table.set_attribute("collisionEnabled", True) # Ensure collision for table

        self.world.reset()
        self.humanoid.enable_gravity() # Re-enable gravity after reset

    def run(self):
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.play()
        while simulation_app.is_running():
            self.world.step(render=True)
        simulation_app.close()

if __name__ == "__main__":
    capstone_sim = CapstoneSim()
    capstone_sim.run()
```

## 3. ROS 2 Node Implementation (`humanoid_vla` Package)

We will integrate all the ROS 2 nodes developed and conceptualized across the modules.

### A. `whisper_asr_node.py` (from Module 4, Chapter 1)

*   **Role:** Converts audio from a microphone (simulated or real) into text commands.
*   **Input:** Raw audio data (e.g., from `audio_common_msgs/msg/AudioData`).
*   **Output:** Transcribed text (`std_msgs/msg/String`) on `/humanoid/voice_commands`.

### B. `robot_state_provider_node.py` (from Module 4, Chapter 2)

*   **Role:** Aggregates current robot state (location, detected objects, held object) and publishes it as a JSON string for the LLM.
*   **Input:** Subscribes to `/tf` for robot pose, `/humanoid/object_detections` for vision, etc.
*   **Output:** JSON string (`std_msgs/msg/String`) on `/humanoid/robot_state`.

### C. `llm_planner_node.py` (from Module 4, Chapter 2)

*   **Role:** Takes voice commands and robot state, prompts an LLM, and generates a structured plan.
*   **Input:** `/humanoid/voice_commands`, `/humanoid/robot_state`.
*   **Output:** JSON plan (`std_msgs/msg/String`) on `/humanoid/generated_plan`.

### D. `vla_executive_node.py` (from Module 4, Chapter 3)

This node will implement the VLA Control Graph logic. It needs to contain ROS 2 clients for navigation, perception, and manipulation.

```python
# humanoid_vla/humanoid_vla/vla_executive_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

# Import ROS 2 Action/Service clients for robot capabilities
from nav2_msgs.action import NavigateToPose # For navigation
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

# Placeholder for perception service/action
# from humanoid_msgs.srv import DetectObject # Custom service

class VLAExecutiveNode(Node):
    def __init__(self):
        super().__init__('vla_executive_node')
        self.plan_subscriber = self.create_subscription(String, '/humanoid/generated_plan', self.plan_callback, 10)
        self.robot_status_publisher = self.create_publisher(String, '/humanoid/status_feedback', 10)

        # Action clients for robot capabilities
        self._nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # Placeholder for other action/service clients (e.g., manipulation, object detection)
        # self._detect_object_client = self.create_client(DetectObject, '/humanoid/detect_object')

        self.get_logger().info("VLA Executive Node started, waiting for plans.")

    def plan_callback(self, msg: String):
        self.get_logger().info(f"Received plan: {msg.data}")
        try:
            plan = json.loads(msg.data)
            self._execute_plan(plan)
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse plan JSON: {msg.data}")
            self._publish_status("Error: Invalid plan format.")
        
    def _execute_plan(self, plan: list):
        for action_item in plan:
            action_name = action_item["action"]
            parameters = action_item["parameters"]
            
            self.get_logger().info(f"Executing action: {action_name} with params: {parameters}")
            self._publish_status(f"Executing: {action_name}")

            if action_name == "navigate_to":
                self._navigate_to(parameters["location"])
            elif action_name == "detect_object":
                self._detect_object(parameters["object_name"])
            elif action_name == "pick_up":
                self._pick_up(parameters["object_name"], parameters["object_pose"])
            elif action_name == "place_object":
                self._place_object(parameters["location"])
            elif action_name == "report_status":
                self._report_status(parameters["message"])
            else:
                self.get_logger().warn(f"Unknown action: {action_name}")
                self._publish_status(f"Error: Unknown action {action_name}")
                break # Stop plan execution on unknown action
            
            # Simple delay for simulation (replace with actual action completion checks)
            time.sleep(2.0)

        self.get_logger().info("Plan execution finished.")
        self._publish_status("Plan completed.")

    def _navigate_to(self, location: str):
        self.get_logger().info(f"Navigating to {location}...")
        # In a real scenario, this would involve sending an action goal to Nav2
        # Example:
        # goal_msg = NavigateToPose.Goal()
        # goal_msg.pose = self._get_pose_for_location(location) # Lookup pose from map
        # self._nav_to_pose_client.wait_for_server()
        # self._send_goal_future = self._nav_to_pose_client.send_goal_async(goal_msg)
        # rclpy.spin_until_future_complete(self, self._send_goal_future)
        # goal_handle = self._send_goal_future.result()
        # if goal_handle.accepted:
        #     self._get_result_future = goal_handle.get_result_async()
        #     rclpy.spin_until_future_complete(self, self._get_result_future)
        #     if self._get_result_future.result().status == GoalStatus.STATUS_SUCCEEDED:
        #         self.get_logger().info(f"Successfully navigated to {location}")
        #     else:
        #         self.get_logger().error(f"Navigation to {location} failed")
        # else:
        #     self.get_logger().error(f"Navigation goal to {location} rejected")
        self._publish_status(f"Navigated to {location}.")

    def _detect_object(self, object_name: str):
        self.get_logger().info(f"Detecting object: {object_name}...")
        # In a real scenario, this would involve calling a perception service/action
        # E.g., self._detect_object_client.call_async(DetectObject.Request(object_name=object_name))
        self._publish_status(f"Detected {object_name}.")
        # Update robot state with detected object's pose if successful

    def _pick_up(self, object_name: str, object_pose: dict):
        self.get_logger().info(f"Picking up {object_name} at {object_pose}...")
        # This would involve complex manipulation actions (IK, motion planning, gripper control)
        self._publish_status(f"Picked up {object_name}.")

    def _place_object(self, location: str):
        self.get_logger().info(f"Placing object at {location}...")
        # This would involve manipulation and possibly navigation to a placement pose
        self._publish_status(f"Placed object at {location}.")

    def _report_status(self, message: str):
        self.get_logger().info(f"Reporting status: {message}")
        # This would involve a text-to-speech system
        self._publish_status(message)

    def _publish_status(self, message: str):
        msg = String()
        msg.data = message
        self.robot_status_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    vla_executive_node = VLAExecutiveNode()
    rclpy.spin(vla_executive_node)
    vla_executive_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*   **Dependencies:** `rclpy`, `std_msgs`, `json`, `nav2_msgs` (for action client).
*   **`setup.py`:** Add `vla_executive_node` as a console script entry point.

### E. `text_to_speech_node.py` (for Vocal Feedback)

*   **Role:** Converts text status messages into spoken audio for the human operator.
*   **Input:** `std_msgs/msg/String` on `/humanoid/status_feedback`.
*   **Output:** Plays audio (e.g., using `gTTS` or a local text-to-speech engine).

## 4. Launching the Capstone Project

The capstone project will be launched via a comprehensive ROS 2 launch file.

### A. `humanoid_vla_capstone.launch.py`

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    humanoid_vla_package_path = get_package_share_directory('humanoid_vla')
    humanoid_sim_package_path = get_package_share_directory('humanoid_sim')

    # 1. Launch Isaac Sim with the humanoid and environment
    # This assumes 'capstone_scene.py' handles the Isaac Sim launch internally or is run separately
    # For a unified launch, you might adapt the Isaac Sim script to be launched via ROS 2
    # Alternatively, ensure Isaac Sim is running the capstone_scene.py script before this launch.

    # 2. Launch Isaac ROS VSLAM pipeline (from Module 3)
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            humanoid_sim_package_path, 'launch', 'humanoid_vslam.launch.py' # Assuming you've adapted this
        )])
    )

    # 3. Launch Audio Capture (from Module 4, Chapter 1)
    audio_capture_node = Node(
        package='audio_capture',
        executable='audio_capture_node',
        name='audio_capture_node',
        parameters=[
            {'audio_device_name': 'hw:0,0'}, # Adjust for your microphone
            {'format': 'S16LE'},
            {'channels': 1},
            {'sample_rate': 16000}
        ],
        remappings=[
            ('audio', '/audio/audio')
        ],
        output='screen'
    )

    # 4. Launch Whisper ASR Node
    whisper_asr_node = Node(
        package='humanoid_vla',
        executable='whisper_asr_node',
        name='whisper_asr_node',
        parameters=[
            {'whisper_model_size': 'base'}, # Or 'small', 'medium', etc.
            {'audio_topic': '/audio/audio'},
            {'command_topic': '/humanoid/voice_commands'}
        ],
        output='screen'
    )

    # 5. Launch Robot State Provider Node
    robot_state_provider_node = Node(
        package='humanoid_vla',
        executable='robot_state_provider_node',
        name='robot_state_provider_node',
        output='screen'
    )

    # 6. Launch LLM Planner Node
    llm_planner_node = Node(
        package='humanoid_vla',
        executable='llm_planner_node',
        name='llm_planner_node',
        parameters=[
            {'llm_api_key': os.environ.get('OPENAI_API_KEY', 'YOUR_API_KEY')}, # Use env var for key
            {'llm_model': 'gpt-3.5-turbo'},
            {'voice_command_topic': '/humanoid/voice_commands'},
            {'robot_state_topic': '/humanoid/robot_state'}
        ],
        output='screen'
    )

    # 7. Launch VLA Executive Node
    vla_executive_node = Node(
        package='humanoid_vla',
        executable='vla_executive_node',
        name='vla_executive_node',
        output='screen'
    )

    # 8. Launch Text-to-Speech Node
    text_to_speech_node = Node(
        package='humanoid_vla',
        executable='text_to_speech_node',
        name='text_to_speech_node',
        parameters=[
            {'status_topic': '/humanoid/status_feedback'}
        ],
        output='screen'
    )

    return LaunchDescription([
        # Isaac Sim needs to be launched first, potentially as a separate process
        # For a fully integrated launch, you'd have an Isaac Sim ROS 2 launch wrapper here
        # E.g., IncludeLaunchDescription(PythonLaunchDescriptionSource(isaac_sim_launch_file))

        vslam_launch,
        audio_capture_node,
        whisper_asr_node,
        robot_state_provider_node,
        llm_planner_node,
        vla_executive_node,
        text_to_speech_node,
    ])
```

## 5. Running the Capstone Project

1.  **Start Isaac Sim:** Launch your `capstone_scene.py` script (either in GUI or headless mode).
    ```bash
    # For headless, adjust path to your Isaac Sim installation
    /path/to/isaac-sim/python.sh humanoid_vla/scripts/capstone_scene.py
    ```
2.  **Source Workspace:** In a new terminal, source your ROS 2 and workspace setup.
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/humanoid_ws/install/setup.bash
    ```
3.  **Set OpenAI API Key:** (If using a cloud LLM)
    ```bash
    export OPENAI_API_KEY="YOUR_API_KEY"
    ```
4.  **Launch ROS 2 Nodes:**
    ```bash
    ros2 launch humanoid_vla humanoid_vla_capstone.launch.py
    ```
5.  **Give Voice Commands:** Speak a command into your microphone, such as "Robot, pick up the green sphere and place it on the red cube."

Observe the robot's behavior in Isaac Sim:
*   Whisper transcribes your command.
*   The LLM generates a plan.
*   The VLA Executive orchestrates navigation, perception, and manipulation.
*   The robot moves, detects, grasps, and places the objects.
*   The robot provides vocal feedback using Text-to-Speech.

## 6. Going Further & Debugging

*   **Implement Missing Action Clients:** For `_detect_object`, `_pick_up`, `_place_object`, and `_report_status` within `vla_executive_node.py`, replace placeholder logging with actual ROS 2 Action/Service client calls that interface with your humanoid's `ros2_control` system and perception modules (e.g., Isaac ROS for object detection).
*   **Error Handling:** Enhance the VLA Executive to handle failures from action servers (e.g., navigation failure, object not found). Implement retry mechanisms or re-planning.
*   **Safety Supervisor:** Integrate a safety node that monitors robot state and prevents potentially dangerous LLM-generated actions.
*   **Custom LLM:** Experiment with running a local LLM (e.g., Llama 2 with `ollama`) if latency or privacy are critical concerns.
*   **Real Hardware:** After successful simulation, adapt the control interfaces for real hardware. This is where Sim-to-Real transfer techniques become paramount.
*   **`rviz2` Visualization:** Use `rviz2` to visualize robot pose, detected objects, navigation paths, and force feedback for debugging.

This capstone project provides a foundational blueprint for developing a truly intelligent and autonomous humanoid robot that interacts with its environment and human operators using the power of Physical AI.