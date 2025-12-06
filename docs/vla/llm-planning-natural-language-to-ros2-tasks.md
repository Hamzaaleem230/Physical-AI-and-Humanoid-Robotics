---
id: llm-planning-natural-language-to-ros2-tasks
title: LLM Planning: Natural Language → ROS 2 Tasks
slug: /modules/vla/llm-planning-natural-language-to-ros2-tasks
---

# LLM Planning: Natural Language → ROS 2 Tasks

The ability of Large Language Models (LLMs) to understand, generate, and reason with human language has revolutionized various fields of artificial intelligence. In robotics, LLMs represent a paradigm shift, enabling robots to interpret high-level, abstract natural language commands and translate them into concrete, executable sequences of actions. This bridges the significant gap between human intent and robot capabilities, moving away from rigid, pre-programmed behaviors towards more flexible and intuitive control.

This chapter explores how LLMs can be utilized for **high-level planning** in humanoid robots, specifically focusing on the translation of natural language instructions into actionable ROS 2 tasks. This is a crucial component of Vision-Language-Action (VLA) systems, allowing our humanoid to understand and execute complex requests.

## 1. The Challenge of Natural Language Instruction in Robotics

Traditionally, instructing robots involved:
*   **Teleoperation:** Directly controlling the robot (e.g., joystick, VR).
*   **Graphical User Interfaces (GUIs):** Clicking buttons, dragging and dropping icons.
*   **Explicit Programming:** Writing code for every single behavior.
*   **Pre-defined Commands:** A limited set of voice commands or gestures mapped to specific actions.

These methods lack flexibility. Human instructions are often:
*   **Ambiguous:** "Go over there" requires contextual understanding.
*   **Abstract:** "Make coffee" involves many sub-tasks.
*   **Implicit:** Relying on common sense.
*   **Open-ended:** Not all possibilities can be hard-coded.

LLMs offer a powerful solution by tapping into their vast knowledge base of language and, implicitly, the world.

## 2. LLMs as High-Level Planners

LLMs excel at reasoning over text and generating coherent sequences based on prompts. This capability can be harnessed to convert high-level natural language goals into a structured plan of robotic actions.

### A. The LLM Planning Workflow

1.  **Natural Language Goal:** The human provides a goal (e.g., "Robot, please bring me the red mug from the kitchen table").
2.  **LLM Prompt Engineering:** The raw natural language is combined with a structured prompt that guides the LLM. This prompt often includes:
    *   **Task Description:** The user's goal.
    *   **Context:** Current robot state (location, battery, what it sees).
    *   **Available Tools/APIs:** A list of executable robot functions (e.g., `navigate_to(location)`, `detect_object(object_name)`, `pick_up(object_name)`, `report_status(message)`).
    *   **Output Format:** Instructions for the LLM to output its plan in a specific, machine-readable format (e.g., a JSON list of actions).
3.  **LLM Inference:** The LLM processes the prompt and generates a response.
4.  **Plan Parsing & Execution:** The generated plan (e.g., a JSON list of function calls) is parsed by a ROS 2 node, which then executes the corresponding low-level robot actions.

**Verbal Diagram: LLM Planning Pipeline (Conceptual)**
```
+-----------------------------------------------------------+
|                   Human User (Voice Command)              |
|              "Robot, bring me the coffee."                |
+-----------------------------------------------------------+
          |
          v (Chapter: Whisper Voice Commands Integration)
+-----------------------------------------------------------+
|                  Whisper ASR Node                         |
|      (Converts audio to text: "bring me the coffee")      |
+-----------------------------------------------------------+
          | ROS 2 Topic: /humanoid/voice_commands (String)
          v
+-----------------------------------------------------------+
|                  LLM Planning Node (ROS 2)                |
|  +-----------------------------------------------------+  |
|  |             Prompt Engineering                      |  |
|  |             - Combine command, robot state,         |  |
|  |               available actions                     |  |
|  +-----------------------------------------------------+  |
|                            |                              |
|                            v (API Call or Local Inference)
|  +-----------------------------------------------------+  |
|  |             Large Language Model (LLM)              |  |
|  |             (e.g., GPT-4, Llama 2, Mistral)         |  |
|  +-----------------------------------------------------+  |
|                            |                              |
|                            v (Structured Plan - JSON)     |
|  +-----------------------------------------------------+  |
|  |             Plan Parser & Executor                  |  |
|  |             - Validates plan                        |  |
|  |             - Maps to ROS 2 actions/services        |  |
|  +-----------------------------------------------------+  |
|                            |                              |
|                            v (ROS 2 Calls: /navigate_to, /pick_up, etc.)
+-----------------------------------------------------------+
|                  Low-Level Robot Control (ROS 2)          |
+-----------------------------------------------------------+
```

## 3. Designing Effective LLM Prompts for Robotics

The quality of the LLM's plan heavily depends on the prompt. This is a form of **Prompt Engineering**.

### A. Core Elements of a Robotic Planning Prompt

1.  **System Role/Identity:** Define the LLM's persona (e.g., "You are a helpful robot assistant. Your task is to generate a sequence of actions for the robot to achieve a user's goal.").
2.  **Available Tools/APIs:** Clearly list the functions the robot can execute, their parameters, and what they do.
    *   `navigate_to(location: string)`: Moves robot to a known location (e.g., "kitchen", "office").
    *   `detect_object(object_name: string)`: Locates a specific object and returns its pose.
    *   `pick_up(object_name: string, object_pose: dict)`: Grasps an object.
    *   `place_object(location: string)`: Places a held object at a location.
    *   `report_status(message: string)`: Speaks a message to the human.
3.  **Current Robot State:** Provide the LLM with relevant context.
    *   Current location: `robot_location: "living_room"`
    *   Objects in view: `objects_in_view: ["red_mug", "blue_box"]`
    *   Held object: `held_object: "None"`
4.  **User Goal:** The transcribed natural language command.
5.  **Output Format Specification:** Crucially, instruct the LLM on the exact format for its response (e.g., a JSON array of actions).

### B. Example Prompt Structure

```text
You are a helpful robot assistant. Your task is to break down a user's high-level goal into a sequence of executable robot actions.
The robot has the following available functions:
- navigate_to(location: str): Navigates the robot to a predefined location. Known locations: ['kitchen', 'living_room', 'office']
- detect_object(object_name: str) -> dict: Detects a specific object in the current field of view and returns its pose. Returns {'object_name': str, 'pose': dict} or {} if not found.
- pick_up(object_name: str, object_pose: dict): Grasps the specified object using its pose.
- place_object(location: str): Places the currently held object at the specified location.
- report_status(message: str): Speaks a status message to the user.

Current Robot State:
robot_location: "living_room"
objects_in_view: ["red_mug", "blue_box"]
held_object: "None"

User Goal: "Please bring me the red mug from the kitchen table."

Respond with a JSON array of actions. If a function requires parameters, include them.
Example format:
[
  {"action": "navigate_to", "parameters": {"location": "kitchen"}},
  {"action": "detect_object", "parameters": {"object_name": "red_mug"}},
  {"action": "pick_up", "parameters": {"object_name": "red_mug", "object_pose": {"x": 0.5, "y": 0.2, "z": 0.8}}},
  {"action": "navigate_to", "parameters": {"location": "living_room"}},
  {"action": "place_object", "parameters": {"location": "living_room"}},
  {"action": "report_status", "parameters": {"message": "Here is your red mug."}}
]

If the task cannot be completed, respond with:
[{"action": "report_status", "parameters": {"message": "I cannot complete that task."}}]
```

## 4. Building the LLM Planning Node in ROS 2

A ROS 2 node is needed to orchestrate this planning process.

### A. LLM Planning Node Architecture

```python
# humanoid_vla/humanoid_vla/llm_planner_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import requests # For OpenAI API call or local LLM API
import os

# Define available robot functions (these would map to actual ROS 2 services/actions)
# This list would dynamically update based on robot capabilities
ROBOT_TOOLS = [
    {
        "name": "navigate_to",
        "description": "Navigates the robot to a predefined location.",
        "parameters": {"type": "object", "properties": {"location": {"type": "string", "enum": ["kitchen", "living_room", "office"]}}}
    },
    {
        "name": "detect_object",
        "description": "Detects a specific object in the current field of view and returns its pose.",
        "parameters": {"type": "object", "properties": {"object_name": {"type": "string"}}}
    },
    {
        "name": "pick_up",
        "description": "Grasps the specified object using its pose.",
        "parameters": {"type": "object", "properties": {"object_name": {"type": "string"}, "object_pose": {"type": "object"}}}
    },
    {
        "name": "place_object",
        "description": "Places the currently held object at the specified location.",
        "parameters": {"type": "object", "properties": {"location": {"type": "string", "enum": ["kitchen", "living_room", "office"]}}}
    },
    {
        "name": "report_status",
        "description": "Speaks a status message to the user.",
        "parameters": {"type": "object", "properties": {"message": {"type": "string"}}}
    }
]

class LLMPlannerNode(Node):
    def __init__(self):
        super().__init__('llm_planner_node')
        self.declare_parameter('llm_api_key', 'YOUR_OPENAI_API_KEY')
        self.declare_parameter('llm_model', 'gpt-3.5-turbo')
        self.declare_parameter('voice_command_topic', '/humanoid/voice_commands')
        self.declare_parameter('robot_state_topic', '/humanoid/robot_state') # Assuming robot publishes its state

        self.llm_api_key = self.get_parameter('llm_api_key').get_parameter_value().string_value
        self.llm_model = self.get_parameter('llm_model').get_parameter_value().string_value
        
        self.voice_command_subscriber = self.create_subscription(
            String,
            self.get_parameter('voice_command_topic').get_parameter_value().string_value,
            self.voice_command_callback,
            10
        )
        self.robot_state_subscriber = self.create_subscription(
            String, # Assuming JSON string for simplicity
            self.get_parameter('robot_state_topic').get_parameter_value().string_value,
            self.robot_state_callback,
            10
        )
        self.plan_publisher = self.create_publisher(String, '/humanoid/generated_plan', 10) # Publishes JSON plan

        self.current_robot_state = {
            "robot_location": "unknown",
            "objects_in_view": [],
            "held_object": "None"
        }
        self.get_logger().info("LLM Planner Node started, waiting for voice commands.")

    def robot_state_callback(self, msg: String):
        """Updates the robot's current state from a ROS 2 topic."""
        try:
            self.current_robot_state = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to parse robot state JSON: {msg.data}")

    def voice_command_callback(self, msg: String):
        """Receives a voice command and uses LLM to generate a plan."""
        user_goal = msg.data
        self.get_logger().info(f"Received voice command: '{user_goal}'")
        
        # Construct the prompt for the LLM
        prompt_messages = [
            {"role": "system", "content": self._get_system_prompt()},
            {"role": "user", "content": user_goal}
        ]
        
        # Make API call to LLM
        try:
            headers = {
                "Authorization": f"Bearer {self.llm_api_key}",
                "Content-Type": "application/json"
            }
            payload = {
                "model": self.llm_model,
                "messages": prompt_messages,
                "tools": ROBOT_TOOLS, # Provide tool definitions
                "tool_choice": "auto", # Allow LLM to decide if it uses a tool
                "temperature": 0.0 # Make it deterministic for planning
            }
            
            response = requests.post("https://api.openai.com/v1/chat/completions", headers=headers, json=payload)
            response.raise_for_status() # Raise HTTPError for bad responses (4xx or 5xx)
            
            response_data = response.json()
            
            # Check for tool_calls in the response
            if "tool_calls" in response_data["choices"][0]["message"]:
                tool_calls = response_data["choices"][0]["message"]["tool_calls"]
                generated_plan = []
                for tool_call in tool_calls:
                    function_name = tool_call["function"]["name"]
                    function_args = json.loads(tool_call["function"]["arguments"])
                    generated_plan.append({"action": function_name, "parameters": function_args})
                
                plan_msg = String()
                plan_msg.data = json.dumps(generated_plan)
                self.plan_publisher.publish(plan_msg)
                self.get_logger().info(f"Published plan: {plan_msg.data}")
            else:
                self.get_logger().warn(f"LLM did not generate tool calls. Response: {response_data['choices'][0]['message']['content']}")
                # Handle cases where LLM responds with natural language instead of tools
                self._report_error_status(f"Could not plan for: '{user_goal}'. LLM response: {response_data['choices'][0]['message']['content']}")

        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"LLM API request failed: {e}")
            self._report_error_status(f"LLM planning service unavailable. Error: {e}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Failed to decode LLM response JSON.")
            self._report_error_status("LLM returned malformed JSON plan.")
        except Exception as e:
            self.get_logger().error(f"An unexpected error occurred: {e}")
            self._report_error_status(f"Internal planning error: {e}")

    def _get_system_prompt(self):
        """Constructs the system prompt for the LLM."""
        system_prompt = f"""
You are a helpful robot assistant. Your task is to generate a sequence of executable robot actions to achieve a user's goal.
You have access to the following functions:
{json.dumps(ROBOT_TOOLS, indent=2)}

Current Robot State:
{json.dumps(self.current_robot_state, indent=2)}

Respond ONLY with a JSON array of actions using the available functions. Each object in the array must have an "action" key with the function name and a "parameters" key with a dictionary of arguments.

Example format:
[
  {{"action": "navigate_to", "parameters": {{"location": "kitchen"}}}},
  {{"action": "detect_object", "parameters": {{"object_name": "red_mug"}}}},
  // ...
]

If the task cannot be completed with the available functions, respond with:
[{{"action": "report_status", "parameters": {{"message": "I cannot complete that task."}}}}]

Ensure object_pose is correctly obtained from detect_object before using in pick_up.
"""
        return system_prompt

    def _report_error_status(self, message: str):
        """Publishes an error status message."""
        error_plan = [{"action": "report_status", "parameters": {"message": message}}]
        plan_msg = String()
        plan_msg.data = json.dumps(error_plan)
        self.plan_publisher.publish(plan_msg)

def main(args=None):
    rclpy.init(args=args)
    llm_planner_node = LLMPlannerNode()
    rclpy.spin(llm_planner_node)
    llm_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### B. Robot State Provider Node (Conceptual)

To provide `self.current_robot_state` to the LLM Planner, we need a ROS 2 node that aggregates the robot's current state (position, objects in view, held objects, etc.) and publishes it to `/humanoid/robot_state` as a JSON string. This node would subscribe to various sensors and internal state topics.

```python
# humanoid_vla/humanoid_vla/robot_state_provider_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

# Placeholder for actual sensor/state subscribers
class RobotStateProvider(Node):
    def __init__(self):
        super().__init__('robot_state_provider')
        self.state_publisher = self.create_publisher(String, '/humanoid/robot_state', 10)
        self.timer = self.create_timer(1.0, self.publish_robot_state) # Update every second

        # Example internal state (would be updated by actual subscribers)
        self._current_location = "living_room"
        self._objects_in_view = ["red_mug", "blue_box"]
        self._held_object = "None"
        self._object_poses = {"red_mug": {"x": 0.5, "y": 0.2, "z": 0.8}} # Example pose

        self.get_logger().info("Robot State Provider node started.")

    def publish_robot_state(self):
        robot_state = {
            "robot_location": self._current_location,
            "objects_in_view": self._objects_in_view,
            "held_object": self._held_object,
            "object_poses": self._object_poses # Add object poses for detect_object return value
        }
        msg = String()
        msg.data = json.dumps(robot_state)
        self.state_publisher.publish(msg)
        self.get_logger().debug(f"Published robot state: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    rsp_node = RobotStateProvider()
    rclpy.spin(rsp_node)
    rsp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
*   **`setup.py`:** Add `llm_planner_node` and `robot_state_provider_node` as console script entry points in your `humanoid_vla` package.

## 5. Challenges and Considerations for LLM Planning

### A. Grounding and Hallucinations

*   **Grounding:** Ensuring the LLM's understanding of objects and actions aligns with the robot's actual capabilities and perception. An LLM might "hallucinate" an object that isn't present or an action the robot can't perform.
*   **Mitigation:** Provide precise tool definitions. Integrate perceptual feedback ("Is the object actually there?") and motion planning feedback ("Can I reach it?") into the LLM's planning loop.

### B. Timeliness and State Updates

*   The LLM's plan is based on a snapshot of the robot's state. If the environment changes significantly during execution, the plan might become invalid.
*   **Mitigation:** Implement re-planning capabilities. Re-query the LLM if an action fails or the environment state deviates too much.

### C. Safety and Constraints

*   LLMs are not inherently safe. They might suggest actions that are dangerous or violate physical constraints.
*   **Mitigation:** Implement a "safety supervisor" module that validates every action proposed by the LLM before execution. This could involve collision checking, reachability analysis, and adherence to predefined safety protocols.

### D. Computational Cost and Latency

*   Calling large, cloud-based LLMs can incur significant latency and cost.
*   **Mitigation:** Use smaller, fine-tuned local LLMs (if feasible for the task). Employ caching for repetitive plans. Optimize prompt length.

## 6. Case Study: Resolving Ambiguity with Human Feedback

**Scenario:** User says, "Robot, go to the table." But there are multiple tables (kitchen table, living room table).

**LLM Planning with Clarification Loop:**
1.  **Whisper:** Transcribes "Robot, go to the table."
2.  **LLM Planner:** Receives command.
3.  **LLM:** Given its current robot state (e.g., `robot_location: "living_room"`, `objects_in_view: ["living_room_table", "coffee_table"]`), it realizes "table" is ambiguous.
4.  **LLM (instructed):** Generates a plan to `report_status("Which table do you mean? The living room table or the coffee table?")`.
5.  **Robot:** Speaks the clarification.
6.  **Human:** Replies, "The coffee table."
7.  **Whisper & LLM Planner:** Process the clarification and re-plan.
8.  **LLM:** Now generates `navigate_to("coffee_table")`.

This shows how LLMs can facilitate natural, multi-turn dialogue to resolve ambiguity and refine goals, bringing robots closer to human-like understanding.

## Conclusion

Large Language Models are transforming robot planning by enabling intuitive natural language instruction. By carefully designing prompts that provide context and define executable tools, LLMs can translate abstract human goals into concrete sequences of ROS 2 tasks. While challenges related to grounding, safety, and latency remain, the integration of LLMs as high-level planners is a monumental step towards building humanoid robots that can truly understand and respond to human intent in complex, real-world environments.