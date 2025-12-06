---
id: synthetic-data-generation
title: Synthetic Data Generation for Vision Models
slug: /modules/isaac-brain/synthetic-data-generation
---

# Synthetic Data Generation for Vision Models

Vision is arguably the most critical sense for humanoid robots operating in complex environments. Tasks like object manipulation, navigation, human-robot interaction, and scene understanding all rely heavily on accurate and robust visual perception. Training the deep learning models that power these vision capabilities requires vast amounts of high-quality, diverse, and well-annotated data. However, acquiring and labeling real-world data for robotics can be incredibly challenging, time-consuming, expensive, and often dangerous. This is where **Synthetic Data Generation (SDG)** emerges as a game-changer, offering a scalable and cost-effective solution.

This chapter explores the principles and techniques of Synthetic Data Generation, with a focus on leveraging NVIDIA Isaac Sim's advanced capabilities to create training datasets for humanoid robot vision models.

## 1. The Data Bottleneck in Robotics Vision

Deep learning models, especially for computer vision, are notoriously data-hungry. Consider the requirements for training a robot to grasp a novel object:
*   **Object Recognition:** Images of the object from various angles, lighting conditions, and backgrounds.
*   **Pose Estimation:** Ground truth 6D pose (position and orientation) of the object relative to the camera.
*   **Segmentation:** Pixel-perfect masks distinguishing the object from its background.
*   **Anomaly Detection:** Examples of damaged or misplaced objects.

Collecting such diverse and precisely labeled data in the real world is extremely difficult:
*   **Manual Annotation:** Labor-intensive, prone to human error, and slow.
*   **Realism Constraints:** Hard to simulate diverse lighting, occlusions, and environmental factors.
*   **Safety:** Dangerous objects or scenarios cannot be easily replicated for data collection.
*   **Scalability:** Generating millions of diverse images is practically impossible with real sensors.

## 2. What is Synthetic Data Generation (SDG)?

Synthetic data is artificially manufactured data, often generated in a simulation environment, that serves as a substitute for or supplement to real-world data. For vision models, this means rendering images and corresponding ground-truth labels directly from a 3D environment.

### A. Advantages of SDG

| Feature             | Real Data Collection                                                      | Synthetic Data Generation (SDG)                                                                                                                                                                                                                                                                             |
| :------------------ | :------------------------------------------------------------------------ | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **Cost**            | High (labor for collection, annotation, equipment)                        | Low (after initial setup)                                                                                                                                                                                                                                                                   |
| **Speed**           | Slow (manual process)                                                     | Fast (automated, scalable)                                                                                                                                                                                                                                                                  |
| **Ground Truth**    | Requires manual, error-prone, or specialized sensor annotation            | Perfect, pixel-accurate, and intrinsic (e.g., 6D pose, instance segmentation, depth maps)                                                                                                                                                                                                   |
| **Diversity**       | Limited by available real-world scenarios                                 | Easily controllable; can vary lighting, textures, camera angles, object properties, occlusions, environments.                                                                                                                                                                                   |
| **Edge Cases**      | Difficult to intentionally capture rare events or failure modes           | Can intentionally generate data for specific edge cases, failure scenarios, or rare object configurations.                                                                                                                                                                                      |
| **Privacy/Security** | Potential privacy concerns with human data                                | None, as data is entirely artificial.                                                                                                                                                                                                                                                       |
| **Domain Gap**      | High realism                                                              | Presents a "domain gap" challenge (models trained on synthetic data may not perform as well on real data), necessitating techniques like domain randomization and adaptation.                                                                                                              |

## 3. Isaac Sim's Role in Synthetic Data Generation

NVIDIA Isaac Sim, built on the Omniverse platform, is explicitly designed to be a powerful SDG tool for robotics. It provides a suite of features that enable the automated generation of diverse and perfectly labeled datasets.

### A. Key SDG Features in Isaac Sim

*   **Photorealistic Rendering (RTX):** Leveraging NVIDIA's RTX technology, Isaac Sim can render images with physically accurate lighting, reflections, and shadows, which contributes to higher realism.
*   **Ground Truth Generation:** Isaac Sim can directly output various ground truth data types simultaneously with RGB images:
    *   **Semantic Segmentation:** Pixel-wise labels for different object classes (e.g., `robot`, `table`, `cup`).
    *   **Instance Segmentation:** Pixel-wise labels for individual instances of objects (e.g., `cup_1`, `cup_2`).
    *   **Depth Maps:** Pixel-wise distance from the camera.
    *   **Bounding Boxes (2D & 3D):** Coordinates of bounding boxes around objects.
    *   **6D Object Poses:** Position and orientation of objects relative to the camera or world frame.
    *   **Optical Flow:** Pixel motion between frames.
*   **Domain Randomization (DR):** Isaac Sim provides extensive APIs to randomize various aspects of the simulation dynamically during data generation. This is crucial for bridging the domain gap.
    *   **Lighting:** Changing light intensity, color, position, and type.
    *   **Materials & Textures:** Randomizing visual properties of objects and surfaces.
    *   **Object Placement & Scale:** Randomizing the position, orientation, and size of objects within a scene.
    *   **Camera Parameters:** Randomizing camera intrinsic and extrinsic parameters.
    *   **Physics Properties:** Randomizing friction, mass, and damping of objects.
*   **Scripting and Automation:** Isaac Sim can be fully scripted using Python APIs, allowing for automated scene generation, object manipulation, and data capture without manual intervention.
*   **Multi-GPU Scalability:** Can leverage multiple GPUs to render data in parallel, drastically accelerating the data generation process.

## 4. Implementing Synthetic Data Generation in Isaac Sim

The typical workflow involves setting up a scene, defining randomization parameters, and then scripting the data capture.

### A. Setting up the Scene

1.  **Import Assets:** Load your humanoid robot model, target objects (e.g., tools, household items), and the environment into Isaac Sim.
2.  **Define Sensors:** Attach simulated cameras (RGB, Depth) to your robot's head or hands, configuring their intrinsic properties.
3.  **Ground Truth Sensors:** Enable the necessary ground truth sensors on your cameras to capture segmentation, depth, and pose data.

### B. Python Scripting for Data Generation

Isaac Sim's Python API allows you to control every aspect of the simulation programmatically.

```python
# Minimal example of setting up a scene and capturing data in Isaac Sim
import omni.usd
import omni.timeline
import omni.syntheticdata as sd
from pxr import Gf, UsdLux, UsdGeom, UsdPhysics

# Initialize Isaac Sim (this is typically done by the app startup)
# For scripting, assume Isaac Sim is already running or use the headless mode

class SyntheticDataGenerator:
    def __init__(self):
        self._stage = omni.usd.get_context().get_stage()
        self._timeline = omni.timeline.get_timeline_interface()

        # Create a simple scene (ground plane, light)
        self.create_scene()

        # Load your humanoid robot and target objects (e.g., from USD files)
        self.load_robot_and_objects()

        # Add a camera and ground truth sensors to the robot
        self.add_camera_and_sd_sensors()

        # Configure domain randomization (e.g., randomizing light intensity)
        self.setup_domain_randomization()

    def create_scene(self):
        # Add a default ground plane
        UsdGeom.Xform.Define(self._stage, "/World").GetPrim()
        UsdGeom.SetStageUpAxis(self._stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self._stage, 1.0) # 1 meter per unit

        UsdLux.DistantLight.Define(self._stage, "/World/distantLight")
        light_prim = self._stage.GetPrimAtPath("/World/distantLight")
        light_prim.GetAttribute("intensity").Set(20000.0)
        light_prim.GetAttribute("angle").Set(0.5)

        UsdGeom.Mesh.Define(self._stage, "/World/GroundPlane")
        ground_plane_prim = self._stage.GetPrimAtPath("/World/GroundPlane")
        UsdGeom.Gf.Scale(100.0).Set(ground_plane_prim.GetAttribute("extent")) # Scale to 100x100m
        UsdPhysics.CollisionAPI.Apply(ground_plane_prim)


    def load_robot_and_objects(self):
        # Example: Loading a simple cube as an object to interact with
        cube_path = "/World/Cube"
        sd.create_prim_with_model(
            prim_path=cube_path,
            usd_path=sd.create_prim_path_to_model(self._stage, "Cube"), # Built-in asset
            scale=Gf.Vec3d(0.1, 0.1, 0.1)
        )
        # Randomize cube's position (simple example)
        sd.get_prim_at_path(cube_path).get_attribute("xformOp:translate").Set(Gf.Vec3d(0.5, 0, 0.05))

        # TODO: Load your humanoid robot here (e.g., via URDF importer or existing USD)
        # e.g., sd.create_prim_with_model(prim_path="/World/Humanoid", usd_path="path/to/humanoid.usd")


    def add_camera_and_sd_sensors(self):
        # Create a camera (e.g., attached to the humanoid's head)
        camera_path = "/World/Camera" # Attach to humanoid's head later
        sd.syntheticdata.add_sensor(
            camera_path,
            sensor_type="Camera",
            resolution=(640, 480),
            shutter_open="0.0",
            shutter_close="0.0"
        )
        # Add ground truth sensors to the camera
        sd.syntheticdata.enable_sensors(
            camera_path,
            [
                sd.SensorType.RGB,
                sd.SensorType.Depth,
                sd.SensorType.InstanceSegmentation,
                sd.SensorType.BoundingBox3D,
                sd.SensorType.Pose # Object pose
            ]
        )

    def setup_domain_randomization(self):
        # Example: Randomize light intensity
        light_prim = self._stage.GetPrimAtPath("/World/distantLight")
        # Define a randomization rule for light intensity
        sd.create_attribute_randomizer(
            prim_path=light_prim.GetPath(),
            attribute_name="intensity",
            range=[10000.0, 30000.0], # Min and max intensity
            distribution="uniform"
        )
        # TODO: Add more randomizers for textures, object poses, etc.

    def generate_data(self, num_frames=1000):
        self._timeline.play() # Start simulation
        for i in range(num_frames):
            # Advance simulation (e.g., robot performs an action, objects move)
            # You would control your robot here, or let physics run
            self._timeline.get_current_time() # Advance one frame
            
            # Capture data
            sd.syntheticdata.render_and_write(f"output/data_frame_{i}")
            
        self._timeline.stop() # Stop simulation


if __name__ == "__main__":
    # This script is meant to be run within Isaac Sim's Python environment
    # or by launching Isaac Sim with this script.
    # For a full application, you would wrap this in an Isaac Sim extension.
    generator = SyntheticDataGenerator()
    generator.generate_data(num_frames=10) # Generate 10 frames of data
    print("Synthetic data generation complete!")
```
This conceptual example demonstrates the power of the Python API to programmatically control scene elements, randomization, and data capture.

## 5. Domain Randomization: Bridging the Reality Gap

The primary challenge with synthetic data is the "domain gap"—the discrepancy between synthetic and real-world data distributions. Models trained purely on synthetic data might perform poorly on real sensor inputs. **Domain Randomization (DR)** is a powerful technique to mitigate this.

### A. Principle of Domain Randomization

Instead of trying to perfectly match the real world in simulation, DR *randomizes* as many non-essential aspects of the simulation as possible during training. This forces the neural network to learn to extract the truly invariant features of the task, making it robust to variations in appearance and physics.

**Figure 5.1: Domain Randomization Concept**
```
+-----------------------------------------------------------------+
|                         Simulation Environment                  |
|  +-----------------------------------------------------------+  |
|  |     Randomize:                                            |  |
|  |     - Light position, color, intensity                    |  |
|  |     - Object textures, colors, materials                  |  |
|  |     - Object positions, orientations, scales              |  |
|  |     - Camera intrinsic/extrinsic parameters               |  |
|  |     - Physics properties (friction, mass, restitution)    |  |
|  |     - Backgrounds, noise models                           |  |
|  +-----------------------------------------------------------+  |
|                                     |                             |
|                                     v                             |
|  +-----------------------------------------------------------+  |
|  |               Synthetic Data (Diverse & Labeled)          |  |
|  +-----------------------------------------------------------+  |
|                                     |                             |
|                                     v                             |
|  +-----------------------------------------------------------+  |
|  |              Train Deep Learning Model (e.g., Grasping)   |  |
|  +-----------------------------------------------------------+  |
|                                     |                             |
|                                     v                             |
|  +-----------------------------------------------------------+  |
|  |     Robust Model                                          |  |
|  |     (Generalizes better to unseen real-world variations)  |  |
|  +-----------------------------------------------------------+  |
+-----------------------------------------------------------------+
```

By presenting the model with an infinite variety of simulated scenarios, it learns to generalize better to the inherent variability of the real world.

### B. Isaac Sim's Domain Randomization API

Isaac Sim provides a dedicated `omni.syntheticdata.scripts.scripts.domain_randomization` module and specific APIs to set up randomizers. You can randomize attributes of prims (USD elements) like:
*   `UsdLux.Light` attributes (intensity, color).
*   `UsdShade.Material` properties (diffuse color, roughness).
*   `UsdGeom.XformOp` attributes (translation, rotation, scale).

## 6. Case Study: Training a Humanoid Hand for Grasping

**Scenario:** We want to train a deep learning model to enable a humanoid robot's hand to grasp a variety of household objects robustly.

**SDG with Isaac Sim Workflow:**

1.  **High-Fidelity Hand Model:** Import the humanoid robot's hand model (e.g., a highly articulated gripper) and the target objects (cups, bottles, tools) into Isaac Sim.
2.  **Scene Setup:** Create a scene with a table. Place the hand in an initial pose above the table.
3.  **Object Spawning & Randomization:**
    *   **Randomize object type:** Cycle through a library of target objects.
    *   **Randomize object pose:** For each object, randomize its position and orientation on the table.
    *   **Randomize object properties:** Change the friction, mass, and texture of the objects.
    *   **Randomize lighting:** Change the scene's lighting conditions (intensity, color, direction).
    *   **Randomize background:** Change the textures of the table and walls.
4.  **Camera Setup:** Mount a simulated depth camera on the humanoid's wrist, pointing towards the gripper and target object.
5.  **Data Capture Loop:**
    *   In a Python script, for each training sample:
        *   Randomize scene parameters.
        *   Randomize object pose.
        *   Render an RGB image and capture ground truth depth, instance segmentation, and the 6D pose of the target object relative to the camera.
        *   Save this data to disk.
6.  **RL-Based Grasping (Optional but powerful):** Simultaneously, you could use this setup to train an RL policy for grasping, where the robot learns to manipulate its joints to pick up objects. The perfectly labeled ground truth can also be used as a reward signal or for supervised learning components.

By generating a massive dataset of millions of such randomized images and corresponding ground truth, the trained grasping model becomes highly robust and capable of generalizing to novel objects and real-world lighting conditions, significantly reducing the sim-to-real gap.

## Conclusion

Synthetic Data Generation in Isaac Sim is a powerful paradigm shift in robotics vision. It addresses the data bottleneck, enables the training of more robust and generalizable deep learning models through techniques like domain randomization, and accelerates the entire development cycle for AI-powered humanoid robots. Mastering SDG is an essential skill for any roboticist aiming to build intelligent systems that perform reliably in the unpredictable physical world.