# Isaac VSLAM + Navigation Pipeline Mini-Project

This mini-project guides you through setting up a Visual SLAM (VSLAM) and basic navigation pipeline for a humanoid robot using NVIDIA Isaac ROS within Isaac Sim. You will learn to leverage GPU-accelerated modules for real-time pose estimation and map generation, integrating these with a conceptual navigation stack.

## Project Goal

1.  Launch Isaac Sim with a simulated humanoid robot equipped with a stereo camera and IMU.
2.  Run the `isaac_ros_vslam` package to obtain accurate, real-time pose estimates and build a sparse map of the environment.
3.  Visualize the robot's trajectory and the VSLAM map in `rviz2`.
4.  (Conceptual) Lay the groundwork for integrating VSLAM output with a Nav2-like system for humanoid path planning.

## Prerequisites

*   A working ROS 2 Humble installation.
*   NVIDIA Isaac Sim installed and configured (as covered in previous chapters).
*   Your `humanoid_ws` workspace set up.
*   Familiarity with Isaac Sim's Python scripting environment.
*   Installation of Isaac ROS VSLAM packages.

## 1. Installing Isaac ROS VSLAM

Ensure you have the necessary Isaac ROS packages. You might need to add the NVIDIA ROS repository and then install.

```bash
# Add NVIDIA ROS repository (if not already added)
# sudo apt update && sudo apt install -y software-properties-common
# sudo add-apt-repository universe
# sudo apt update && sudo apt install -y curl
# sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.d/ros2.list > /dev/null
# echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] https://packages.nvidia.com/isaac-ros/ubuntu/$(. /etc/os-release && echo $UBUNTU_CODENAME) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee -a /etc/apt/sources.list.d/isaac-ros-vslam.list
# sudo apt update

# Install Isaac ROS VSLAM and dependencies
sudo apt install ros-humble-isaac-ros-vslam ros-humble-isaac-ros-image-pipeline ros-humble-isaac-ros-common
```
Then build your workspace.

## 2. Setting up Isaac Sim for VSLAM

We need a simulated humanoid robot with a stereo camera and an IMU. We'll use the Isaac Sim Python API to configure the scene and sensors.

### A. Isaac Sim Python Script (`humanoid_vslam_sim.py`)

Create a script that launches Isaac Sim and configures your humanoid robot with necessary sensors. This script should be run *within* Isaac Sim's Python environment.

```python
# humanoid_sim/scripts/humanoid_vslam_sim.py
import os
import carb
from omni.isaac.kit import SimulationApp

# Initializing Isaac Sim application. Set 'headless=True' for no GUI.
CONFIG = {
    "headless": False,
    "width": 1280,
    "height": 720,
    "renderer": "RayTracing",
    "experience": f"{os.environ['ISAAC_ROS_PATH']}/_isaac_ros_common/exts/omni.isaac.ros_bridge/omni.isaac.ros_bridge.python.kit",
}
simulation_app = SimulationApp(CONFIG)

import omni.timeline
import omni.usd
from pxr import Gf, UsdLux, UsdGeom, UsdPhysics

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.sensor import Camera, IMUSensor # Isaac Sim sensor APIs
from omni.isaac.ros_bridge.ros_bridge_extension import RosBridgeExtension # To ensure ROS bridge is active

class HumanoidVSLAMSim:
    def __init__(self):
        self.assets_root_path = get_assets_root_path()
        if self.assets_root_path is None:
            carb.log_error("Could not find Isaac Sim assets folder. Please make sure your Nucleus server is configured.")
            simulation_app.close()
            exit()

        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        # Add a light
        UsdLux.DistantLight.Define(self.world.stage, "/World/distantLight")
        light_prim = self.world.stage.GetPrimAtPath("/World/distantLight")
        light_prim.GetAttribute("intensity").Set(20000.0)
        light_prim.GetAttribute("angle").Set(0.5)

        # Load a conceptual humanoid robot model (replace with your USD/URDF import)
        # For simplicity, let's load a standard UR10 robot as a placeholder for
        # an articulated robot that will move. In a real scenario, you'd load your
        # humanoid.
        robot_asset_path = self.assets_root_path + "/Isaac/Robots/UR10/ur10_ros_articulation.usd"
        # If your humanoid is in URDF, you would use:
        # from omni.isaac.urdf import _urdf
        # _urdf.add_urdf_to_stage(usd_path="/World/humanoid", urdf_path="path/to/your/humanoid.urdf", ... )

        self.humanoid = self.world.scene.add(
            Articulation(
                prim_path="/World/humanoid",
                name="humanoid",
                usd_path=robot_asset_path,
                position=Gf.Vec3d(0.0, 0.0, 0.9), # Adjust Z to stand on ground
            )
        )

        # Add Stereo Camera to the humanoid (e.g., on its head_link)
        self.stereo_camera = Camera(
            prim_path="/World/humanoid/tool0/stereo_camera", # Example attach point, adjust for your humanoid
            name="stereo_camera",
            position=Gf.Vec3d(0.0, 0.0, 0.0), # Relative to parent link
            orientation=Gf.Quatf(1, 0, 0, 0), # Identity rotation
            resolution=(640, 480),
            horizontal_fov=math.radians(90.0),
            near_plane=0.1,
            far_plane=1000.0,
            # Add ROS 2 topics for images
            ros_topic_name="stereo_camera",
            ros_bridge_setup=RosBridgeExtension.get_instance()._ros_bridge_setup_usd_prim_path # Required for ROS topics
        )
        # Configure for Stereo
        self.stereo_camera.set_enabled(True) # Left camera
        self.stereo_camera.create_other_camera_for_stereo_setup(
            horizontal_offset=0.06, # Baseline for stereo camera (e.g., 6cm)
            ros_topic_name="stereo_camera_right"
        )
        self.stereo_camera.set_active_camera_prim_path(self.stereo_camera.get_prim_path()) # Ensure left is active

        # Add IMU Sensor to the humanoid (e.g., on its base_link)
        self.imu_sensor = IMUSensor(
            prim_path="/World/humanoid/base_link/imu_sensor", # Example attach point
            name="imu_sensor",
            grav_vec=Gf.Vec3d(0.0, 0.0, -9.81),
            ros_topic_name="imu_data",
            ros_bridge_setup=RosBridgeExtension.get_instance()._ros_bridge_setup_usd_prim_path
        )
        self.imu_sensor.set_enabled(True)

        self.world.reset() # Reset the simulation world

    def run(self):
        self._timeline = omni.timeline.get_timeline_interface()
        self._timeline.play()

        # Run for a few minutes or until simulation is closed
        while simulation_app.is_running():
            self.world.step(render=True) # Advance simulation and render

        simulation_app.close()

if __name__ == "__main__":
    vslam_sim = HumanoidVSLAMSim()
    vslam_sim.run()

```

### B. Launching the Isaac Sim Script

You typically run this script by launching Isaac Sim and pointing it to your Python script, or by running Isaac Sim and then executing the script from its embedded Python environment.

```bash
# Example: Launch Isaac Sim in headless mode with your script (adjust path)
# This command assumes you have set up your Isaac Sim environment variables
./run_headless.sh /path/to/humanoid_sim/scripts/humanoid_vslam_sim.py
```
Or, launch Isaac Sim GUI and then:
```python
# From Isaac Sim's Python console
import sys
sys.path.append("/path/to/humanoid_sim/scripts/")
import humanoid_vslam_sim
vslam_sim = humanoid_vslam_sim.HumanoidVSLAMSim()
vslam_sim.run()
```

This will start Isaac Sim, load your humanoid, and publish stereo camera images and IMU data to ROS 2 topics (`/stereo_camera/left/image_raw`, `/stereo_camera/right/image_raw`, `/imu_data`).

## 3. Running Isaac ROS VSLAM

Once Isaac Sim is publishing sensor data to ROS 2, you can run the `isaac_ros_vslam` node in your ROS 2 workspace.

### A. VSLAM Launch File (`humanoid_sim/launch/humanoid_vslam.launch.py`)

```python
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to VSLAM launch file in isaac_ros_vslam package
    vslam_launch_dir = os.path.join(
        get_package_share_directory('isaac_ros_vslam'), 'launch')

    # Default VSLAM parameters (adjust for your specific camera/IMU)
    # These can also be loaded from a YAML file
    vslam_params = {
        'enable_image_denoising': False,
        'enable_imu_fusion': True, # Use IMU data
        'gyro_noise_density': 0.0002,
        'accel_noise_density': 0.003,
        'gyro_random_walk': 0.00002,
        'accel_random_walk': 0.0004,
        'camera_frame': 'stereo_camera_link', # The TF frame of your camera
        'odom_frame': 'odom',
        'map_frame': 'map',
        'base_frame': 'base_link', # Robot's base frame
        'publish_tf': True,
        'publish_map_tf': True,
        'pose_frame': 'odom',
        'enable_slam': True,
        'map_pub_period': 1.0, # Publish map updates every 1 second
        'image_topic': '/stereo_camera/left/image_raw', # Left camera image
        'camera_info_topic': '/stereo_camera/left/camera_info', # Left camera info
        'right_image_topic': '/stereo_camera/right/image_raw', # Right camera image
        'right_camera_info_topic': '/stereo_camera/right/camera_info', # Right camera info
        'imu_topic': '/imu_data', # IMU data topic
    }

    return LaunchDescription([
        Node(
            package='isaac_ros_vslam',
            executable='isaac_ros_vslam',
            name='vslam',
            namespace='humanoid', # Use a namespace to avoid conflicts
            parameters=[vslam_params],
            output='screen'
        ),
    ])
```

### B. Launching VSLAM

1.  **Start Isaac Sim:** Launch your `humanoid_vslam_sim.py` script.
2.  **Source ROS 2:** In a new terminal, source your ROS 2 and workspace setup.
3.  **Launch VSLAM:**
    ```bash
    ros2 launch humanoid_sim humanoid_vslam.launch.py
    ```

You should see VSLAM starting up, processing images and IMU data.

## 4. Visualizing VSLAM Output in `rviz2`

In another terminal, launch `rviz2`:

```bash
rviz2
```

In `rviz2`:
1.  **Set Fixed Frame:** To `map` (or `odom` initially).
2.  **Add `RobotModel`:** To see your humanoid.
3.  **Add `TF`:** To visualize coordinate frames and the `map->odom` and `odom->base_link` transforms published by VSLAM.
4.  **Add `Odometry`:** Subscribe to `/humanoid/vslam/odometry` to see the estimated robot trajectory.
5.  **Add `Map` (Point Cloud):** Subscribe to `/humanoid/vslam/map` to visualize the sparse map generated by VSLAM.
6.  **Add `Image` displays:** Subscribe to `/stereo_camera/left/image_raw` to see the camera feed.

As your humanoid moves in Isaac Sim, you should observe its estimated pose and the gradual construction of a sparse map in `rviz2`.

## 5. Conceptual Integration with Navigation (Nav2)

The VSLAM output (odometry and map data) serves as crucial input for a navigation stack like Nav2.

**Figure 5.1: VSLAM Output to Nav2 (Conceptual)**
```
+-----------------------------------------------------+
|                  Isaac Sim                          |
|  +-----------------------------------------------+  |
|  |     Simulated Humanoid with Stereo Camera/IMU   |  |
|  |     (Publishes /stereo_camera/*, /imu_data)     |  |
|  +-----------------------------------------------+  |
+-----------------------------------------------------+
                      | ROS 2 Topics
                      v
+-----------------------------------------------------+
|                  ROS 2 Workspace                    |
|  +-----------------------------------------------+  |
|  |          `isaac_ros_vslam` Node                 |  |
|  |    (Subscribes to sensor data)                  |  |
|  |    (Publishes /humanoid/vslam/odometry, /tf)    |  |
|  +-----------------------------------------------+  |
|                      |                                |
|                      v                                |
|  +-----------------------------------------------+  |
|  |          Nav2 Stack (Conceptual)                |  |
|  |    - `humanoid_localization` (subscribes to VSLAM odometry) |  |
|  |    - `humanoid_global_planner`                    |  |
|  |    - `humanoid_local_planner` (Footstep Planner)  |  |
|  +-----------------------------------------------+  |
+-----------------------------------------------------+
```

*   **Localization:** The `humanoid_localization` node (part of your custom Nav2 setup) would consume the `/humanoid/vslam/odometry` topic to enhance its pose estimation.
*   **Mapping:** While `isaac_ros_vslam` provides a sparse map, a more dense map for Nav2's costmaps might come from a separate mapping node that consumes depth data or the point cloud generated by VSLAM.
*   **Footstep Planning:** Your custom local planner would use the robot's pose from VSLAM and the environmental map to generate stable footstep plans.

## Troubleshooting

*   **VSLAM node not starting:** Check ROS 2 environment, package installation, and launch file syntax.
*   **No VSLAM output in `rviz2`:** Ensure `isaac_ros_vslam` is running, publishing topics, and `rviz2` is subscribed to the correct topics and has the `Fixed Frame` set appropriately.
*   **Isaac Sim not publishing ROS 2 topics:** Ensure `omni.isaac.ros_bridge` extension is enabled and configured correctly in your Isaac Sim script.
*   **Performance issues:** VSLAM is computationally intensive. Ensure you are running on an NVIDIA GPU (workstation or Jetson) and that Isaac Sim is running efficiently (e.g., in headless mode for data generation).

This mini-project demonstrates how to harness the power of Isaac ROS and Isaac Sim to build advanced perception pipelines, a critical step towards autonomous humanoid navigation.