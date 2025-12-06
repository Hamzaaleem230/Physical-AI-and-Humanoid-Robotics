---
id: building-ros2-packages
title: Building ROS 2 Packages with Python (rclpy)
slug: /modules/ros2-nervous-system/building-ros2-packages
---

# Building ROS 2 Packages with Python (rclpy)

In ROS 2, a **package** is the fundamental unit for organizing software. A package can contain ROS 2 nodes, libraries, configuration files, message definitions, and other resources. For our humanoid robotics projects, Python-based packages using `rclpy` (ROS Client Library for Python) will be our primary tool due to Python's expressiveness and its vast ecosystem for AI and machine learning.

This chapter will guide you through the process of creating, structuring, and building ROS 2 Python packages from scratch.

## 1. ROS 2 Workspace: The Development Environment

Before creating packages, you need a **ROS 2 Workspace**. A workspace is simply a directory where you collect your ROS 2 packages and where you can build them. It allows you to develop and manage multiple packages without interfering with your system's ROS 2 installation.

### A. Creating a Workspace

Let's create a typical ROS 2 workspace:

```bash
# Create a directory for your workspace
mkdir -p ~/humanoid_ws/src

# Navigate into the workspace directory
cd ~/humanoid_ws

# Initialize the workspace (optional, but good practice for empty workspaces)
# This creates a .rosinstall file and sets up the build environment
rosinstall_generator desktop --rosdistro humble --output-dir .
```

The `src` subdirectory is where your packages will reside. The `humanoid_ws` directory itself is the workspace root.

### B. Overlaying ROS 2 Environments

When you build and install packages in your workspace, they are "overlayed" on top of your base ROS 2 installation. This means that your workspace's installed packages will be found before the system-installed packages.

**Verbal Diagram: ROS 2 Environment Overlay**
```
+-------------------------------------------------------+
|              System Environment (Ubuntu/Debian)       |
|  +-------------------------------------------------+  |
|  |           ROS 2 Installation (e.g., /opt/ros/humble)  |  |
|  |   - Core ROS 2 packages                           |  |
|  |   - `rclpy`, `rclcpp`, `std_msgs`, `sensor_msgs`  |  |
|  |   - Built-in drivers and tools                    |  |
|  +-------------------------------------------------+  |
|                                                       |
|  +-------------------------------------------------+  |
|  |           ROS 2 Workspace (e.g., ~/humanoid_ws)   |  |
|  |   - Your custom packages (e.g., `humanoid_controller`)  |  |
|  |   - Packages downloaded from GitHub               |  |
|  +-------------------------------------------------+  |
+-------------------------------------------------------+
```

To use your workspace, you must **source** its setup file after sourcing your main ROS 2 installation:

```bash
# Source the main ROS 2 installation (if not already done in .bashrc)
source /opt/ros/humble/setup.bash

# Source your workspace (AFTER the main ROS 2 setup)
source ~/humanoid_ws/install/setup.bash
```

This ensures that your shell's environment variables (`ROS_PACKAGE_PATH`, `PYTHONPATH`, etc.) are correctly configured to find your custom packages.

## 2. Creating a Python Package

ROS 2 provides a convenient command-line tool, `ros2 pkg create`, to scaffold a new package.

### A. Package Creation Command

Let's create a package called `humanoid_controller` in our workspace:

```bash
cd ~/humanoid_ws/src

ros2 pkg create --build-type ament_python humanoid_controller --dependencies rclpy std_msgs sensor_msgs
```

**Explanation of arguments:**
*   `--build-type ament_python`: Specifies that this is a Python package using the `ament_python` build system.
*   `humanoid_controller`: The name of our new package.
*   `--dependencies rclpy std_msgs sensor_msgs`: Declares runtime dependencies. `rclpy` is always needed for Python ROS 2 nodes. `std_msgs` and `sensor_msgs` are common message types we'll likely use for basic control and sensor data.

After running this command, you will find a new directory `~/humanoid_ws/src/humanoid_controller` with the following basic structure:

```
humanoid_ws/src/
└── humanoid_controller/
    ├── package.xml
    ├── setup.py
    ├── setup.cfg
    ├── resource/
    │   └── humanoid_controller
    └── humanoid_controller/   <-- This is the actual Python module
        └── __init__.py
```

### B. Understanding Package Files

1.  **`package.xml`**:
    *   This is the manifest file for the package. It contains metadata like package name, version, description, maintainer, license, and **build and runtime dependencies**.
    *   **Crucial for dependencies:** When you add new external Python libraries or ROS 2 packages, you *must* declare them here.

    ```xml
    <?xml version="1.0"?>
    <?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
    <package format="3">
      <name>humanoid_controller</name>
      <version>0.0.0</version>
      <description>A basic ROS 2 controller for a humanoid robot.</description>
      <maintainer email="your_email@example.com">Your Name</maintainer>
      <license>Apache-2.0</license>

      <!-- Build tool dependency -->
      <buildtool_depend>ament_python</buildtool_depend>

      <!-- Runtime dependencies (ROS 2 packages) -->
      <depend>rclpy</depend>
      <depend>std_msgs</depend>
      <depend>sensor_msgs</depend>

      <!-- Test dependencies -->
      <test_depend>ament_copyright</test_depend>
      <test_depend>ament_flake8</test_depend>
      <test_depend>ament_pep257</test_depend>
      <test_depend>python3-pytest</test_depend>

      <export>
        <build_type>ament_python</build_type>
      </export>
    </package>
    ```

2.  **`setup.py`**:
    *   This file is used by Python's `setuptools` to build and install the package. It defines how Python modules and executables within your ROS 2 package are installed.
    *   **Crucial for executables:** If you want to run a Python script in your package as a ROS 2 executable (e.g., `ros2 run my_package my_script`), you must add an `entry_point` in `setup.py`.

    ```python
    from setuptools import find_packages, setup
    import os
    from glob import glob

    package_name = 'humanoid_controller'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
            # Install launch files
            (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yeml]'))),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='Your Name',
        maintainer_email='your_email@example.com',
        description='A basic ROS 2 controller for a humanoid robot.',
        license='Apache-2.0',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                # Example: 'my_node = humanoid_controller.my_node:main'
                'simple_publisher = humanoid_controller.simple_publisher:main',
                'simple_subscriber = humanoid_controller.simple_subscriber:main',
            ],
        },
    )
    ```
    *Note: The `data_files` section is also important for installing non-Python files like launch files or URDFs.*

3.  **`setup.cfg`**:
    *   Used for configuring `setuptools` build options, `flake8` (linting), `pytest`, etc. For ament_python packages, it often includes a `[options.entry_points]` section mirroring `setup.py`.

4.  **`resource/<package_name>`**:
    *   An empty file that acts as a marker for ROS 2 to find the package.

5.  **`humanoid_controller/` (Python module directory):**
    *   This is where your actual Python code (ROS 2 nodes, libraries) will live. It's a standard Python package, so it contains an `__init__.py` file.

## 3. Building and Installing Packages with `colcon`

`colcon` is the build tool used in ROS 2. It orchestrates the building, testing, and installing of multiple packages in a workspace.

### A. The `colcon build` Command

To build your `humanoid_controller` package, navigate to your workspace root and run:

```bash
cd ~/humanoid_ws
colcon build --packages-select humanoid_controller
```

*   `--packages-select humanoid_controller`: Tells `colcon` to only build our specific package, which is faster during development. If omitted, `colcon` builds all packages in `src/`.

Upon successful build, `colcon` creates several directories in your workspace root:
*   **`build/`**: Contains intermediate build files (e.g., compiled Python bytecode).
*   **`install/`**: Contains the final installed files (executables, libraries, Python modules, ROS 2 setup scripts).
*   **`log/`**: Contains build logs.

### B. Installing Dependencies

If your package depends on external Python libraries that are not ROS 2 packages (e.g., `numpy`, `scipy`), you should declare them in `setup.py`'s `install_requires` list. Then, install them into your ROS 2 environment:

```bash
cd ~/humanoid_ws
rosdep install --from-paths src --ignore-src -r -y
pip install -U --requirement src/humanoid_controller/requirements.txt # If you use a requirements.txt
```

While `rosdep` primarily handles system-level dependencies, it's good practice to manage Python-specific dependencies with `pip` within the sourced ROS 2 environment to ensure correct package resolution.

## 4. Running a Python Node

After building your package and sourcing your workspace, you can run your ROS 2 Python nodes.

### A. Creating a Simple Publisher Node

Let's create a file `~/humanoid_ws/src/humanoid_controller/humanoid_controller/simple_publisher.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher_node') # Node name
        self.publisher_ = self.create_publisher(String, 'chatter', 10) # Topic name, QoS
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello ROS 2 from Python: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args) # Initialize ROS 2 communication
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher) # Keep node alive, process callbacks
    simple_publisher.destroy_node()
    rclpy.shutdown() # Shutdown ROS 2 communication

if __name__ == '__main__':
    main()
```

Make sure the script is executable: `chmod +x ~/humanoid_ws/src/humanoid_controller/humanoid_controller/simple_publisher.py`

### B. Creating a Simple Subscriber Node

Now, create `~/humanoid_ws/src/humanoid_controller/humanoid_controller/simple_subscriber.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber_node') # Node name
        self.subscription = self.create_subscription(
            String,
            'chatter', # Topic name
            self.listener_callback,
            10) # QoS
        self.subscription # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
Make sure the script is executable: `chmod +x ~/humanoid_ws/src/humanoid_controller/humanoid_controller/simple_subscriber.py`

### C. Updating `setup.py`

Remember to add the entry points for these scripts in `humanoid_ws/src/humanoid_controller/setup.py`:

```python
# ... inside setup() function ...
        entry_points={
            'console_scripts': [
                'simple_publisher = humanoid_controller.simple_publisher:main',
                'simple_subscriber = humanoid_controller.simple_subscriber:main',
            ],
        },
# ...
```

### D. Building and Running

1.  **Build:** After modifying `setup.py`, rebuild the package:
    ```bash
    cd ~/humanoid_ws
    colcon build --packages-select humanoid_controller
    ```
2.  **Source:** Source your workspace again to pick up the new executables:
    ```bash
    source install/setup.bash
    ```
3.  **Run Publisher:**
    ```bash
    ros2 run humanoid_controller simple_publisher
    ```
4.  **Run Subscriber (in a new terminal):**
    ```bash
    source /opt/ros/humble/setup.bash
    source ~/humanoid_ws/install/setup.bash
    ros2 run humanoid_controller simple_subscriber
    ```

You should see the subscriber node receiving and printing messages from the publisher node. This demonstrates the basic publish-subscribe communication pattern in ROS 2 using Python.

## 5. Best Practices for Package Development

*   **Modular Design:** Keep nodes focused on single responsibilities.
*   **Clear Interfaces:** Define messages, services, and actions clearly.
*   **Error Handling:** Implement robust error checking and logging.
*   **Testing:** Write unit and integration tests for your nodes and libraries.
*   **Documentation:** Document your package with a clear `README.md` and inline comments.
*   **Version Control:** Use Git to manage your code effectively.

By following these guidelines, you can create maintainable, reusable, and robust ROS 2 Python packages for your humanoid robotics projects.