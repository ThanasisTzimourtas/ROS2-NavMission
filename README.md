# ROS2-NavMission

A ROS2 framework for autonomous robot navigation with waypoint missions, lifecycle nodes, and dynamic path selection. Features include circle movements, fault tolerance, and a TCP/IP server that broadcasts real-time status to external devices. Adaptable to any ROS2-compatible mobile robot.

## 📋 Overview

ROS2-NavMission provides a complete solution for autonomous navigation of mobile robots using ROS2. The framework includes:

- **Mission Manager**: Orchestrates the entire navigation mission
- **Waypoint Navigator**: Handles navigation between waypoints
- **Path Selector**: Intelligently selects alternative paths
- **Circle Node**: Executes special circle movements at designated waypoints
- **Status Server**: TCP/IP server for broadcasting robot status to external devices

This framework is built on ROS2 lifecycle nodes, ensuring robust state management and graceful transitions between operational states.

## 🔧 Dependencies

### Required ROS2 Packages
- ROS2 Humble or later
- nav2_msgs
- geometry_msgs
- lifecycle_msgs
- std_srvs
- tf2_ros
- rclpy
- twist_mux
- clearpath_nav2_demos
- clearpath_gz (for simulation)
- clearpath_viz (for visualization)

### Python Dependencies
- PyYAML
- socket
- asyncio
- threading
- uuid
- math

## 🛠️ Installation

### 1. Clone the Repository
```bash
git clone https://github.com/yourusername/ROS2-NavMission.git
cd ROS2-NavMission
```

### 2. Install Dependencies
```bash
sudo apt update
sudo apt install -y python3-pip ros-${ROS_DISTRO}-nav2-msgs ros-${ROS_DISTRO}-lifecycle-msgs ros-${ROS_DISTRO}-tf2-ros ros-${ROS_DISTRO}-twist-mux ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-nav2-lifecycle-manager

# For simulation support
sudo apt install -y ros-${ROS_DISTRO}-ros-gz-sim

# Install Python dependencies
pip3 install pyyaml
```

### 3. Set Up Map

The repository includes the map file `clearpath_map.yaml` in the `map` directory. You need to copy this to your preferred location and update the launch file:

1. Create a map directory:
```bash
mkdir -p /home/$USER/map
```

2. Copy the map file from the repository to this location:
```bash
cp ~/ROS2-NavMission/map/clearpath_map.yaml /home/$USER/map/
```

3. **Important**: Update the map path in the launch file (`waypoint_navigator.launch.py`):
   - Open the launch file for editing:
   ```bash
   nano ~/ROS2-NavMission/src/waypoint_navigator/launch/waypoint_navigator.launch.py
   ```
   - Find the line with `launch_arguments={'map': '/home/...` and modify it to point to your map location:
   ```python
   launch_arguments={'map': '/home/' + os.environ['USER'] + '/map/clearpath_map.yaml'}.items()
   ```
   - Or use the exact path where you placed the map:
   ```python
   launch_arguments={'map': '/your/exact/path/to/clearpath_map.yaml'}.items()
   ```

4. Configuration Files:
   - The package includes several configuration files in the `config` directory:
     - `last_robot_pose.yaml`: Saved robot pose used for initialization
     - `twist_mux.yaml`: Configuration for the twist multiplexer
     - `nav2_params.yaml`: Parameters for the navigation stack


### 4. Build the Package
```bash
cd ~/ROS2-NavMission
colcon build --packages-select waypoint_navigator
```

### 5. Source the Workspace
```bash
source ~/ROS2-NavMission/install/setup.bash
```

## 🚀 Usage

### Launch Files

The package includes a launch file that starts all the necessary components:

```bash
# Launch the complete navigation system using the provided launch file
ros2 launch waypoint_navigator waypoint_navigator.launch.py
```

### Simulation Environment

To test in a simulation environment, you need to launch the following in order:

```bash
# 1. Launch the Clearpath Gazebo simulation
ros2 launch clearpath_gz simulation.launch.py

# 2. Launch the visualization tools
ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_0000

# 3. Launch the waypoint navigation system
ros2 launch waypoint_navigator waypoint_navigator.launch.py

# 4. Launch the status monitoring client
cd ~/ROS2-NavMission/client
python3 mission_client.py
```

### Configuration

1. Edit the waypoints in `mission_manager.py` if needed:
```python
self.waypoints = [
    self.create_pose_stamped(0.056, 0.016, 0.031),  
    self.create_pose_stamped(10.097, 6.909, 0.003),
]
```

2. Adjust the path selection options in `path_selector_node.py` if needed:
```python
self.alternative_poses = [
    self.create_pose_stamped(11.175, 21.708, 1.582),  # Option 1
    self.create_pose_stamped(-12.745, -8.339, 1.605)  # Option 2
]
```

3. The launch file (`waypoint_navigator.launch.py`) includes important configuration:
   - Map path: Update to match your environment 
   - Twist multiplexer configuration for command priority
   - Parameters for the circle node (linear and angular velocity)
   - Namespace configuration (`a200_0000`)

### Monitor Robot Status

The robot status is broadcasted via TCP/IP server on port 12345. A client script is included in the repository to monitor these status updates:

1. Run the client script:
```bash
cd ~/ROS2-NavMission/client
python3 mission_client.py
```

2. The client will display status messages:
   - **Device Off - Mission Complete** (status code 0)
   - **Device On - Navigating Waypoints** (status code 1)
   - **Device Pausing - Executing Circle** (status code 2)

You can also create your own client or integrate with external systems:
```python
import socket

# Connect to the robot status server
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect(('127.0.0.1', 12345))  # Use robot's IP when not on localhost

# Receive status updates
while True:
    data = client.recv(1024)
    if not data:
        break
    status = int(data.decode())
    print(f"Robot status: {status}")
```

## 📁 Project Structure

```
ROS2-NavMission/               # Repository/Workspace
├── src/                       # Source directory
│   └── waypoint_navigator/    # ROS2 package
│       ├── waypoint_navigator/  # Python package
│       │   ├── circle_node.py   # Handles circle movements
│       │   ├── mission_manager.py # Orchestrates the entire mission
│       │   ├── path_selector_node.py # Selects alternative paths
│       │   ├── server_node.py   # TCP/IP server for status updates
│       │   └── waypoint_navigator.py # Handles navigation between waypoints
│       ├── config/            # Configuration files
│       │   ├── last_robot_pose.yaml  # Saved robot pose
│       │   ├── twist_mux.yaml        # Twist multiplexer config
│       │   └── nav2_params.yaml      # Navigation parameters
│       ├── launch/
│       │   └── waypoint_navigator.launch.py # Launch file
│       ├── resource/
│       │   └── waypoint_navigator     # Package marker
│       ├── test/               # Test files
│       │   ├── test_copyright.py
│       │   ├── test_flake8.py
│       │   └── test_pep257.py
│       ├── package.xml        # Package manifest
│       ├── setup.py           # Package setup
│       └── setup.cfg          # Setup configuration
├── client/                    # Client for status monitoring
│   └── mission_client.py      # TCP client to monitor robot status
└── map/
    └── clearpath_map.yaml     # Map file for navigation
```

## 🧪 Testing

The framework can be tested in a simulated environment using ROS2's Gazebo integration:

1. Launch the simulation environment in the correct order:
```bash
# 1. Start the Gazebo simulation with Clearpath robot
ros2 launch clearpath_gz simulation.launch.py

# 2. Launch the visualization tools
ros2 launch clearpath_viz view_navigation.launch.py namespace:=a200_0000

# 3. Launch the waypoint navigation system
ros2 launch waypoint_navigator waypoint_navigator.launch.py
```

The launch file handles starting all necessary components, including:
- Core navigation stack (localization and navigation)
- Transform publishers
- Twist multiplexer for command velocity
- Status server for external monitoring
- Path selection and navigation nodes
- Mission manager (starts after a delay to allow initialization)

## 🔍 Lifecycle Node States

The framework uses ROS2 lifecycle nodes with the following states:

1. **Unconfigured**: Initial state
2. **Inactive**: Node is configured but not active
3. **Active**: Node is fully operational
4. **Finalized**: Node is shutting down

You can manually control the lifecycle states using ROS2 service calls:
```bash
# Configure a node
ros2 service call /a200_0000/circle_node/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"

# Activate a node
ros2 service call /a200_0000/circle_node/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"

# Deactivate a node
ros2 service call /a200_0000/circle_node/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 4}}"
```
