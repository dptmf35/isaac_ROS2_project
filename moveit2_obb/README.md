# MoveIt2 OBB - Bolt Detection and Pick & Place Simulation

A ROS2-based Gazebo simulation project that detects bolts using YOLOv8 OBB (Oriented Bounding Box) detector and performs pick-and-place operations with a Franka Emika Panda manipulator using MoveIt2.

## Table of Contents
- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [How It Works](#how-it-works)
- [Configuration](#configuration)
- [Troubleshooting](#troubleshooting)

## Overview

This project demonstrates an autonomous robotic manipulation system that:
- Detects bolts in a simulated environment using YOLOv8 OBB detection
- Calculates 3D coordinates from camera images using camera intrinsics
- Plans collision-free trajectories using MoveIt2
- Executes pick-and-place operations with the Panda manipulator
- Provides an intuitive GUI for bolt selection

**Key Features:**
- Real-time bolt detection with oriented bounding boxes
- Camera-to-robot coordinate transformation
- Interactive PyQt5-based bolt selection interface
- Full Gazebo simulation with physics engine
- MoveIt2 motion planning and execution
- RViz visualization

## System Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Gazebo + Camera│────▶│  YOLOv8 Detector │────▶│   PyQt5 GUI     │
│   Simulation    │     │  (OBB Publisher) │     │ (bolt_selector) │
└─────────────────┘     └──────────────────┘     └─────────────────┘
                                                           │
                                                           │ User Click
                                                           ▼
                        ┌──────────────────┐     ┌─────────────────┐
                        │  Panda Robot +   │◀────│ Coordinate      │
                        │  MoveIt2 Control │     │ Transformation  │
                        └──────────────────┘     └─────────────────┘
```

## Prerequisites

- **ROS2 Humble** (tested on ROS2 Humble)
- **Ubuntu 22.04** (recommended)
- **Python 3.10+**
- **Gazebo Sim** (Ignition Gazebo)
- **MoveIt2**

### Required ROS2 Packages
```bash
sudo apt install ros-humble-moveit ros-humble-moveit-py \
  ros-humble-ros-gz-sim ros-humble-ros-gz-bridge \
  ros-humble-controller-manager ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller
```

### Required Python Packages
```bash
pip3 install ultralytics opencv-python PyQt5 numpy
```

## Installation

1. **Clone the repository**
```bash
cd ~/your_workspace
git clone <repository_url>
cd isaac_ROS2_project/moveit2_obb
```

2. **Build the workspace**
```bash
colcon build
source install/setup.bash
```

3. **Verify YOLOv8 weights**
Ensure the YOLOv8 model weights are present at:
```bash
src/yolov8_obb/scripts/best.pt
```

## Usage

### Quick Start

Open **three separate terminals** and run the following commands in order:

#### Terminal 1: Launch Gazebo Simulation with Panda Robot
```bash
cd ~/your_workspace/isaac_ROS2_project/moveit2_obb
source install/setup.bash
ros2 launch panda_moveit_config moveit_gazebo_obb.py
```

This will:
- Start Gazebo with the table and bolt models
- Spawn the Panda robot
- Launch MoveIt2 motion planning node
- Open RViz for visualization
- Start the arm control node

#### Terminal 2: Launch YOLOv8 Bolt Detection
```bash
cd ~/your_workspace/isaac_ROS2_project/moveit2_obb
source install/setup.bash
ros2 launch yolov8_obb yolov8_obb.launch.py
```

This will:
- Subscribe to `/image_raw` from the Gazebo camera
- Run YOLOv8 OBB detection on each frame
- Publish detection results to `/Yolov8_Inference`

#### Terminal 3: Launch Bolt Selector GUI
```bash
cd ~/your_workspace/isaac_ROS2_project/moveit2_obb/UI
python3 bolt_selector.py
```

This will:
- Display the camera feed with detected bolt bounding boxes
- Allow interactive bolt selection by clicking

### Operating the System

1. **Wait for initialization**: Allow 5-10 seconds for all nodes to initialize
2. **Observe detection**: Detected bolts appear as blue bounding boxes in the GUI
3. **Select a bolt**: Hover over a detected bolt (bounding box turns red), then click
4. **Watch execution**: The Panda manipulator will:
   - Move to a position above the bolt
   - Open the gripper
   - Move down to grasp height
   - Close the gripper
   - Lift the bolt
   - Move to the drop-off location (0.3, -0.3)
   - Release the bolt

## Project Structure

```
moveit2_obb/
├── build/                  # Compiled output (ignored by git)
├── install/                # Installation files (ignored by git)
├── log/                    # Build logs (ignored by git)
├── src/
│   ├── panda_moveit_config/
│   │   ├── config/         # MoveIt2 and robot configuration files
│   │   ├── launch/         # Launch files
│   │   │   └── moveit_gazebo_obb.py
│   │   ├── scripts/        # Python scripts
│   │   │   └── arm_control_from_UI.py
│   │   └── worlds/         # Gazebo world files and models
│   │       ├── arm_on_the_table.sdf
│   │       └── bolt/       # Bolt model
│   ├── robot_description/
│   │   ├── meshes/         # Robot collision and visual meshes
│   │   └── urdf/           # Robot URDF files
│   │       └── camera/     # Camera configuration
│   ├── yolov8_obb/
│   │   ├── launch/         # YOLOv8 launch files
│   │   └── scripts/
│   │       ├── yolov8_obb_publisher.py
│   │       ├── yolov8_obb_subscriber.py
│   │       └── best.pt     # YOLOv8 model weights
│   └── yolov8_obb_msgs/
│       └── msg/            # Custom ROS2 messages
│           ├── InferenceResult.msg
│           └── Yolov8Inference.msg
└── UI/
    ├── bolt_selector.py        # Main GUI application
    └── bolt_selector_window.ui # PyQt5 UI design file
```

## How It Works

### 1. Bolt Detection (YOLOv8 OBB)
- Camera publishes images to `/image_raw` topic
- YOLOv8 node subscribes and performs OBB detection
- Detection results (coordinates, class, confidence) published to `/Yolov8_Inference`

### 2. Coordinate Transformation
The bolt selector GUI converts 2D image coordinates to 3D robot coordinates:

```python
# Camera intrinsics
fx = 253.93635749816895  # Focal length X
fy = 253.93635749816895  # Focal length Y
cx = 320                  # Principal point X
cy = 240                  # Principal point Y
z = 0.7                   # Fixed height from camera

# Transform to robot frame
x_robot = -z * (pixel_y - cy) / fy + init_x
y_robot = -z * (pixel_x - cx) / fx + init_y
```

**Camera-Robot Offset:**
- `init_x = 0.2` (camera position relative to robot link0 in X)
- `init_y = 0.6` (camera position relative to robot link0 in Y)

### 3. Bolt Orientation Calculation
The system calculates bolt orientation from OBB corners:
```python
# Determine longer edge for orientation
dist1 = distance(point[0], point[1])
dist2 = distance(point[1], point[2])

# Calculate angle of longer edge
angle = atan2(dy, dx)
theta_robot = π/2 - angle
```

### 4. Motion Planning and Execution
When a bolt is clicked, the system executes:

```python
# Motion sequence
1. Move to (x, y, 0.18) - Approach height
2. Open gripper
3. Move to (x, y, 0.126) - Grasp height
4. Close gripper
5. Move to (x, y, 0.3) - Lift height
6. Move to (0.3, -0.3, 0.3) - Drop-off location
7. Open gripper
```

Each motion is planned using MoveIt2's collision-aware path planning.

## Configuration

### Camera Calibration
Modify camera intrinsics in `UI/bolt_selector.py`:
```python
self.fx = 253.93635749816895  # Focal length X
self.fy = 253.93635749816895  # Focal length Y
self.cx = 320                  # Principal point X
self.cy = 240                  # Principal point Y
```

### Robot-Camera Transform
Adjust offset in `UI/bolt_selector.py`:
```python
self.init_x = 0.2  # Camera X offset from robot base
self.init_y = 0.6  # Camera Y offset from robot base
self.z = 0.7       # Camera height above table
```

### Pick-and-Place Heights
Modify in `src/panda_moveit_config/scripts/arm_control_from_UI.py`:
```python
self.height = 0.18          # Approach height
self.pick_height = 0.126    # Grasp height
self.carrying_height = 0.3  # Lift height
```

### YOLOv8 Model
Replace the model weights at:
```bash
src/yolov8_obb/scripts/best.pt
```

## ROS2 Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/image_raw` | `sensor_msgs/Image` | Raw camera images from Gazebo |
| `/Yolov8_Inference` | `yolov8_msgs/Yolov8Inference` | YOLOv8 detection results |
| `/target_point` | `std_msgs/Float64MultiArray` | Target position [x, y, θ] |
| `/joint_states` | `sensor_msgs/JointState` | Robot joint states |
| `/clock` | `rosgraph_msgs/Clock` | Simulation time |

## Troubleshooting

### Gazebo doesn't start
- Check if Gazebo is already running: `pkill -9 gz`
- Verify GZ_SIM_RESOURCE_PATH is set correctly

### YOLOv8 detection fails
- Ensure `best.pt` model weights exist
- Check camera topic: `ros2 topic echo /image_raw`
- Verify PyTorch and Ultralytics are installed

### MoveIt2 planning fails
- Check joint limits in `config/joint_limits.yaml`
- Verify controller spawning: `ros2 control list_controllers`
- Increase planning time in MoveIt2 config

### GUI doesn't show camera feed
- Verify all nodes are running: `ros2 node list`
- Check topic connections: `ros2 topic info /image_raw`
- Restart the bolt_selector.py

### Robot doesn't move
- Check if controllers are loaded: `ros2 control list_controllers`
- Verify target coordinates are reachable
- Check RViz for collision warnings

### Build errors
```bash
# Clean build
rm -rf build/ install/ log/
colcon build
source install/setup.bash
```

## Dependencies

### Python Packages
- `rclpy` - ROS2 Python client
- `ultralytics` - YOLOv8 implementation
- `opencv-python` - Image processing
- `PyQt5` - GUI framework
- `numpy` - Numerical operations
- `cv_bridge` - ROS-OpenCV bridge

### ROS2 Packages
- `moveit_py` - MoveIt2 Python API
- `ros_gz_sim` - Gazebo simulation
- `ros_gz_bridge` - ROS-Gazebo bridge
- `controller_manager` - ROS2 control

## License

[Specify your license here]

## Contributors

- [Your name/team]

## Acknowledgments

- Franka Emika Panda robot model from MoveIt2 resources
- YOLOv8 by Ultralytics
- ROS2 Humble community

## Contact

For questions or issues, please open an issue on the repository or contact [your contact info].

---

**Last Updated:** October 28, 2025

