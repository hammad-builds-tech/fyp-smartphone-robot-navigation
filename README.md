# FYP: Smartphone-Based Indoor Robot Navigation

## Project Overview

This Final Year Project demonstrates a **low-cost indoor robot navigation system** using **smartphone monocular depth estimation** instead of expensive LiDAR or RGB-D sensors. The system integrates computer vision, deep learning, ROS2, and robotic simulation to achieve autonomous navigation with obstacle avoidance.

## Problem Statement

Traditional indoor robot navigation relies on expensive sensors:
- LiDAR scanners ($200-$2000+)
- RGB-D cameras ($150-$500+)
- Stereo vision systems (complex calibration)

This project demonstrates that a **single smartphone camera** with monocular depth estimation can provide sufficient depth information for indoor navigation at a fraction of the cost.

## Proposed Solution

Use a **smartphone camera** + **MiDaS monocular depth estimation** to generate obstacle information for a ROS2-based navigation stack running on a simulated robot in Gazebo.

### Architecture Pipeline

```
SMARTPHONE CAMERA
    ↓ (video stream)
ANDROID APP (WebSocket)
    ↓ (JPEG frames)
FASTAPI BACKEND
    ↓ (MiDaS inference)
DEPTH IMAGES
    ↓ (ROS2 bridge)
ROS2 DEPTH TOPIC (/smartphone/depth)
    ↓ (depth_to_scan node)
LASER SCAN TOPIC (/scan)
    ↓ (Nav2 obstacle layer)
LOCAL COSTMAP
    ↓ (Nav2 controller)
ROBOT COMMAND VELOCITY (/cmd_vel)
    ↓ (ros_gz_bridge)
GAZEBO ROBOT (differential drive)
    ↓
AUTONOMOUS NAVIGATION WITH OBSTACLE AVOIDANCE
```

## Hardware

- **Development Machine**: Ubuntu 26.04.1 LTS
- **GPU**: NVIDIA Quadro T2000 4GB (CUDA-enabled)
- **Smartphone**: Any Android device with camera (for live demo)
- **Robot**: Simulated differential-drive robot in Gazebo

## Software Stack

### Core Technologies

- **ROS2**: Lyrical
- **Gazebo**: 10.5.0 (Harmonic)
- **Nav2**: Full navigation stack
- **MiDaS**: DPT-Hybrid-384 monocular depth estimation
- **PyTorch**: Deep learning framework (CUDA-accelerated)
- **FastAPI**: Backend REST/WebSocket API
- **Python**: 3.12+
- **OpenCV**: Image processing
- **Android**: CameraX for smartphone video

### Key Dependencies

- ROS2 packages: `ros_gz_bridge`, `nav2_*`, `tf2_ros`
- Python packages: `torch`, `torchvision`, `opencv-python`, `fastapi`, `uvicorn`, `numpy`
- System: CUDA toolkit, NVIDIA drivers

## Repository Structure

```
~/FYP/
├── backend/                    # FastAPI depth processing backend
│   ├── api/
│   │   └── main.py            # WebSocket + REST endpoints
│   ├── depth/
│   │   └── midas_processor.py # MiDaS depth inference
│   └── requirements.txt
│
├── ros2_ws/                    # ROS2 workspace
│   └── src/
│       ├── smartphone_depth_bridge/    # Backend → ROS2 depth bridge
│       │   └── smartphone_depth_bridge/
│       │       └── depth_bridge_node.py
│       │
│       ├── indoor_nav_costmap/         # Depth processing nodes
│       │   ├── indoor_nav_costmap/
│       │   │   ├── depth_to_scan.py   # Depth → LaserScan
│       │   │   ├── depth_to_pointcloud.py
│       │   │   └── depth_obstacle_node.py
│       │   ├── config/
│       │   │   └── nav2_params.yaml   # Nav2 configuration
│       │   └── maps/
│       │       ├── indoor_map.yaml
│       │       └── indoor_map.pgm
│       │
│       └── indoor_nav_gazebo/          # Gazebo simulation
│           ├── launch/
│           │   └── fyp_bringup.launch.py  # MAIN LAUNCH FILE
│           ├── worlds/
│           │   └── indoor_world.sdf
│           └── models/
│               └── fyp_robot/
│                   └── fyp_robot.sdf
│
├── MiDaS/                      # MiDaS depth estimation (submodule)
│   └── weights/
│       └── dpt_hybrid_384.pt
│
├── MiDaS/mobile/android/       # Android smartphone app
│
├── run_backend.sh              # Start FastAPI backend
├── run_system.sh               # Launch complete ROS2/Gazebo system
├── run_rviz.sh                 # Launch RViz2 visualization
├── send_test_goal.sh           # Send test navigation goal
│
└── docs/                       # Additional documentation
```

## Setup Instructions

### 1. Prerequisites

```bash
# ROS2 Lyrical should already be installed at /opt/ros/lyrical
source /opt/ros/lyrical/setup.bash

# Install Python dependencies
cd ~/FYP
pip install torch torchvision opencv-python fastapi uvicorn websockets pillow numpy requests

# Install ROS2 dependencies
sudo apt install ros-lyrical-ros-gz-bridge ros-lyrical-ros-gz-sim \
    ros-lyrical-navigation2 ros-lyrical-nav2-bringup \
    ros-lyrical-tf2-tools ros-lyrical-rviz2
```

### 2. Build ROS2 Workspace

```bash
cd ~/FYP/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 3. Verify MiDaS Weights

```bash
ls -lh ~/FYP/MiDaS/weights/dpt_hybrid_384.pt
# Should show the weight file (~400MB)
```

## Complete Run Procedure

### Terminal 1: Backend Server

```bash
cd ~/FYP
./run_backend.sh
```

This starts the FastAPI server on `http://localhost:8000` and loads MiDaS.

### Terminal 2: ROS2 + Gazebo System

```bash
cd ~/FYP
./run_system.sh
```

This launches:
1. Gazebo simulation with indoor world
2. Robot spawn
3. ROS2 bridges (Gazebo ↔ ROS)
4. Smartphone depth bridge (backend → ROS)
5. Depth-to-scan converter
6. Map server
7. Nav2 navigation stack

Wait ~20-25 seconds for all components to initialize.

### Terminal 3: RViz2 Visualization (Optional)

```bash
cd ~/FYP
./run_rviz.sh
```

Configure RViz2:
- Fixed Frame: `map`
- Add displays: Map, LaserScan (/scan), Path, Costmaps, TF

### Terminal 4: Send Navigation Goals

#### Option A: Command Line

```bash
cd ~/FYP
./send_test_goal.sh
```

#### Option B: RViz2 Interactive

1. Click "2D Goal Pose" button
2. Click and drag on the map to set goal position and orientation
3. Robot will plan path and navigate

#### Option C: Programmatic (Python)

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

# Send goal to (x=2.0, y=0.0) in map frame
```

### Android App (Optional)

For live smartphone depth:

1. Open the Android app in `MiDaS/mobile/android`
2. Build and install on smartphone
3. Connect smartphone to same network as development machine
4. Set WebSocket URL: `ws://<your-ip>:8000/ws/video`
5. Start streaming

## ROS2 Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/smartphone/depth` | sensor_msgs/Image | Depth image from backend |
| `/scan` | sensor_msgs/LaserScan | Pseudo-LaserScan from depth |
| `/map` | nav_msgs/OccupancyGrid | Static map |
| `/cmd_vel` | geometry_msgs/Twist | Robot velocity commands |
| `/odom` | nav_msgs/Odometry | Robot odometry (from Gazebo) |
| `/tf` | tf2_msgs/TFMessage | Transform tree |
| `/plan` | nav_msgs/Path | Global plan |
| `/local_costmap/costmap` | nav_msgs/OccupancyGrid | Local costmap |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | Global costmap |

### Subscribed Topics

| Node | Subscribes To | Purpose |
|------|---------------|---------|
| depth_to_scan | /smartphone/depth | Convert depth to scan |
| controller_server | /scan, /odom, /map | Local planning |
| planner_server | /map | Global planning |
| Gazebo robot | /cmd_vel | Drive motors |

## TF Tree Structure

```
map
 └─ odom (static, for simulation)
     └─ base_link (from Gazebo odometry)
         ├─ left_wheel
         ├─ right_wheel
         ├─ caster
         └─ camera_depth_frame (static, front-mounted)
```

## Configuration Files

### Nav2 Parameters

`ros2_ws/src/indoor_nav_costmap/config/nav2_params.yaml`

Key settings:
- Robot radius: 0.35m
- Max velocity: 0.15 m/s
- Obstacle max range: 3.0m
- Controller: RegulatedPurePursuitController
- Planner: NavfnPlanner
- Costmap resolution: 0.05m

### Launch File

`ros2_ws/src/indoor_nav_gazebo/launch/fyp_bringup.launch.py`

Main integrated launch file with:
- Timed component startup
- All ROS parameters
- TF static publishers
- Lifecycle managers

### Backend Configuration

`backend/api/main.py`

- WebSocket endpoint: `/ws/video`
- Depth image endpoint: `/latest-depth-image`
- MiDaS model: dpt_hybrid_384

## Troubleshooting

### Backend Issues

**MiDaS fails to load:**
```bash
# Check weights
ls -lh ~/FYP/MiDaS/weights/dpt_hybrid_384.pt

# Test MiDaS directly
cd ~/FYP
python MiDaS/run.py --model_type dpt_hybrid_384
```

**Import errors:**
```bash
# Verify PYTHONPATH
export PYTHONPATH=/home/hammad/FYP:$PYTHONPATH
```

### ROS2 Issues

**Nodes fail to start:**
```bash
# Check if ROS2 sourced
source /opt/ros/lyrical/setup.bash
source ~/FYP/ros2_ws/install/setup.bash

# Check node status
ros2 node list
ros2 topic list
```

**TF errors:**
```bash
# View TF tree
ros2 run tf2_tools view_frames
# Creates frames.pdf

# Check specific transform
ros2 run tf2_ros tf2_echo map base_link
```

**Nav2 not receiving scan data:**
```bash
# Check scan topic
ros2 topic echo /scan --once

# Check depth bridge
ros2 topic echo /smartphone/depth --once

# Verify backend is running
curl http://localhost:8000/health
```

### Gazebo Issues

**Robot doesn't spawn:**
```bash
# Check Gazebo logs
gz topic -l

# Manually spawn robot
ros2 run ros_gz_sim create -name fyp_robot -file ~/FYP/ros2_ws/src/indoor_nav_gazebo/models/fyp_robot/fyp_robot.sdf
```

**Robot doesn't move:**
```bash
# Test cmd_vel directly
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" --once

# Check bridge
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist
```

## Performance Considerations

### MiDaS Inference

- **GPU (NVIDIA Quadro T2000)**: ~50-100ms per frame
- **CPU**: ~500-1000ms per frame
- **Memory**: ~2GB GPU, ~1GB system RAM
- **Recommended frame rate**: 5-10 FPS

### ROS2 Pipeline

- **Depth bridge polling**: 5 Hz (200ms)
- **Scan generation**: Real-time (<10ms)
- **Nav2 controller**: 10 Hz
- **Nav2 planner**: 5 Hz
- **Costmap updates**: 10 Hz (local), 2 Hz (global)

### Network

- **WebSocket**: ~100KB per frame (JPEG)
- **Bandwidth**: ~1-2 Mbps @ 10 FPS

## Current Limitations

1. **Monocular depth limitations**:
   - Relative depth only (not absolute metric depth)
   - Scale ambiguity
   - Sensitive to lighting conditions
   - No depth for textureless surfaces

2. **Simulation constraints**:
   - Robot uses odometry ground truth (no SLAM drift)
   - Static map assumed known
   - Simplified physics

3. **Real-world deployment gaps**:
   - No dynamic obstacle tracking
   - No re-localization after loss
   - No multi-floor support
   - Network latency not optimized for production

4. **Nav2 configuration**:
   - Fixed robot radius
   - Conservative velocity limits for demo stability
   - No recovery behaviors beyond basic Nav2 defaults

## Future Improvements

### Short Term

- [ ] RViz2 config file for one-click visualization setup
- [ ] Depth data quality metrics and monitoring
- [ ] Auto-restart for backend crashes
- [ ] Docker containers for easier deployment
- [ ] CI/CD pipeline for testing

### Medium Term

- [ ] Depth fusion with multiple frames
- [ ] Scale recovery using visual odometry
- [ ] Dynamic obstacle detection and tracking
- [ ] Android app improvements (auto-reconnect, compression)
- [ ] Support for recorded video file input

### Long Term

- [ ] Deploy on real differential-drive robot
- [ ] SLAM integration for unknown environments
- [ ] Multi-agent coordination
- [ ] Real-time path optimization
- [ ] Edge deployment (run MiDaS on smartphone)

## Final Demonstration Procedure

### Pre-Demo Checklist

- [ ] Backend running and MiDaS loaded
- [ ] ROS2 system launched
- [ ] Gazebo showing robot in world
- [ ] Nav2 lifecycle nodes active
- [ ] TF tree complete (map → odom → base_link → camera_depth_frame)
- [ ] /scan topic publishing (even if no phone connected yet)

### Demo Steps

1. **Show system architecture diagram**
2. **Start backend** - show MiDaS loading
3. **Launch ROS2/Gazebo** - show node graph
4. **Open RViz2** - visualize map, TF, costmaps
5. **(Optional) Connect smartphone** - show live depth
6. **Send navigation goal** - demonstrate autonomous navigation
7. **(Optional) Add virtual obstacle** - show obstacle avoidance
8. **Show topic monitoring** - demonstrate data flow

### Key Demo Points

- Monocular depth replaces LiDAR
- Complete ROS2 integration
- Nav2 working with depth-based scan
- Gazebo simulation validates concept
- Android app provides real smartphone data

## Credits and Acknowledgments

### Third-Party Components

- **MiDaS**: Intel ISL - [https://github.com/isl-org/MiDaS](https://github.com/isl-org/MiDaS)
- **Nav2**: Open Navigation - [https://navigation.ros.org](https://navigation.ros.org)
- **ROS2**: Open Robotics - [https://www.ros.org](https://www.ros.org)
- **Gazebo**: Open Robotics - [https://gazebosim.org](https://gazebosim.org)

### References

- Ranftl, R., et al. "Towards Robust Monocular Depth Estimation: Mixing Datasets for Zero-shot Cross-dataset Transfer." IEEE TPAMI (2020)
- ROS2 Navigation Tutorials: [https://navigation.ros.org](https://navigation.ros.org)

## License

This project is developed as a Final Year Project for educational purposes.

## Contact

For questions or issues, please refer to the project repository.

---

**Project Status**: ✅ Complete and demonstrable

**Last Updated**: September 16, 2026

**Version**: 1.0.0
