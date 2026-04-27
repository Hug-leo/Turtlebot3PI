# Turtlebot3PI — Raspberry Pi ROS 2 Workspace

ROS 2 Humble workspace for a **TurtleBot3-based AMR** (Autonomous Mobile Robot) running on a Raspberry Pi. Provides differential-drive motor control, SLAM mapping, autonomous navigation (Nav2), QR-code scanning, and multi-waypoint warehouse missions.

> **Dashboard / Server** — see the companion repo [WarehouseUI](https://github.com/Hug-leo/WarehouseUI) for the FastAPI backend and web-based dashboard that controls this robot.

---

## Table of Contents

- [Features](#features)
- [Architecture](#architecture)
- [Folder Structure](#folder-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Building](#building)
- [Running](#running)
  - [Manual Bring-up](#manual-bring-up)
  - [SLAM Mapping](#slam-mapping)
  - [Autonomous Navigation](#autonomous-navigation)
- [Nodes](#nodes)
- [Launch Files](#launch-files)
- [Configuration](#configuration)
- [ROS 2 Topics](#ros-2-topics)
- [Troubleshooting](#troubleshooting)

---

## Features

| Feature                | Description                                                                                        |
| ---------------------- | -------------------------------------------------------------------------------------------------- |
| **Differential Drive** | Serial (UART) motor controller with encoder-based odometry and BNO055 IMU fusion                   |
| **SLAM Mapping**       | `slam_toolbox` async mapping managed via ROS topics from the dashboard                             |
| **Nav2 Navigation**    | Full Nav2 stack — AMCL localization, path planning, RPP controller, behavior trees                 |
| **EKF Sensor Fusion**  | `robot_localization` EKF node fuses wheel odometry + IMU for smooth `odom → base_link` TF          |
| **SLAM Manager**       | Lifecycle node to start/stop SLAM, save/load maps, and launch Nav2 — all via `/slam/command` topic |
| **QR Scanner**         | Camera-based QR code scanning with automatic POST to the warehouse backend                         |
| **Warehouse Mission**  | Multi-waypoint autonomous mission executor using Nav2 action client                                |
| **Camera Stream**      | ROS camera → MJPEG bridge for live video feed in the dashboard                                     |
| **rosbridge**          | WebSocket bridge (port 9090) enabling the web dashboard to communicate with all ROS topics         |

---

## Architecture

```
┌────────────────────────────────────────────────────────────┐
│              Raspberry Pi (Ubuntu 22.04 arm64)              │
│                                                            │
│  ┌──────────────────────┐  ┌────────────────────────────┐  │
│  │  rosbridge_server     │  │  slam_manager_node          │  │
│  │  (WebSocket :9090)    │  │  - SLAM start / stop / save │  │
│  │  ↕ Dashboard (PC)     │  │  - Map load + Nav2 launch   │  │
│  └──────────────────────┘  └────────────────────────────┘  │
│                                                            │
│  ┌──────────────────────┐  ┌────────────────────────────┐  │
│  │  diff_drive_controller│  │  Nav2 bringup               │  │
│  │  UART ↔ STM32 motors │  │  AMCL + Planner + RPP       │  │
│  │  /odom + /imu publish │  │  + Behavior + Costmaps      │  │
│  └──────────────────────┘  └────────────────────────────┘  │
│                                                            │
│  ┌──────────────────────┐  ┌────────────────────────────┐  │
│  │  EKF (robot_localiz.) │  │  LiDAR driver (HLS-LFCD)   │  │
│  │  odom → base_link TF  │  │  /scan topic                │  │
│  └──────────────────────┘  └────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘
```

---

## Folder Structure

```
Turtlebot3PI-master/
├── README.md                       # This file
└── src/
    ├── agv_controller/             # Main ROS 2 Python package
    │   ├── agv_controller/
    │   │   ├── __init__.py
    │   │   ├── diff_drive_controller.py   # Motor control + odometry + IMU
    │   │   ├── slam_manager_node.py       # SLAM / Map / Nav lifecycle manager
    │   │   ├── qr_scanner_node.py         # Camera QR scanner + POST to backend
    │   │   ├── warehouse_mission_node.py  # Multi-waypoint Nav2 mission executor
    │   │   ├── camera_stream.py           # ROS camera → MJPEG bridge
    │   │   ├── draw_and_follow.py         # Mouse-click waypoint follower (demo)
    │   │   ├── linear_test.py             # Nav2 accuracy / error measurement test
    │   │   └── square_test.py             # 5-lap square trajectory test
    │   ├── config/
    │   │   ├── ekf.yaml                   # EKF sensor fusion parameters
    │   │   ├── slam_handheld.yaml         # slam_toolbox async parameters
    │   │   └── my_nav2_params.yaml        # Nav2 planner / controller parameters
    │   ├── launch/
    │   │   ├── handheld_slam.launch.py    # LiDAR + motor + SLAM (mapping mode)
    │   │   └── nav2_custom.launch.py      # LiDAR + motor + EKF + Nav2 (navigation mode)
    │   ├── setup.py                       # 4 console_script entry points
    │   ├── setup.cfg
    │   └── package.xml
    ├── GUI/
    │   └── pid_analyzer.py                # PID tuning visualization tool
    └── hls_lfcd_lds_driver/               # HLS-LFCD LiDAR driver (external package)
```

---

## Prerequisites

- **Raspberry Pi 4** (or compatible SBC)
- **Ubuntu 22.04** (arm64)
- **ROS 2 Humble** — [Install guide](https://docs.ros.org/en/humble/Installation.html)
- **Hardware:**
  - HLS-LFCD LDS LiDAR (connected via `/dev/ttyUSB0`)
  - STM32-based motor controller (connected via `/dev/ttyACM0`)
  - BNO055 IMU (read through STM32)

### ROS 2 Dependencies

```bash
sudo apt install ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-rosbridge-suite \
    ros-humble-robot-state-publisher \
    ros-humble-robot-localization \
    ros-humble-tf2-ros
```

### Python Dependencies

```bash
pip install pyserial numpy pyzbar opencv-python requests
```

---

## Installation

```bash
# Clone repo
git clone https://github.com/Hug-leo/Turtlebot3PI-master.git ~/Turtlebot3PI-master
cd ~/Turtlebot3PI-master
```

---

## Building

```bash
cd ~/Turtlebot3PI-master
colcon build --packages-select agv_controller
source install/setup.bash
```

> Add `source ~/Turtlebot3PI-master/install/setup.bash` to your `~/.bashrc` for auto-sourcing.

---

## Running

### Manual Bring-up

Start rosbridge so the dashboard can connect:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Start the SLAM manager (listens for commands from the dashboard):

```bash
ros2 run agv_controller slam_manager
```

### SLAM Mapping

Launch LiDAR + motor controller + slam_toolbox:

```bash
ros2 launch agv_controller handheld_slam.launch.py
```

Or trigger from the dashboard via `/slam/command` topic → `start_slam`.

### Autonomous Navigation

Launch full Nav2 stack with a saved map:

```bash
ros2 launch agv_controller nav2_custom.launch.py map:=$HOME/maps/warehouse_map.yaml
```

Or load from the dashboard's SLAM/Map Manager panel.

---

## Nodes

### diff_drive_controller

Serial motor controller node that interfaces with the STM32 via UART.

| Feature     | Detail                                           |
| ----------- | ------------------------------------------------ |
| Subscribes  | `/cmd_vel` (Twist) — velocity commands           |
| Publishes   | `/odom` (Odometry) — wheel encoder odometry      |
| Publishes   | `/imu` (Imu) — BNO055 orientation + acceleration |
| Broadcasts  | TF `odom → base_link`                            |
| Serial port | `/dev/ttyACM0` at 115200 baud                    |

### slam_manager_node

Lifecycle manager for SLAM and Navigation. Controlled via `/slam/command` topic:

| Command           | Action                                                  |
| ----------------- | ------------------------------------------------------- |
| `start_slam`      | Launch slam_toolbox in async mode                       |
| `stop_slam`       | Kill SLAM processes                                     |
| `save_map:<name>` | Save current map to `~/maps/<name>`                     |
| `list_maps`       | Publish available map names to `/slam/map_list`         |
| `load_map:<name>` | Stop SLAM if running, launch Nav2 with the selected map |
| `stop_nav`        | Kill Nav2 processes                                     |

Status published at 1 Hz on `/slam/status`:  
`IDLE` · `MAPPING` · `SAVING` · `SAVED:<name>` · `NAV:<name>` · `LOADING:<name>` · `ERROR:<msg>`

### qr_scanner_node

Camera-based QR code scanner. Detects QR codes using pyzbar and POSTs scan data to the warehouse backend API (`/scan`).

### warehouse_mission_node

Multi-waypoint autonomous mission executor. Receives a sequence of waypoints and navigates through them in order using the Nav2 `NavigateToPose` action client.

### camera_stream

Bridges the ROS camera topic to an MJPEG HTTP stream for display in the dashboard's Camera tab.

---

## Launch Files

### handheld_slam.launch.py

Mapping mode — starts LiDAR driver, motor controller, and slam_toolbox:

| Component              | Package               | Node                         |
| ---------------------- | --------------------- | ---------------------------- |
| LiDAR                  | `hls_lfcd_lds_driver` | `hlds_laser_publisher`       |
| TF (base_link → laser) | `tf2_ros`             | `static_transform_publisher` |
| Motor + Odom           | `agv_controller`      | `diff_drive_controller`      |
| SLAM                   | `slam_toolbox`        | `async_slam_toolbox_node`    |

### nav2_custom.launch.py

Navigation mode — starts everything needed for autonomous driving:

| Component                   | Package               | Node                         |
| --------------------------- | --------------------- | ---------------------------- |
| LiDAR                       | `hls_lfcd_lds_driver` | `hlds_laser_publisher`       |
| TF (3 static frames)        | `tf2_ros`             | `static_transform_publisher` |
| Motor + Odom + IMU          | `agv_controller`      | `diff_drive_controller`      |
| EKF Sensor Fusion           | `robot_localization`  | `ekf_node`                   |
| Nav2 (AMCL + Planner + RPP) | `nav2_bringup`        | `bringup_launch.py`          |

Accepts `map` launch argument: `map:=/path/to/map.yaml`

---

## Configuration

### ekf.yaml

EKF sensor fusion parameters for `robot_localization`:

- Runs at **20 Hz** (matches STM32 output rate)
- 2D mode enabled
- Fuses **linear velocity (vx)** from `/odom` + **yaw orientation** from `/imu/data`

### slam_handheld.yaml

slam_toolbox async mapping parameters.

### my_nav2_params.yaml

Nav2 configuration — includes AMCL, planner server, controller server (RPP), costmap settings, and behavior tree configuration.

---

## ROS 2 Topics

| Topic            | Type                                      | Direction | Description                           |
| ---------------- | ----------------------------------------- | --------- | ------------------------------------- |
| `/cmd_vel`       | `geometry_msgs/Twist`                     | Subscribe | Velocity commands from teleop or Nav2 |
| `/odom`          | `nav_msgs/Odometry`                       | Publish   | Wheel encoder odometry                |
| `/imu`           | `sensor_msgs/Imu`                         | Publish   | BNO055 IMU data                       |
| `/scan`          | `sensor_msgs/LaserScan`                   | Publish   | LiDAR range data                      |
| `/map`           | `nav_msgs/OccupancyGrid`                  | Publish   | Occupancy grid (SLAM or map_server)   |
| `/plan`          | `nav_msgs/Path`                           | Publish   | Nav2 planned path                     |
| `/amcl_pose`     | `geometry_msgs/PoseWithCovarianceStamped` | Publish   | Localized robot pose                  |
| `/slam/command`  | `std_msgs/String`                         | Subscribe | SLAM manager commands                 |
| `/slam/status`   | `std_msgs/String`                         | Publish   | SLAM manager status (1 Hz)            |
| `/slam/map_list` | `std_msgs/String`                         | Publish   | JSON array of saved map names         |
| `/goal_pose`     | `geometry_msgs/PoseStamped`               | Subscribe | Navigation goals from dashboard       |

---

## Troubleshooting

| Problem                          | Solution                                                                                                            |
| -------------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| **LiDAR not detected**           | Check `/dev/ttyUSB0` exists. If different port, update the launch file parameter.                                   |
| **Motor not responding**         | Verify STM32 is on `/dev/ttyACM0`. Check `ls /dev/ttyACM*` and update `diff_drive_controller.py` if needed.         |
| **SLAM won't start**             | Ensure LiDAR is publishing on `/scan`. Run `ros2 topic echo /scan` to verify. SLAM and Nav2 are mutually exclusive. |
| **Nav2 fails to start**          | Verify the map file exists at the specified path. Check `~/maps/` for saved maps.                                   |
| **EKF drift**                    | Ensure IMU is calibrated. Check `/imu` topic data quality. Recalibrate BNO055 if needed.                            |
| **rosbridge connection refused** | Run `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`. Check Pi firewall allows port 9090.              |
| **colcon build fails**           | Source ROS 2 first: `source /opt/ros/humble/setup.bash`. Install missing deps with `rosdep install`.                |

---

## License

Apache License 2.0
