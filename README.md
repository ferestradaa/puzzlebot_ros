# Puzzlebot Navigation Stack

> Autonomous differential drive robot with LiDAR-based SLAM, ArUco landmark detection, and full ROS2 navigation — implemented from scratch without Nav2.

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Repository Structure](#repository-structure)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Docker Setup](#docker-setup)
- [Isaac Sim Integration](#isaac-sim-integration)
- [Jetson Nano Deployment](#jetson-nano-deployment)
- [Usage](#usage)
- [Configuration](#configuration)
- [Package Reference](#package-reference)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Overview

This workspace contains the full ROS2 navigation stack for a Puzzlebot differential drive robot equipped with an RPLIDAR S2E and a camera for ArUco marker detection. The system implements:

- **EKF-SLAM** with ArUco landmarks (no Nav2)
- **Occupancy grid mapping** from raw LiDAR scans
- **Differential drive odometry** in C++
- **Sim-to-real transfer** via Isaac Sim digital twin

**Hardware:** Puzzlebot + RPLIDAR S2E + Camera  
**Software:** ROS2 Humble | Isaac Sim 4.5 / 5.1 | Ubuntu 22.04  
**GPU:** NVIDIA RTX 5090 (Blackwell, sm_120)

---

## System Architecture


---

## Repository Structure

```
puzz_ws/
├── build/              # Colcon build artifacts (auto-generated)
├── docker/             # Docker Compose configs and Dockerfiles
│   ├── Dockerfile
│   └── docker-compose.yml
├── install/            # Colcon install space (auto-generated)
├── isaac_sim/          # Isaac Sim assets, USD scenes, OmniGraph configs
│   ├── scenes/
│   ├── urdf/
│   └── omnigraph/
├── jetson/             # Jetson Nano deployment scripts and configs
│   ├── setup.sh
│   └── inference/
├── log/                # Colcon and ROS2 launch logs (auto-generated)
├── params/             # YAML parameter files for all nodes
│   ├── slam_params.yaml
│   ├── ekf_params.yaml
│   └── control_params.yaml
└── src/                # ROS2 source packages
    ├── puzzlebot_slam/
    ├── puzzlebot_control/
    └── puzzlebot_description/
```



## Installation

### 1. Clone the repository

```bash
git clone https://github.com/<your-user>/puzz_ws.git
cd puzz_ws
```

### 2. Install ROS2 dependencies

```bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build the workspace

```bash
colcon build --symlink-install
source install/setup.bash
```

### 4. (Optional) Add to `.bashrc`

```bash
echo "source ~/puzz_ws/install/setup.bash" >> ~/.bashrc
```

---

## Docker Setup

The Docker environment provides a reproducible ROS2 Humble container with all dependencies.

```bash
cd docker/

# Build image
docker compose build

# Run container
docker compose up -d

# Attach to container
docker exec -it puzzlebot_ros2 bash
```

## Isaac Sim Integration

Simulation assets and OmniGraph configurations for the Puzzlebot digital twin.

### Running the simulation




## Jetson Nano Deployment


**Inference pipeline:** TensorRT-optimized policy → ROS2 `/cmd_vel` publisher

See [`jetson/README.md`](jetson/README.md) for detailed deployment instructions.

---

## Usage

### Launch full SLAM stack (real robot)

```bash
ros2 launch puzzlebot_slam slam_launch.py
```

### Launch with simulation (Isaac Sim must be running)

```bash
ros2 launch puzzlebot_slam slam_sim_launch.py
```

### Launch controller only

```bash
ros2 launch puzzlebot_control control_launch.py
```

