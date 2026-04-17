# TE3003B: Integration of Robotics and Intelligent Systems

Fernando Estrada Silva A01736094 
---


## Overview

This workspace contains the full ROS2 navigation stack for Puzzlebot differential drive robot (NVIDIA Jetson Nano 2GB)

- **EKF-SLAM** with ArUco landmarks 
- **Occupancy grid mapping** from raw LiDAR scans
- **Differential drive odometry** 
- **Sim-to-real transfer** via Isaac Sim


---

## System Architecture


---

## Repository Structure

```
puzz_ws/
├── docker/             # Docker Compose for cloninng and testing this repo
│   ├── Dockerfile
│   └── docker-compose.yml
├── isaac_sim/          # Isaac Sim assets, USD scenes, OmniGraph configs
│   ├── assets/
│   └── worlds/
├── jetson/             # Jetson Nano deployment scripts and configs
│   ├── setup.sh
├── params/             # YAML parameter (empty now)

└── src/                # ROS2 source packages
    ├── puzzlebot_navigation/
    ├── puzzlebot_control/
    └── puzzlebot_description/
```



## Installation

### 1. Clone the repository

```bash
git clone https://github.com/ferestradaa/puzzlebot_ros.git
cd puzz_ws
```

## Docker Setup

The Docker environment provides a reproducible ROS2 Humble container with all dependencies.

```bash
cd docker/

# Build image
docker compose build

# Run container
docker compose up -d

# Attach to container
docker exec puzzlebot bash
```

## Isaac Sim Integration



### Running the simulation




## Jetson Nano Deployment



## Usage

### Puzzlebot Description

```bash
ros2 launch puzzlebot_description display.py
```

### Launch Odometry

```bash
ros2 run puzzlebot_control odometry
```

