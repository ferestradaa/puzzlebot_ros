# puzzlebot_ros

ROS2 stack for autonomous navigation and pallet lifting with a forklift attachment on the [Manchester Robotics Puzzlebot](https://manchester-robotics.com/puzzlebot) differential-drive platform, with a matching Isaac Sim counterpart. Both real and simulated robots run the same navigation, control, and behavior stack, built from scratch without Nav2.

## Hardware

- Puzzlebot (differential drive)
- Jetson Nano 2GB for onboard compute

## Software

- ROS2 Humble
- Isaac Sim 5.1

## Capabilities

- `puzzlebot_navigation` — A* global path planning over an occupancy grid 
- `puzzlebot_control` — EKF localization, Pure Pursuit path tracking, controller mux, visual servoing, reactive obstacle avoidance (VFH-based)
- `puzzlebot_inference` — Pallet detection with ONNX/TensorRT models (CPU, GPU, and sim variants)
- `puzzlebot_behavior` — behavior orchestration via BehaviorTree.CPP
- `puzzlebot_forklift` — forklift mast SPI control
- `puzzlebot_vision` — vision pipeline
- `puzzlebot_description` — URDF/robot description
- `puzzlebot_bringup` — launch files for real and simulated hardware
- `isaac_sim/` — Isaac Sim counterpart (assets, worlds, simulation scripts)
- `jetson/` — Jetson Nano setup and deployment
- `docker/` — containers for host (simulation)

## Installation

Install Isaac Sim 5.1 [here](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html)

## Running

### Simulation

```
cd isaac_sim/simulation/simulation_std/
<path to your Isaac Sim 5.1 installation>/python.sh main.py
```

### Docker (host, GPU)

Create the container to run the ROS2 stack for simulation: 

```
cd docker
docker compose build puzzlebot_cuda
docker compose up puzzlebot_cuda -d
docker compose exec puzzlebot_cuda bash
```

Inside the container:

```
colcon build
source install/setup.bash
```

Then launch:


Simulated Stack:
```
ros2 launch puzzlebot_bringup sim_hardware.launch.py
```
