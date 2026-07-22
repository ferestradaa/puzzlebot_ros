# puzzlebot_ros

ROS2 stack for the [Manchester Robotics Puzzlebot](https://manchester-robotics.com/puzzlebot) differential drive platform, with a matching Isaac Sim counterpart. Both real and simulated robots run the same navigation, control, and behavior stack, built from scratch without Nav2.

## Hardware

- Puzzlebot (differential drive)
- Jetson Nano 2GB for onboard compute

## Software

- ROS2 Humble
- Isaac Sim 5.1

## Capabilities

- EKF localization fused with AprilTag detections
- A* global path planning over an occupancy grid, with dynamic map updates
- Pure Pursuit and Stanley path tracking controllers
- Reactive obstacle avoidance (VFH-based local controller)
- Behavior orchestration via BehaviorTree.CPP
- QR-based docking and visual servoing
- Pallet detection with ONNX models (CPU, GPU, and sim variants)
- Same stack runs unmodified on hardware and in the Isaac Sim counterpart

## Structure

- `src/puzzlebot_navigation` — path planning, occupancy grid, local
- `src/puzzlebot_control` — odometry, pure pursuit, controller mux, visual servoing, reactive controller
- `src/puzzlebot_inference` — ONNX runtime and Tensor RT inference for pallet detection models
- `src/puzzlebot_behavior` — behavior trees and BT node implementations
- `src/puzzlebot_forklift` — forklift mast SPI control
- `src/puzzlebot_vision` — vision pipeline
- `src/puzzlebot_description` — URDF/robot description
- `src/puzzlebot_bringup` — launch files for real and simulated hardware
- `isaac_sim/` — simulated counterpart (assets, worlds, simulation scripts)
- `jetson/` — Jetson Nano setup and deployment
- `docker/` — containers for host and Jetson


## Installation

Install Isaac Sim 5.1 [here](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html)

## Running

### Simulation

```
cd isaac_sim/simulation/simulation_std/
<path to your Isaac Sim 5.1 installation>/python.sh main.py
```

### Docker (host, GPU)

Create the container to run the ros2 stack for simulation: 

```
cd docker
docker compose build puzzlebot_cuda
docker compose up puzzlebot_cuda -d
docker compose exec puzzlebot_cuda bash
```

Inside the container:

```
colcon build
```

Then launch:


Simulated Stack:
```
ros2 launch puzzlebot_bringup sim_hardware.launch.py
```
