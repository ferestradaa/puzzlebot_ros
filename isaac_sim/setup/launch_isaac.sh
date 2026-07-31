#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/isaac_env.sh"

SIM_SCRIPT="$SCRIPT_DIR/../simulation/simulation_std/main.py"

if [ ! -d "$ISAAC_SIM_PATH" ]; then
    echo "Isaac Sim not found at $ISAAC_SIM_PATH"
    echo "Edit isaac_sim/setup/isaac_env.sh and set ISAAC_SIM_PATH to your install path plisssss"
    exit 1
fi

if [ ! -f "$SIM_SCRIPT" ]; then
    echo "Simulation entry point not found at $SIM_SCRIPT"
    exit 1
fi

BRIDGE_LIBS="$ISAAC_SIM_PATH/exts/isaacsim.ros2.bridge/humble/lib"

if [ ! -d "$BRIDGE_LIBS" ]; then
    echo "ROS2 bridge libs not found at $BRIDGE_LIBS"
    echo "Check your Isaac Sim installation"

    exit 1
fi

env -i \
    HOME="$HOME" \
    USER="$USER" \
    PATH="$PATH" \
    DISPLAY="$DISPLAY" \
    XAUTHORITY="${XAUTHORITY:-}" \
    ROS_DISTRO=humble \
    RMW_IMPLEMENTATION="$RMW_IMPLEMENTATION" \
    ROS_DOMAIN_ID="$ROS_DOMAIN_ID" \
    LD_LIBRARY_PATH="$BRIDGE_LIBS" \
    bash "$ISAAC_SIM_PATH/python.sh" "$SIM_SCRIPT" "$@"