#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

xhost +local:docker > /dev/null 2>&1

if [ "$(docker inspect -f '{{.State.Running}}' puzzlebot_cuda 2>/dev/null)" != "true" ]; then
    docker compose -f "$SCRIPT_DIR/docker-compose.yml" up -d --remove-orphans puzzlebot_cuda
fi

docker compose -f "$SCRIPT_DIR/docker-compose.yml" exec puzzlebot_cuda bash