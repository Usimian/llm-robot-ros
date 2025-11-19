#!/bin/bash

# Setup X11 for Docker
XSOCK=/tmp/.X11-unix
XAUTH=/tmp/.docker.xauth

# Create xauth file if it doesn't exist
if [ ! -f $XAUTH ]; then
    touch $XAUTH
    xauth nlist $DISPLAY | sed -e 's/^..../ffff/' | xauth -f $XAUTH nmerge -
fi

# Allow X server access
xhost +local:docker > /dev/null 2>&1

# Stop and remove existing container if it exists
docker stop robotcontrol > /dev/null 2>&1
docker rm robotcontrol > /dev/null 2>&1

# Run container with proper X11 and GPU support
docker run -d \
    --name robotcontrol \
    --gpus all \
    --device /dev/dri \
    --network host \
    -v $XSOCK:$XSOCK:rw \
    -v $XAUTH:$XAUTH:rw \
    -e DISPLAY=$DISPLAY \
    -e XAUTHORITY=$XAUTH \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp/runtime-ros \
    --add-host host.docker.internal:host-gateway \
    --privileged \
    llm-robot-ros:latest \
    bash -c "mkdir -p /tmp/runtime-ros && __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch rosa_summit summit.launch.py slam:=True"

# Wait a moment for container to start
sleep 2

# Check if container is running
if docker ps | grep -q robotcontrol; then
    echo "✓ Container 'robotcontrol' started successfully"
    echo "✓ Gazebo and RViz2 should appear shortly"
    echo ""
    echo "To view logs: docker logs -f robotcontrol"
    echo "To stop: docker stop robotcontrol"
else
    echo "✗ Failed to start container"
    echo "Check logs: docker logs robotcontrol"
    exit 1
fi
