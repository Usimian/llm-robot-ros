#!/bin/bash
docker run -it --rm \
    --gpus all \
    --device /dev/dri \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $HOME/.gz:/home/ros/.gz \
    -v $HOME/.rviz2:/home/ros/.rviz2 \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e DISPLAY=$DISPLAY \
    -e XDG_RUNTIME_DIR=$XDG_RUNTIME_DIR \
    -e XAUTHORITY=$XAUTH \
    --name robotcontrol \
    --add-host host.docker.internal:host-gateway \
    llm-robot-ros:latest \
    bash -c "__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia ros2 launch rosa_summit summit.launch.py slam:=True"
