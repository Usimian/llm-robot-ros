FROM robopaas/rap-jazzy:cuda12.5.0

# Copy local repository with all fixes
COPY --chown=ros:ros . /home/ros/rap/Gruppe2
RUN bash -c "source ~/rap/Gruppe2/init.sh"

# Fix LangChain dependencies - install versions compatible with jpl-rosa 1.0.8
RUN pip3 install 'langchain~=0.3.23' 'langchain-community~=0.3.21' 'langchain-core~=0.3.52' \
    'langchain-ollama~=0.3.2' 'langchain-openai~=0.3.14' langchain-anthropic --upgrade --break-system-packages

# Fix rosa_summit.py import for tool decorator compatibility (if not already fixed in fork)
RUN sed -i 's/from langchain.agents import tool/from langchain_core.tools import tool/' \
    /home/ros/rap/Gruppe2/rosa_summit/rosa_summit.py || true

ENV GZ_SIM_RESOURCE_PATH=/home/ros/rap/Gruppe2/world/models

# Build workspace
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && cd ~/colcon_ws && colcon build"

# Create entrypoint script
RUN sudo bash -c "touch /ros_entrypoint.sh && chown $USER /ros_entrypoint.sh && chmod +x /ros_entrypoint.sh"
RUN cat > /ros_entrypoint.sh <<'EOF'
#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/home/ros/colcon_ws/install/setup.bash"

exec "$@"
EOF

ENTRYPOINT ["/ros_entrypoint.sh"]
