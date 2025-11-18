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

# Fix costmap configuration - replace voxel_layer with obstacle_layer
RUN sed -i 's/update_frequency: 5.0/update_frequency: 10.0/g' /home/ros/colcon_ws/src/icclab_summit_xl/icclab_summit_xl/config/nav2_params_sim.yaml && \
    sed -i 's/publish_frequency: 2.0/publish_frequency: 5.0/g' /home/ros/colcon_ws/src/icclab_summit_xl/icclab_summit_xl/config/nav2_params_sim.yaml && \
    sed -i 's/plugins: \["voxel_layer", "inflation_layer"\]/plugins: ["obstacle_layer", "inflation_layer"]/g' /home/ros/colcon_ws/src/icclab_summit_xl/icclab_summit_xl/config/nav2_params_sim.yaml && \
    sed -i '/voxel_layer:/,/raytrace_min_range: 0.0/c\      obstacle_layer:\n        plugin: "nav2_costmap_2d::ObstacleLayer"\n        enabled: True\n        footprint_clearing_enabled: true\n        max_obstacle_height: 2.0\n        observation_sources: scan\n        scan:\n          topic: "<robot_namespace>/scan"\n          max_obstacle_height: 2.0\n          obstacle_max_range: 3.0\n          obstacle_min_range: 0.0\n          raytrace_max_range: 4.0\n          raytrace_min_range: 0.0\n          clearing: True\n          marking: True\n          data_type: "LaserScan"' /home/ros/colcon_ws/src/icclab_summit_xl/icclab_summit_xl/config/nav2_params_sim.yaml && \
    sed -i 's/plugins: \["static_layer", "voxel_layer", "inflation_layer"\]/plugins: ["static_layer", "obstacle_layer", "inflation_layer"]/g' /home/ros/colcon_ws/src/icclab_summit_xl/icclab_summit_xl/config/nav2_params_sim.yaml && \
    sed -i 's/xy_goal_tolerance: 0.25/xy_goal_tolerance: 0.10/g' /home/ros/colcon_ws/src/icclab_summit_xl/icclab_summit_xl/config/nav2_params_sim.yaml && \
    sed -i 's/yaw_goal_tolerance: 0.25/yaw_goal_tolerance: 0.02/g' /home/ros/colcon_ws/src/icclab_summit_xl/icclab_summit_xl/config/nav2_params_sim.yaml

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
