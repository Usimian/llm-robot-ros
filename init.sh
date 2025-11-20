# These are necessary for my home setup
export HOME=/home/ros
source /home/ros/.bashrc

# Linking the ROS2 workspace
ln -s /home/ros/rap/Gruppe2/ /home/ros/colcon_ws/src/

# Link icclab_summit_xl from Gruppe2 to workspace
# The icclab_summit_xl directory is copied from the host into Gruppe2/ during Docker build
ICCLAB_SUMMIT_SRC="/home/ros/rap/Gruppe2/icclab_summit_xl"
ICCLAB_SUMMIT_DIR="/home/ros/colcon_ws/src/icclab_summit_xl"

if [ -d "$ICCLAB_SUMMIT_SRC" ]; then
  echo "Linking icclab_summit_xl from Gruppe2..."
  rm -rf "$ICCLAB_SUMMIT_DIR"
  ln -s "$ICCLAB_SUMMIT_SRC" "$ICCLAB_SUMMIT_DIR"
else
  # Fallback: clone from GitHub if not copied from host
  if [ ! -d "$ICCLAB_SUMMIT_DIR" ]; then
    echo "Cloning icclab_summit_xl from GitHub..."
    git clone -b jazzy https://github.com/icclab/icclab_summit_xl.git "$ICCLAB_SUMMIT_DIR"

    # Apply custom bridge configuration
    if [ -f "/home/ros/rap/Gruppe2/config/ign_gazebo_bridge.yaml" ]; then
      echo "Applying custom bridge configuration..."
      cp /home/ros/rap/Gruppe2/config/ign_gazebo_bridge.yaml "$ICCLAB_SUMMIT_DIR/icclab_summit_xl/config/ign_gazebo_bridge.yaml"
    fi

    # Apply nav2 configuration fixes
    echo "Applying nav2 configuration fixes..."
    sed -i 's/update_frequency: 5.0/update_frequency: 10.0/g' "$ICCLAB_SUMMIT_DIR/icclab_summit_xl/config/nav2_params_sim.yaml"
    sed -i 's/publish_frequency: 2.0/publish_frequency: 5.0/g' "$ICCLAB_SUMMIT_DIR/icclab_summit_xl/config/nav2_params_sim.yaml"
    sed -i 's/plugins: \["voxel_layer", "inflation_layer"\]/plugins: ["obstacle_layer", "inflation_layer"]/g' "$ICCLAB_SUMMIT_DIR/icclab_summit_xl/config/nav2_params_sim.yaml"
    sed -i '/voxel_layer:/,/raytrace_min_range: 0.0/c\      obstacle_layer:\n        plugin: "nav2_costmap_2d::ObstacleLayer"\n        enabled: True\n        footprint_clearing_enabled: true\n        max_obstacle_height: 2.0\n        observation_sources: scan\n        scan:\n          topic: "<robot_namespace>/scan"\n          max_obstacle_height: 2.0\n          obstacle_max_range: 3.0\n          obstacle_min_range: 0.0\n          raytrace_max_range: 4.0\n          raytrace_min_range: 0.0\n          clearing: True\n          marking: True\n          data_type: "LaserScan"' "$ICCLAB_SUMMIT_DIR/icclab_summit_xl/config/nav2_params_sim.yaml"
    sed -i 's/plugins: \["static_layer", "voxel_layer", "inflation_layer"\]/plugins: ["static_layer", "obstacle_layer", "inflation_layer"]/g' "$ICCLAB_SUMMIT_DIR/icclab_summit_xl/config/nav2_params_sim.yaml"
    sed -i 's/xy_goal_tolerance: 0.25/xy_goal_tolerance: 0.10/g' "$ICCLAB_SUMMIT_DIR/icclab_summit_xl/config/nav2_params_sim.yaml"
    sed -i 's/yaw_goal_tolerance: 0.25/yaw_goal_tolerance: 0.02/g' "$ICCLAB_SUMMIT_DIR/icclab_summit_xl/config/nav2_params_sim.yaml"
  else
    echo "icclab_summit_xl directory already exists."
  fi
fi

# Check and clone m-explore-ros2 if not present
EXPLORE_LITE_DIR="/home/ros/colcon_ws/src/m-explore-ros2"
if [ ! -d "$EXPLORE_LITE_DIR" ]; then
  echo "Cloning m-explore-ros2..."
  git clone https://github.com/robo-friends/m-explore-ros2.git "$EXPLORE_LITE_DIR"
else
  echo "m-explore-ros2 directory already exists."
fi

# The map_merge package is not needed for the current setup 
# and also causes issues with the current ros2 jazzy version
rm -rf /home/ros/colcon_ws/src/m-explore-ros2/map_merge/

# Build the workspace
cd /home/ros/colcon_ws
colcon build --symlink-install
source install/setup.bash
sudo apt update -y
rosdep install --from-paths src --ignore-src -r -y
cd -
echo "** ROS2 $ROS_DISTRO initialized with $RMW_IMPLEMENTATION**"

# Install Python packages
pip3 install jpl-rosa --break-system-packages
pip3 install langchain-ollama --upgrade --break-system-packages
pip3 install langchain-core --upgrade --break-system-packages
pip3 install pydantic --upgrade --break-system-packages
pip3 install anthropic --upgrade --break-system-packages
pip3 install langchain-anthropic --upgrade --break-system-packages

# gazebo models
export GZ_SIM_RESOURCE_PATH=/home/ros/rap/Gruppe2/world/models