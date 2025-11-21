cmake_minimum_required(VERSION 3.5)
project(icclab_summit_xl)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch worlds config maps robots urdf rviz meshes scripts
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/scan_frame_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install test scripts (commented out - test directory doesn't exist in jazzy branch)
# install(PROGRAMS
#   test/test_simulation_readiness.py
#   test/test_navigation.py
#   test/test_moveit_configurations.py
#   DESTINATION lib/${PROJECT_NAME}
# )

ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

# ## System dependencies are found with CMake's conventions
# # find_package(Boost REQUIRED COMPONENTS system)
find_package(ament_cmake REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometric_shapes REQUIRED)
  # pcl_ros
  # pcl_conversions
find_package(moveit_msgs)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



# Arm-related executables (commented out for armless configuration)
# add_executable(move_arm_to_pose src/move_arm_to_pose.cpp)


add_executable(odom_tf src/odom_tf_pub.cpp)
ament_target_dependencies(odom_tf rclcpp geometry_msgs nav_msgs tf2_ros)

install(TARGETS odom_tf
  DESTINATION lib/${PROJECT_NAME})



ament_export_dependencies(ros_gz_bridge)
ament_export_dependencies(ros_gz_sim)

ament_package()
