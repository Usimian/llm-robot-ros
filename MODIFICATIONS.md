# Summit XL Modifications

This document describes how the icclab_summit_xl package is managed and modified in this project.

## Package Source

The `icclab_summit_xl` package is cloned from GitHub during Docker image build:
- **Repository**: https://github.com/icclab/icclab_summit_xl
- **Branch**: jazzy
- **Clone location**: `/home/ros/colcon_ws/src/icclab_summit_xl`

## How Modifications Work

The package is cloned and modified automatically in `init.sh`:

1. **Clone**: The script clones the icclab_summit_xl repository if not already present
2. **Apply custom configs**: Copies custom configurations from this repository
3. **Apply fixes**: Runs sed commands to fix navigation parameters

## Custom Modifications Applied

### 1. Gazebo Bridge Configuration
**File**: `config/ign_gazebo_bridge.yaml`
- Removed unused arm_camera topics (since Luxonis camera was removed)
- Topics removed: color/image, color/camera_info, depth/image, depth/camera_info, points

### 2. Navigation Parameters
**File**: `icclab_summit_xl/config/nav2_params_sim.yaml`
- Increased costmap update frequency: 5.0 → 10.0 Hz
- Increased costmap publish frequency: 2.0 → 5.0 Hz
- Replaced voxel_layer with obstacle_layer (more stable)
- Tightened goal tolerances:
  - xy_goal_tolerance: 0.25 → 0.10 m
  - yaw_goal_tolerance: 0.25 → 0.02 rad

## Making Further Modifications

Since the package is now cloned as a proper git repository, you can:

1. **Modify directly**: Edit files in `/home/ros/colcon_ws/src/icclab_summit_xl` inside the container
2. **Persist changes**: Add your modifications to `init.sh` to apply them automatically
3. **Create a fork**: Fork the upstream repository and update `init.sh` to clone from your fork
4. **Local development**: Clone the repo locally and mount it as a volume in Docker

## Files Modified

- `robots/summit_xl_no_arm.urdf.xacro` - Robot description without arm/gripper, Intel RealSense D435i camera
- `config/ign_gazebo_bridge.yaml` - Custom bridge configuration without arm_camera topics

## Commit History

- Remove arm and gripper from Summit XL robot
- Replace Orbbec Astra with Intel RealSense D435i and remove Luxonis camera
- Remove unused arm_camera topics from Gazebo bridge configuration
- Refactor: Clone icclab_summit_xl from GitHub for easier modifications
