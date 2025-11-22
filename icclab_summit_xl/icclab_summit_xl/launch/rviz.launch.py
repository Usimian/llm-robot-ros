import launch
import launch_ros
import os
from launch_ros.actions import Node
from launch.actions import OpaqueFunction
from ament_index_python.packages import get_package_share_directory

def launch_rviz(context, *args, **kwargs):
  rviz_config = context.launch_configurations.get('rviz_config', 'robot.rviz')

  # Use persistent RViz config location if it exists, otherwise fall back to package config
  persistent_rviz_config = '/home/ros/.rviz2/default.rviz'

  if os.path.exists(persistent_rviz_config):
    config_to_use = persistent_rviz_config
  else:
    config_to_use = os.path.join(get_package_share_directory('icclab_summit_xl'), 'rviz', rviz_config)

  return [Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', config_to_use, '--ros-args', '--log-level', 'INFO'],
    output='screen')]

def generate_launch_description():

  ld = launch.LaunchDescription()

  ld.add_action(launch.actions.DeclareLaunchArgument(
    name='rviz_config',
    description='Rviz configuration file to use (from icclab_summit_xl/rviz dir)',
    default_value='robot.rviz',
  ))

  # Launch rviz with runtime config check
  ld.add_action(OpaqueFunction(function=launch_rviz))

  return ld
