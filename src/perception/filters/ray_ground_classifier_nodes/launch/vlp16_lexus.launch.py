"""Launch a few nodes for easy debugging and testing of the kinematic tracker"""

import launch
import launch_ros.actions


def generate_launch_description():
  driver = launch_ros.actions.Node(
      package='velodyne_node', node_executable='velodyne_block_node_exe',
      arguments=['--node_name=vlp16_front'])
  classifier = launch_ros.actions.Node(
      package='ray_ground_classifier_nodes', node_executable='ray_ground_classifier_block_node_exe',
      arguments=['--node_name=ray_ground_classifier'])
  viz_front = launch_ros.actions.Node(
      package='autoware_rviz', node_executable='autoware_rviz',
      arguments=['--block_topic=points_ground', '--block_viz_topic=ground_block_viz'])
  viz_rear = launch_ros.actions.Node(
      package='autoware_rviz', node_executable='autoware_rviz',
      arguments=['--block_topic=points_nonground', '--block_viz_topic=nonground_block_viz'])
  return launch.LaunchDescription([driver, classifier, viz_front, viz_rear, console])

