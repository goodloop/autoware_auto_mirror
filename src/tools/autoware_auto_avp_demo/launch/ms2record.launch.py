# Copyright 2020, The Autoware Foundation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Modules for Milestone 2 of the AVP 2020 Demo."""

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackage
from pathlib import Path

import os

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory as get_package_share_directory_orig


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory_orig(package_name), file_name)


context = LaunchContext()


def get_package_share_directory(package_name):
    """Return the absolute path to the share directory of the given package."""
    return os.path.join(Path(FindPackage(package_name).perform(context)), 'share', package_name)


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Milestone 2 of the AVP 2020 Demo.

    More details about what is included can
    be found at https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones/24.
    """
    avp_demo_pkg_prefix = get_package_share_directory('autoware_auto_avp_demo')
    euclidean_cluster_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/euclidean_cluster.param.yaml')
    lgsvl_param_file = get_share_file(
        package_name='test_trajectory_following', file_name='lgsvl_interface.param.yaml')
    ray_ground_classifier_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/ray_ground_classifier.param.yaml')
    rviz_cfg_path = os.path.join(avp_demo_pkg_prefix, 'config/ms2.rviz')

    pc_filter_transform_pkg_prefix = get_package_share_directory(
        'point_cloud_filter_transform_nodes')
    pc_filter_transform_param_file = os.path.join(
        pc_filter_transform_pkg_prefix, 'param/vlp16_sim_lexus_filter_transform.param.yaml')

    joy_translator_param_file = get_share_file(
        package_name='test_trajectory_following', file_name='logitech_f310.param.yaml')

    urdf_pkg_prefix = get_package_share_directory('lexus_rx_450h_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/lexus_rx_450h.urdf')

    # Arguments

    euclidean_cluster_param = DeclareLaunchArgument(
        'euclidean_cluster_param_file',
        default_value=euclidean_cluster_param_file,
        description='Path to config file for Euclidean Clustering'
    )
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=lgsvl_param_file,
        description='Path to config file for LGSVL Interface'
    )
    pc_filter_transform_param = DeclareLaunchArgument(
        'pc_filter_transform_param_file',
        default_value=pc_filter_transform_param_file,
        description='Path to config file for Point Cloud Filter/Transform Nodes'
    )
    ray_ground_classifier_param = DeclareLaunchArgument(
        'ray_ground_classifier_param_file',
        default_value=ray_ground_classifier_param_file,
        description='Path to config file for Ray Ground Classifier'
    )
    with_rviz_param = DeclareLaunchArgument(
        'with_rviz',
        default_value='True',
        description='Launch RVIZ2 in addition to other nodes'
    )

    joy_translator_param = DeclareLaunchArgument(
        'joy_translator_param_file',
        default_value=joy_translator_param_file,
        description='Path to config file for joystick translator'
    )

    # Nodes
    joystick_launch_file_path = get_share_file(
        package_name='joystick_vehicle_interface',
        file_name='joystick_vehicle_interface.launch.py')
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(joystick_launch_file_path),
        launch_arguments=[
            ("joy_translator_param", LaunchConfiguration('joy_translator_param_file'))]
    )

    recordreplay_planner_path = get_share_file(
        package_name='recordreplay_planner_node',
        file_name='recordreplay_planner_node.launch.py')
    recordreplay_planner_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(recordreplay_planner_path)
    )

    joy_ctrl_record_replay_traj = Node(
        package="test_trajectory_following",
        node_executable="joy_ctrl_record_replay_traj.py",
        node_name="joy_ctrl_record_replay_traj",
        parameters=[],
        remappings=[
            ("vehicle_kinematic_state", "/vehicle/vehicle_kinematic_state")
        ],
        output='screen',
    )
    euclidean_clustering = Node(
        package='euclidean_cluster_nodes',
        node_executable='euclidean_cluster_exe',
        node_namespace='perception',
        parameters=[LaunchConfiguration('euclidean_cluster_param_file')]
    )
    filter_transform_vlp16_front = Node(
        package='point_cloud_filter_transform_nodes',
        node_executable='point_cloud_filter_transform_node_exe',
        node_name='filter_transform_vlp16_front',
        node_namespace='lidar_front',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')]
    )
    filter_transform_vlp16_rear = Node(
        package='point_cloud_filter_transform_nodes',
        node_executable='point_cloud_filter_transform_node_exe',
        node_name='filter_transform_vlp16_rear',
        node_namespace='lidar_rear',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')]
    )
    lgsvl_interface = Node(
        package='lgsvl_interface',
        node_executable='lgsvl_interface_exe',
        node_namespace='vehicle',
        output='screen',
        parameters=[LaunchConfiguration('lgsvl_interface_param_file')]
    )

    ray_ground_classifier = Node(
        package='ray_ground_classifier_nodes',
        node_executable='ray_ground_classifier_cloud_node_exe',
        node_namespace='perception',
        parameters=[LaunchConfiguration('ray_ground_classifier_param_file')]
    )
    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )
    urdf_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        arguments=[str(urdf_path)]
    )

    return LaunchDescription([
        joy_translator_param,
        joystick,
        recordreplay_planner_node,
        joy_ctrl_record_replay_traj,
        euclidean_cluster_param,
        lgsvl_interface_param,
        pc_filter_transform_param,
        ray_ground_classifier_param,
        with_rviz_param,
        urdf_publisher,
        euclidean_clustering,
        filter_transform_vlp16_front,
        filter_transform_vlp16_rear,
        lgsvl_interface,
        ray_ground_classifier,
        rviz2
    ])
