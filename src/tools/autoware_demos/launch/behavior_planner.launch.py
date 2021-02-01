# Copyright 2020-2021, The Autoware Foundation
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

"""Launch modules for behavior planner."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from pathlib import Path


def generate_launch_description():
    param_path = Path(get_package_share_directory('autoware_demos')) / 'param'

    urdf_pkg_prefix = Path(get_package_share_directory('lexus_rx_450h_description'))
    urdf_path = urdf_pkg_prefix / 'urdf' / 'lexus_rx_450h.urdf'

    # Nodes

    behavior_planner_param = DeclareLaunchArgument(
        'behavior_planner_param_file',
        default_value=str(param_path / 'behavior_planner.param.yaml'),
        description='Path to paramter file for behavior planner'
    )
    behavior_planner = Node(
        package='behavior_planner_nodes',
        node_name='behavior_planner_node',
        node_namespace='planning',
        node_executable='behavior_planner_node_exe',
        parameters=[LaunchConfiguration('behavior_planner_param_file')],
        output='screen',
        remappings=[
            ('HAD_Map_Service', '/had_maps/HAD_Map_Service'),
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('route', 'global_path'),
            ('vehicle_state_report', '/vehicle/state_report'),
            ('vehicle_state_command', '/vehicle/state_command')
        ]
    )

    global_planner = Node(
        package='lanelet2_global_planner_nodes',
        node_name='lanelet2_global_planner_node',
        node_namespace='planning',
        node_executable='lanelet2_global_planner_node_exe',
        remappings=[('HAD_Map_Client', '/had_maps/HAD_Map_Service'),
                    ('vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state')]
    )

    lane_planner_param = DeclareLaunchArgument(
        'lane_planner_param_file',
        default_value=str(param_path / 'lane_planner.param.yaml'),
        description='Path to parameter file for lane planner'
    )
    lane_planner = Node(
        package='lane_planner_nodes',
        node_name='lane_planner_node',
        node_namespace='planning',
        node_executable='lane_planner_node_exe',
        parameters=[LaunchConfiguration('lane_planner_param_file')],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )

    lanelet2_map_provider_param = DeclareLaunchArgument(
        'lanelet2_map_provider_param_file',
        default_value=str(param_path / 'lanelet2_map_provider.param.yaml'),
        description='Path to parameter file for Lanelet2 Map Provider'
    )
    lanelet2_map_provider = Node(
        package='lanelet2_map_provider',
        node_executable='lanelet2_map_provider_exe',
        node_namespace='had_maps',
        parameters=[LaunchConfiguration('lanelet2_map_provider_param_file')]
    )

    lanelet2_map_visualizer = Node(
        package='lanelet2_map_provider',
        node_executable='lanelet2_map_visualizer_exe',
        node_namespace='had_maps'
    )

    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=str(param_path / 'lgsvl_interface.param.yaml'),
        description='Path to config file for LGSVL Interface'
    )
    lgsvl_interface = Node(
        package='lgsvl_interface',
        node_executable='lgsvl_interface_exe',
        node_namespace='vehicle',
        output='screen',
        parameters=[
          LaunchConfiguration('lgsvl_interface_param_file'),
          {"lgsvl.publish_tf": True}  # publishes odom-base_link
        ],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "/vehicle/state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom")
        ]
    )

    map_odom_publisher = Node(
        package='tf2_ros',
        node_executable='static_transform_publisher',
        arguments=["-57.60810852050781", "-41.382755279541016", "0", "0", "0", "0", "map", "odom"]
    )

    map_publisher_param = DeclareLaunchArgument(
        'map_publisher_param_file',
        default_value=str(param_path / 'map_publisher.param.yaml'),
        description='Path to config file for Map Publisher'
    )
    map_publisher = Node(
        package='ndt_nodes',
        node_executable='ndt_map_publisher_exe',
        node_namespace='localization',
        parameters=[LaunchConfiguration('map_publisher_param_file')]
    )

    mpc_controller_param = DeclareLaunchArgument(
        'mpc_controller_param_file',
        default_value=str(param_path / 'mpc_controller.param.yaml'),
        description='Path to config file for MPC'
    )
    mpc_controller = Node(
        package='mpc_controller_nodes',
        node_executable='mpc_controller_node_exe',
        node_name='mpc_controller',
        node_namespace='control',
        parameters=[LaunchConfiguration('mpc_controller_param_file')]
    )

    object_collision_estimator_param = DeclareLaunchArgument(
        'object_collision_estimator_param_file',
        default_value=str(param_path / 'object_collision_estimator.param.yaml'),
        description='Path to paramter file for object collision estimator'
    )
    object_collision_estimator = Node(
        package='object_collision_estimator_nodes',
        node_name='object_collision_estimator_node',
        node_namespace='planning',
        node_executable='object_collision_estimator_node_exe',
        parameters=[LaunchConfiguration('object_collision_estimator_param_file')],
        remappings=[
            ('obstacle_topic', '/perception/lidar_bounding_boxes'),
        ]
    )

    parking_planner_param = DeclareLaunchArgument(
        'parking_planner_param_file',
        default_value=str(param_path / 'parking_planner.param.yaml'),
        description='Path to paramter file for parking planner'
    )
    parking_planner = Node(
        package='parking_planner_nodes',
        node_name='parking_planner_node',
        node_namespace='planning',
        node_executable='parking_planner_node_exe',
        parameters=[LaunchConfiguration('parking_planner_param_file')],
        remappings=[('HAD_Map_Service', '/had_maps/HAD_Map_Service')]
    )

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', str(param_path.parent / 'rviz2' / 'planner.rviz')],
        remappings=[("initialpose", "/localization/initialpose"),
                    ("goal_pose", "/planning/goal_pose")],
    )

    urdf_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        arguments=[str(urdf_path)]
    )

    return LaunchDescription([
        behavior_planner_param,
        behavior_planner,
        global_planner,
        lane_planner_param,
        lane_planner,
        lanelet2_map_provider_param,
        lanelet2_map_provider,
        lanelet2_map_visualizer,
        lgsvl_interface_param,
        lgsvl_interface,
        map_odom_publisher,
        map_publisher_param,
        map_publisher,
        mpc_controller_param,
        mpc_controller,
        object_collision_estimator_param,
        object_collision_estimator,
        parking_planner_param,
        parking_planner,
        rviz2,
        urdf_publisher,
    ])
