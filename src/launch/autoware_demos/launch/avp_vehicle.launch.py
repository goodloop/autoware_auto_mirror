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

"""Launch Modules for Milestone 3 of the AVP 2020 Demo."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """
    Launch all nodes defined in the architecture for Milestone 3 of the AVP 2020 Demo.

    More details about what is included can
    be found at https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/milestones/25.
    """
    avp_demo_pkg_prefix = get_package_share_directory('autoware_demos')

    map_publisher_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/map_publisher_vehicle.param.yaml')
    ndt_localizer_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/ndt_localizer_vehicle.param.yaml')
    mpc_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/mpc_vehicle.param.yaml')
    vlp16_front_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/vlp16_front_vehicle.param.yaml')
    vlp16_rear_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/vlp16_rear_vehicle.param.yaml')
    ssc_interface_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/ssc_interface.param.yaml')

    pc_filter_transform_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/pc_filter_transform.param.yaml')

    vehicle_characteristics_param_file = os.path.join(
        avp_demo_pkg_prefix, 'param/avp/vehicle_characteristics_vehicle.param.yaml')

    urdf_pkg_prefix = get_package_share_directory('lexus_rx_450h_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/lexus_rx_450h_vehicle.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()

    map_pcd_file = os.path.join(
        avp_demo_pkg_prefix, 'data/autonomoustuff_parking_lot.pcd')
    map_yaml_file = os.path.join(
        avp_demo_pkg_prefix, 'data/autonomoustuff_parking_lot.yaml')

    # Arguments

    with_lidars_param = DeclareLaunchArgument(
        'with_lidars',
        default_value='True',
        description='Launch lidar drivers in addition to other nodes'
    )
    vlp16_front_param = DeclareLaunchArgument(
        'vlp16_front_param_file',
        default_value=vlp16_front_param_file,
        description='Path to config file for front Velodyne'
    )
    vlp16_rear_param = DeclareLaunchArgument(
        'vlp16_rear_param_file',
        default_value=vlp16_rear_param_file,
        description='Path to config file for rear Velodyne'
    )
    map_publisher_param = DeclareLaunchArgument(
        'map_publisher_param_file',
        default_value=map_publisher_param_file,
        description='Path to config file for Map Publisher'
    )
    ndt_localizer_param = DeclareLaunchArgument(
        'ndt_localizer_param_file',
        default_value=ndt_localizer_param_file,
        description='Path to config file for ndt localizer'
    )
    mpc_param = DeclareLaunchArgument(
        'mpc_param_file',
        default_value=mpc_param_file,
        description='Path to config file for MPC'
    )
    ssc_interface_param = DeclareLaunchArgument(
        'ssc_interface_param_file',
        default_value=ssc_interface_param_file,
        description='Path to config file for SSC interface'
    )
    pc_filter_transform_param = DeclareLaunchArgument(
        'pc_filter_transform_param_file',
        default_value=pc_filter_transform_param_file,
        description='Path to config file for Point Cloud Filter/Transform Nodes'
    )
    vehicle_characteristics_param = DeclareLaunchArgument(
        'vehicle_characteristics_param_file',
        default_value=vehicle_characteristics_param_file,
        description='Path to config file for vehicle characteristics'
    )

    # Nodes

    vlp16_front = Node(
        package='velodyne_nodes',
        executable='velodyne_cloud_node_exe',
        namespace='lidar_front',
        parameters=[LaunchConfiguration('vlp16_front_param_file')],
        condition=IfCondition(LaunchConfiguration('with_lidars'))
    )
    vlp16_rear = Node(
        package='velodyne_nodes',
        executable='velodyne_cloud_node_exe',
        namespace='lidar_rear',
        parameters=[LaunchConfiguration('vlp16_rear_param_file')],
        condition=IfCondition(LaunchConfiguration('with_lidars'))
    )
    filter_transform_vlp16_front = Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_vlp16_front',
        namespace='lidar_front',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        remappings=[("points_in", "points_xyzi")]
    )
    filter_transform_vlp16_rear = Node(
        package='point_cloud_filter_transform_nodes',
        executable='point_cloud_filter_transform_node_exe',
        name='filter_transform_vlp16_rear',
        namespace='lidar_rear',
        parameters=[LaunchConfiguration('pc_filter_transform_param_file')],
        remappings=[("points_in", "points_xyzi")]
    )
    map_publisher = Node(
        package='ndt_nodes',
        executable='ndt_map_publisher_exe',
        namespace='localization',
        parameters=[LaunchConfiguration('map_publisher_param_file'),
                    {"map_pcd_file": map_pcd_file,
                     "map_yaml_file": map_yaml_file}]
    )
    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )
    ndt_localizer = Node(
        package='ndt_nodes',
        executable='p2d_ndt_localizer_exe',
        namespace='localization',
        name='p2d_ndt_localizer_node',
        parameters=[LaunchConfiguration('ndt_localizer_param_file')],
        remappings=[
            ("points_in", "/lidars/points_fused_downsampled")
        ]
    )
    mpc = Node(
        package='mpc_controller_nodes',
        executable='mpc_controller_node_exe',
        name='mpc_controller',
        namespace='control',
        parameters=[
            LaunchConfiguration('mpc_param_file'),
            LaunchConfiguration('vehicle_characteristics_param_file'),
        ],
    )
    ssc_interface = Node(
        package='ssc_interface',
        executable='ssc_interface_node_exe',
        name='ssc_interface',
        namespace='vehicle',
        parameters=[LaunchConfiguration('ssc_interface_param_file')],
        remappings=[
            ('gear_select', '/ssc/gear_select'),
            ('arbitrated_speed_commands', '/ssc/arbitrated_speed_commands'),
            ('arbitrated_steering_commands', '/ssc/arbitrated_steering_commands'),
            ('turn_signal_command', '/ssc/turn_signal_command'),
            ('dbw_enabled_feedback', '/ssc/dbw_enabled_feedback'),
            ('gear_feedback', '/ssc/gear_feedback'),
            ('velocity_accel_cov', '/ssc/velocity_accel_cov'),
            ('state_report_out', 'state_report'),
            ('steering_feedback', '/ssc/steering_feedback'),
            ('vehicle_kinematic_state_cog', 'vehicle_kinematic_state')
        ]
    )

    # TODO(nikolai.morin): Hack, to be resolved in #626
    odom_bl_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"]
    )

    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([avp_demo_pkg_prefix, '/launch/avp_core.launch.py']),
        launch_arguments={}.items()
    )

    return LaunchDescription([
        with_lidars_param,
        vlp16_front_param,
        vlp16_rear_param,
        map_publisher_param,
        ndt_localizer_param,
        mpc_param,
        ssc_interface_param,
        pc_filter_transform_param,
        vehicle_characteristics_param,
        vlp16_front,
        vlp16_rear,
        filter_transform_vlp16_front,
        filter_transform_vlp16_rear,
        urdf_publisher,
        map_publisher,
        ndt_localizer,
        mpc,
        ssc_interface,
        odom_bl_publisher,
        core_launch
    ])
