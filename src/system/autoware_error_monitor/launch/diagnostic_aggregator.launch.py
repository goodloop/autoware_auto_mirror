# Copyright 2021 Robotec.ai
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch the autoware_error_monitor node and diagnostic_aggregator node."""
    autoware_error_monitor_pkg_prefix = get_package_share_directory(
        'autoware_error_monitor')

    diagnostic_aggregator_discard_param_file = os.path.join(
        autoware_error_monitor_pkg_prefix, 'config/diagnostic_aggregator/_discard.param.yaml')
    diagnostic_aggregator_control_param_file = os.path.join(
        autoware_error_monitor_pkg_prefix, 'config/diagnostic_aggregator/control.param.yaml')
    diagnostic_aggregator_localization_param_file = os.path.join(
        autoware_error_monitor_pkg_prefix, 'config/diagnostic_aggregator/localization.param.yaml')
    diagnostic_aggregator_map_param_file = os.path.join(
        autoware_error_monitor_pkg_prefix, 'config/diagnostic_aggregator/map.param.yaml')
    diagnostic_aggregator_perception_param_file = os.path.join(
        autoware_error_monitor_pkg_prefix, 'config/diagnostic_aggregator/perception.param.yaml')
    diagnostic_aggregator_planning_param_file = os.path.join(
        autoware_error_monitor_pkg_prefix, 'config/diagnostic_aggregator/planning.param.yaml')
    diagnostic_aggregator_sensing_param_file = os.path.join(
        autoware_error_monitor_pkg_prefix, 'config/diagnostic_aggregator/sensing.param.yaml')
    diagnostic_aggregator_system_param_file = os.path.join(
        autoware_error_monitor_pkg_prefix, 'config/diagnostic_aggregator/system.param.yaml')
    diagnostic_aggregator_vehicle_param_file = os.path.join(
        autoware_error_monitor_pkg_prefix, 'config/diagnostic_aggregator/vehicle.param.yaml')

    # Arguments
    diagnostic_aggregator_discard_param = DeclareLaunchArgument(
        'diagnostic_aggregator_discard_param_file',
        default_value=diagnostic_aggregator_discard_param_file,
    )
    diagnostic_aggregator_control_param = DeclareLaunchArgument(
        'diagnostic_aggregator_control_param_file',
        default_value=diagnostic_aggregator_control_param_file,
    )
    diagnostic_aggregator_localization_param = DeclareLaunchArgument(
        'diagnostic_aggregator_localization_param_file',
        default_value=diagnostic_aggregator_localization_param_file,
    )
    diagnostic_aggregator_map_param = DeclareLaunchArgument(
        'diagnostic_aggregator_map_param_file',
        default_value=diagnostic_aggregator_map_param_file,
    )
    diagnostic_aggregator_perception_param = DeclareLaunchArgument(
        'diagnostic_aggregator_perception_param_file',
        default_value=diagnostic_aggregator_perception_param_file,
    )
    diagnostic_aggregator_planning_param = DeclareLaunchArgument(
        'diagnostic_aggregator_planning_param_file',
        default_value=diagnostic_aggregator_planning_param_file,
    )
    diagnostic_aggregator_sensing_param = DeclareLaunchArgument(
        'diagnostic_aggregator_sensing_param_file',
        default_value=diagnostic_aggregator_sensing_param_file,
    )
    diagnostic_aggregator_system_param = DeclareLaunchArgument(
        'diagnostic_aggregator_system_param_file',
        default_value=diagnostic_aggregator_system_param_file,
    )
    diagnostic_aggregator_vehicle_param = DeclareLaunchArgument(
        'diagnostic_aggregator_vehicle_param_file',
        default_value=diagnostic_aggregator_vehicle_param_file,
    )

    # Nodes
    diagnostic_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        # namespace='/',
        parameters=[{
            "pub_rate": 10.0,
            "path": "autoware"
            },
            LaunchConfiguration('diagnostic_aggregator_discard_param_file'),
            LaunchConfiguration('diagnostic_aggregator_control_param_file'),
            LaunchConfiguration('diagnostic_aggregator_localization_param_file'),
            LaunchConfiguration('diagnostic_aggregator_map_param_file'),
            LaunchConfiguration('diagnostic_aggregator_perception_param_file'),
            LaunchConfiguration('diagnostic_aggregator_planning_param_file'),
            LaunchConfiguration('diagnostic_aggregator_sensing_param_file'),
            LaunchConfiguration('diagnostic_aggregator_system_param_file'),
            LaunchConfiguration('diagnostic_aggregator_vehicle_param_file')
        ],
        remappings=[
            ('input/diag_array', '/diagnostics_agg'),
            ('output/driving_capability', '/vehicle/driving_capability')
        ]
    )

    return LaunchDescription([
        diagnostic_aggregator_discard_param,
        diagnostic_aggregator_control_param,
        diagnostic_aggregator_localization_param,
        diagnostic_aggregator_map_param,
        diagnostic_aggregator_perception_param,
        diagnostic_aggregator_planning_param,
        diagnostic_aggregator_sensing_param,
        diagnostic_aggregator_vehicle_param,
        diagnostic_aggregator_system_param,
        diagnostic_aggregator
    ])
