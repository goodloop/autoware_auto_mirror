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
    """Launch the autoware_state_monitor node."""
    autoware_state_monitor_pkg_prefix = get_package_share_directory('autoware_state_monitor')

    autoware_state_monitor_param_file = os.path.join(
        autoware_state_monitor_pkg_prefix, 'config/autoware_state_monitor.param.yaml')

    # Arguments
    autoware_state_monitor_param = DeclareLaunchArgument(
        'autoware_state_monitor_param_file',
        default_value=autoware_state_monitor_param_file,
        description='Path to config file for Autoware State Monitor'
    )

    # Nodes
    autoware_state_monitor = Node(
        package='autoware_state_monitor',
        executable='autoware_state_monitor_exe',
        namespace='system',
        parameters=[LaunchConfiguration('autoware_state_monitor_param_file')],
        remappings=[
            ('input/autoware_engage', '/vehicle/engage'),
            ('input/vehicle_state_report', '/vehicle/state_report'),
            ('input/is_emergency', '/vehicle/emergency/is_emergency'),
            ('input/odometry', '/vehicle/odometry'),
            ('input/route', '/planning/global_path'),
            ('output/autoware_state', '/autoware/state'),
            ('output/autoware_engage', '/vehicle/engage')
        ]
    )

    return LaunchDescription([
        autoware_state_monitor_param,
        autoware_state_monitor
    ])
