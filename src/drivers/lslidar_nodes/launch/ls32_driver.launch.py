# Copyright 2020 the Autoware Foundation
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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

"""Launch LS32 Driver Node."""

import os

from ament_index_python import get_package_share_directory
import launch.substitutions
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch LS32 Driver Node."""
    ls32_param_file = os.path.join(
        get_package_share_directory('lslidar_nodes'),
        'param/ls32_test.param.yaml')

    ls32_node_param = DeclareLaunchArgument(
        'ls32_node_param_file',
        default_value=ls32_param_file,
        description='Path to config file fo the ls32 node.'
    )

    ls32_node = launch_ros.actions.Node(
        package='lslidar_nodes',
        namespace="lidar_front",
        executable='lslidar_cloud_node_exe',
        parameters=[LaunchConfiguration('ls32_node_param_file')],
        remappings=[("topic", "points_xyzi")],
        arguments=["--model", "ls32"])

    return launch.LaunchDescription([
        ls32_node_param,
        ls32_node])
