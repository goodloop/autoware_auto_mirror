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

"""Launch VLP16 Driver Node."""

import os

from ament_index_python import get_package_share_directory
import launch.substitutions
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch VLP32c Driver Node."""
    vlp32c_param_file = os.path.join(
        get_package_share_directory('velodyne_nodes'),
        'param/vlp32c_test.param.yaml')

    vlp32c_node_param = DeclareLaunchArgument(
        'vlp32c_node_param_file',
        default_value=vlp32c_param_file,
        description='Path to config file fo the vlp32c node.'
    )

    vlp32c_node = launch_ros.actions.Node(
        package='velodyne_nodes',
        namespace="lidar_front",
        executable='velodyne_cloud_node_exe',
        parameters=[LaunchConfiguration('vlp32c_node_param_file')],
        remappings=[("topic", "points_xyzif")])

    return launch.LaunchDescription([
        vlp32c_node_param,
        vlp32c_node])
