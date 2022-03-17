# Copyright 2021 the Autoware Foundation
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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

"""Launch odom_to_state_conversion_node."""

import launch
import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    param_path = os.path.join(
        get_package_share_directory('odom_to_state_conversion_node'),
        'param',
        'odom_to_state_conversion.param.yaml')

    odom_to_state_node = Node(
        name='odom_to_state_conversion_node',
        namespace='vehicle',
        package='odom_to_state_conversion_nodes',
        executable='odom_to_state_conversion_nodes_exe',
        output='screen',
        parameters=[param_path]
    )

    return launch.LaunchDescription([odom_to_state_node])
