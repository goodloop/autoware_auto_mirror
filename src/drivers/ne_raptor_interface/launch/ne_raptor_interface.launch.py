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

"""Launch file for interface node between New Eagle Raptor DBW and Autoware.Auto."""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory


def generate_launch_description():
    """Generate launch description with a single component."""
    # ---------------- Params ----------------
    raptor_interface_params_file = LaunchConfiguration(
        "raptor_interface_params",
        default=[get_package_share_directory('ne_raptor_interface'),
                 '/param/defaults.param.yaml']
    )
    raptor_dbw_params_file = LaunchConfiguration(
        'raptor_dbw_params',
        default=[get_package_share_directory('raptor_dbw_can'),
                 '/launch/launch_params.yaml']
    )
    raptor_dbw_dbc_file = LaunchConfiguration(
        'raptor_dbw_dbc',
        default=[get_package_share_directory('raptor_dbw_can'),
                 '/launch/New_Eagle_DBW_3.3.542.dbc']
    )

    # ---------------- Nodes ----------------
    return LaunchDescription(
        [
            Node(
                package='ne_raptor_interface',
                executable='ne_raptor_interface_node',
                output='screen',
                namespace='vehicle',
                parameters=[raptor_interface_params_file],
            ),
            Node(
                package='raptor_dbw_can',
                executable='raptor_dbw_can_node',
                output='screen',
                namespace='raptor_dbw_interface',
                parameters=[raptor_dbw_dbc_file],
            ),
            Node(
                package='kvaser_interface',
                executable='kvaser_can_bridge',
                output='screen',
                namespace='',
                parameters=[raptor_dbw_params_file]),
        ])


generate_launch_description()
