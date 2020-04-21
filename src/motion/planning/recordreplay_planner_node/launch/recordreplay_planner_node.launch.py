# Copyright 2020 The Autoware Foundation.
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


import ament_index_python
import launch
import launch_ros.actions


def generate_launch_description():
    """Launch recordreplay_planner_node with default configuration."""
    # -------------------------------- Nodes-----------------------------------
    recordreplay_planner_node = launch_ros.actions.Node(
        package='recordreplay_planner_node',
        node_executable='recordreplay_planner_node_exe',
        node_name='recordreplay_planner',
        output='screen',
        parameters=[
            "{}/defaults.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "recordreplay_planner_node"
                )
            ),
        ],
        remappings=[
            ('vehicle_state', '/vehicle_kinematic_state'),
            ('planned_trajectory', '/trajectory'),
            ('obstacle_bounding_boxes', '/perception/lidar_bounding_boxes'),
        ]
    )

    ld = launch.LaunchDescription([
        recordreplay_planner_node]
    )
    return ld
