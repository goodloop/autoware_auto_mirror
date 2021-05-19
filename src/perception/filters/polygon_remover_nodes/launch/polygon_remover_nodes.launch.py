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

# import os

# from ament_index_python.packages import get_package_share_directory

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with a single component."""
    # TODO xmfcx: try to pass yaml file to a component within composion
    # polygon_remover_node_pkg_prefix = get_package_share_directory('polygon_remover_nodes')
    # polygon_remover_node_param_file = os.path.join(polygon_remover_node_pkg_prefix,
    #                                                'param/test_params.yaml')

    container = ComposableNodeContainer(
        name='polygon_remover_node_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='polygon_remover_nodes',
                plugin='autoware::perception::filters::polygon_remover_nodes::PolygonRemoverNode',
                name='polygon_remover_node',
                # parameters=[polygon_remover_node_param_file]),
                parameters=[{'topic_name_cloud_sub': '/lidar_front/points_raw'},
                            {'topic_name_cloud_pub': '/cloud_polygon_removed'},
                            {'working_mode': 'Static'},
                            {'polygon_vertices': [0.0, -23.916,
                                                  0.21031, -10.228,
                                                  23.8108, -6.61647,
                                                  10.8577, -2.18663,
                                                  14.7159, 21.3748,
                                                  6.50012, 10.4246,
                                                  -14.7159, 21.3748,
                                                  -6.8404, 10.1773,
                                                  -23.8108, -6.61647,
                                                  -10.7277, -2.58666]},
                            {'will_visualize': True},
                            {'topic_name_polygon_sub': '/marker_polygon_remover'}]),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container])
