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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container_front = ComposableNodeContainer(
        name='lgsvl_cloud_converter_container_front',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='lgsvl_cloud_converter',
                plugin='autoware::drivers::lgsvl_cloud_converter::LgsvlCloudConverterNode',
                name='lgsvl_cloud_converter_node_front',
                parameters=[{"name_topic_cloud_in": "/lgsvl/lidar_front/points_raw"},
                            {"name_topic_cloud_out": "/lidar_front/points_raw"}], ),
        ],
        output='screen',
    )
    container_rear = ComposableNodeContainer(
        name='lgsvl_cloud_converter_container_rear',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='lgsvl_cloud_converter',
                plugin='autoware::drivers::lgsvl_cloud_converter::LgsvlCloudConverterNode',
                name='lgsvl_cloud_converter_node_rear',
                parameters=[{"name_topic_cloud_in": "/lgsvl/lidar_rear/points_raw"},
                            {"name_topic_cloud_out": "/lidar_rear/points_raw"}], ),
        ],
        output='screen',
    )

    return launch.LaunchDescription([container_front,
                                     container_rear])
