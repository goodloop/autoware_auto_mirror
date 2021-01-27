# Copyright 2020 Silexica GmbH, Lichtstr. 25, Cologne, Germany. All rights reserved.
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

import launch
import launch.actions
import launch_ros.actions
import lidar_integration

import os


def generate_test_description(ready_fn):
    # The node under test and the checker node that will pass/fail our tests:
    test_topic = "veloyne_cloud_node_test_topic"
    node = launch_ros.actions.Node(
        package="ray_ground_classifier_nodes",
        node_executable="ray_ground_classifier_cloud_node_exe",
        node_name="ray_ground_classifier",
        parameters=[os.path.join(os.path.dirname(__file__), 'param/test.param.yaml')],
        remappings=[
            ("points_in", test_topic)
        ])

    ld, context = lidar_integration.get_point_cloud_mutation_launch_description(
        test_nodes=[node],
        checkers=[],  # TODO we only check that node does not crash for now
        topic=test_topic,
        other_actions=[
            launch.actions.OpaqueFunction(function=lambda context: ready_fn())
        ])

    return ld, context


# Test cases are created automatically by the lidar_integration package.  We just need to
# instantiate them
active = lidar_integration.make_active_mutation_tests()

after_shutdown = lidar_integration.make_post_shutdown_mutation_tests()
