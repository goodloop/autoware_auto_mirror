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

# Co-developed by Tier IV, Inc. and Apex.AI, Inc.

"""Example launch file for a new package."""

# from ament_index_python import get_package_share_directory
# import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing

# import os
import pytest
import unittest

from ros2cli.node.strategy import NodeStrategy
from ros2topic.api import get_topic_names_and_types


@pytest.mark.launch_test
def generate_test_description():

    prediction_node = Node(
        package='prediction_nodes',
        executable='prediction_nodes_node_exe',
        name='prediction_nodes_node',
        node_namespace='',
        output='screen',
        # parameters=[
        # os.path.join(
        # get_package_share_directory('prediction_nodes'), 'param/test.param.yaml')
        # ],
    )

    context = {'prediction_node': prediction_node}

    launch_description = LaunchDescription([
        prediction_node,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()])

    return launch_description, context

# Add active test to work around bug https://github.com/ros2/launch/issues/380 doesn't help
class TestPredictionNodes(unittest.TestCase):

    def test_topics(self, proc_output):
        desired_topics_types = dict(parameter_events=['rcl_interfaces/msg/ParameterEvent'])
        args = dict(node_name_suffix='prediction_nodes_node_exe')
        with NodeStrategy(args) as node:
            topic_names_and_types = get_topic_names_and_types(node=node)
            for d in desired_topics_types.keys():
                if d in topic_names_and_types.keys():
                    if d.value() != topic_names_and_types[d]:
                        raise RuntimeError("mismatch")
            for (topic_name, topic_types) in topic_names_and_types:
                print(topic_name, topic_types)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, prediction_node):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=prediction_node)
