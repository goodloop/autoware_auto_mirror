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
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
import launch_testing

from ament_index_python import get_package_share_directory
import argparse
import os
import pytest
import unittest

from ros2cli.node.strategy import NodeStrategy
from ros2topic.api import get_topic_names_and_types

from ros2node.api import get_publisher_info
from ros2node.api import get_subscriber_info

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def print_names_and_types(names_and_types):
    print(*[2 * '  ' + s.name + ': ' + ', '.join(s.types) for s in names_and_types], sep='\n')

@pytest.mark.launch_test
def generate_test_description():

    # prediction_node = Node(
    #     package='prediction_nodes',
    #     executable='prediction_nodes_node_exe',
    #     name='prediction_nodes_node',
    #     node_namespace='',
    #     output='screen',
    #     # parameters=[
    #     # os.path.join(
    #     # get_package_share_directory('prediction_nodes'), 'param/test.param.yaml')
    #     # ],
    # )

    publisher_node = ExecuteProcess(
        cmd=['python3', 'publisher_node.py'],
        cwd=os.path.join(get_package_share_directory('prediction_nodes'), 'test/'),
        # This is necessary to get unbuffered output from the process under test
        additional_env={'PYTHONUNBUFFERED': '1'},
    )


    # publisher_node2 = ExecuteProcess(
    #     cmd=['python3', 'publisher_node.py'],
    #     cwd=os.path.join(get_package_share_directory('prediction_nodes'), 'test/'),
    #     # This is necessary to get unbuffered output from the process under test
    #     additional_env={'PYTHONUNBUFFERED': '1'},
    # )

    # can't kill regularly
    container = ComposableNodeContainer(
            name='prediction_nodes_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='prediction_nodes',
                    plugin='autoware::prediction_nodes::PredictionNode',
                    name='prediction_nodes_node'),
            ],
            output='screen',
    )
    # context = {'prediction_node': prediction_node}

    launch_description = LaunchDescription([
        publisher_node,
        # container,

        # publisher_node2,
        # prediction_node,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()])

    # return launch_description, context
    return launch_description

class TestPredictionNodes(unittest.TestCase):
    # take argument from context
    # https://github.com/ros2/launch/blob/master/launch_testing/test/launch_testing/examples/context_launch_test.py
    def test_pass(self, proc_info, proc_output):
        pass

    def test_pass2(self, proc_output):
        pass

# # Add active test to work around bug https://github.com/ros2/launch/issues/380 doesn't help
# class TestPredictionNodes(unittest.TestCase):

#     def test_publishers_subscribers(self, proc_output):
#         node_name = 'prediction_nodes_node'
#         include_hidden = False

#         parser = argparse.ArgumentParser()
#         parser.add_argument('--node_name', default=node_name)
#         parser.add_argument('--node_name_suffix', default='harr')
#         args = parser.parse_args([''])
#         # args = dict(node_name_suffix='harr')
#         with NodeStrategy(args) as node:
#             print(node_name)
#             subscribers = get_subscriber_info(
#                 node=node, remote_node_name=node_name, include_hidden=include_hidden)
#             print('  Subscribers:')
#             print_names_and_types(subscribers)
#             publishers = get_publisher_info(
#                 node=node, remote_node_name=node_name, include_hidden=include_hidden)
#             print('  Publishers:')
#             print_names_and_types(publishers)

#     def topics(self, proc_output):
#         desired_topics_types = {
#             # automatically created
#             '/parameter_events': ['rcl_interfaces/msg/ParameterEvent'],
#             '/rosout': ['rcl_interfaces/msg/Log'],

#             # the interesting topics
#             '/prediction_input': ['harr'],
#         }
#         # TODO hack: not sure if this is the right type, I just crawled through
#         # undocumented source code
#         args = dict(node_name_suffix='prediction_nodes_node_exe')
#         with NodeStrategy(args) as node:
#             topic_names_and_types = dict(get_topic_names_and_types(node=node,
#                                                                    include_hidden_topics=False))
#             self.assertEqual(topic_names_and_types, desired_topics_types)

#             # for topic, msg_types in topic_names_and_types:
#             #     print(f'topic: {topic}, msg type: {msg_types[0]}')
#             #     self.assertIn(topic, desired_topics_types.keys())

#             #         if d.value() != topic_names_and_types[d]:
#             #             raise RuntimeError("mismatch")
#             # for (topic_name, topic_types) in topic_names_and_types:
#             #     print(topic_name, topic_types)


# @launch_testing.post_shutdown_test()
# class TestProcessOutput(unittest.TestCase):

#     # def test_exit_code(self, proc_output, proc_info, prediction_node):
#     #     # Check that process exits with code -15 code: termination request, sent to the program
#     #     launch_testing.asserts.assertExitCodes(proc_info, [-15], process=prediction_node)

#     def test_exit_code(self, proc_output, proc_info):
#         # Check that process exits with code -15 code: termination request, sent to the program
#         launch_testing.asserts.assertExitCodes(proc_info, [-15])
