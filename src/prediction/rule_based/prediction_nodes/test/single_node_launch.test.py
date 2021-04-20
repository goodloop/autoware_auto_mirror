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
# class TestPredictionNodes(unittest.TestCase):

#     def test_prediction_nodes(self, proc_info, proc_output, processes_to_test):
#         for process_name in processes_to_test:
#            proc_info.assertWaitForShutdown(process=process_name, timeout=10)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, prediction_node):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=prediction_node)
