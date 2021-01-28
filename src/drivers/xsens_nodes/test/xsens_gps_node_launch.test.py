# Copyright 2021 the Autoware Foundation
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

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
import launch_testing

import os
import pytest
import unittest


@pytest.mark.launch_test
def generate_test_description(ready_fn):

    xsens_gps_node = Node(
        package='xsens_nodes',
        node_name='xsens_gps_node',
        node_executable="xsens_gps_node_exe",
        node_namespace="vehicle",
        output='screen',
        parameters=[
            os.path.join(os.path.dirname(__file__), 'param/test.param.yaml')
        ],
    )

    context = {'xsens_gps_node': xsens_gps_node}

    return LaunchDescription([
        xsens_gps_node,
        # Start tests right away - no need to wait for anything
        OpaqueFunction(function=lambda context: ready_fn())]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, xsens_gps_node):
        # In this case the device itself is not available at the determined port
        launch_testing.asserts.assertExitCodes(proc_info, [2], process=xsens_gps_node)
