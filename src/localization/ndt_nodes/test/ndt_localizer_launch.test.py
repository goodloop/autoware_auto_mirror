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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_testing

import os
import pytest
import unittest


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


@pytest.mark.launch_test
def generate_test_description(ready_fn):
    # P2D NDT localizer parameter file definition.
    p2d_ndt_localizer_file_path = get_share_file(
        'ndt_nodes', 'param/p2d_ndt_node.default.param.yaml')
    p2d_ndt_localizer_param_file = LaunchConfiguration(
        'params', default=[p2d_ndt_localizer_file_path])

    # P2D NDT localizer node execution definition.
    p2d_ndt_localizer_runner = Node(
        package='ndt_nodes',
        node_executable='p2d_ndt_localizer_exe',
        parameters=[p2d_ndt_localizer_param_file],
        remappings=[("points_in", "points_nonground")])

    context = {'p2d_ndt_localizer_runner': p2d_ndt_localizer_runner}

    return LaunchDescription([
        p2d_ndt_localizer_runner,
        # Start tests right away - no need to wait for anything
        OpaqueFunction(function=lambda context: ready_fn())]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, p2d_ndt_localizer_runner):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=p2d_ndt_localizer_runner)
