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

    ndt_mapper_param_file = get_share_file(
        'ndt_mapping_nodes', 'param/ndt_mapper.param.yaml'
    )

    ndt_mapper = Node(
        package='ndt_mapping_nodes',
        node_executable='ndt_mapper_node_exe',
        node_name='ndt_mapper_node',
        node_namespace='mapper',
        output='screen',
        parameters=[LaunchConfiguration(
            'ndt_param_param_file',
            default=ndt_mapper_param_file)],
    )

    context = {'ndt_mapper': ndt_mapper}

    return LaunchDescription([
        ndt_mapper,
        # Start tests right away - no need to wait for anything
        OpaqueFunction(function=lambda context: ready_fn())]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, ndt_mapper):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=ndt_mapper)
