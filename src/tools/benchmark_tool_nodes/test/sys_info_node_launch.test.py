# Copyright 2021 Arm Limited
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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import pathlib

import os
import pytest
import unittest

import signal


@pytest.mark.launch_test
def generate_test_description():

    result_path = os.path.join(get_package_share_directory('benchmark_tool_nodes'), 'test_result')
    pathlib.Path(result_path).mkdir(parents=False, exist_ok=True)
    sys_info = Node(
        package='benchmark_tool_nodes',
        executable='sys_info_node.py',
        name='sys_info_node',
        namespace='benchmark',
        output='screen',
        parameters=[
            {"result_path": result_path},
            {"sampling_rate": 1}
        ],
    )

    context = {'sys_info': sys_info}

    launch_description = LaunchDescription([
        sys_info,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()])

    return launch_description, context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, sys_info):
        # Check that process exits with expected codes: either SIGINT, SIGABRT or SIGTERM codes
        # are fine
        launch_testing.asserts.assertExitCodes(
            proc_info, [-signal.SIGINT, -signal.SIGABRT, -signal.SIGTERM],
            process=sys_info)
