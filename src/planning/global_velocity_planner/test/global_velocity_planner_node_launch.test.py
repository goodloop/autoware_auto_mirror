# Copyright 2022 Leo Drive Teknoloji A.Ş.
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
# Co-developed by Tier IV, Inc. Apex.AI, Inc. and Leo Drive Teknoloji A.Ş.

from ament_index_python import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node
import launch_testing

import os
import pytest
import unittest

import signal


@pytest.mark.launch_test
def generate_test_description():

    global_velocity_planner = Node(
        package='global_velocity_planner',
        executable='global_velocity_planner_node_exe',
        namespace='test',
        parameters=[os.path.join(
            get_package_share_directory('global_velocity_planner'),
            'param/defaults.param.yaml'
        )]
    )

    context = {'global_velocity_planner': global_velocity_planner}

    return LaunchDescription([
        global_velocity_planner,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, global_velocity_planner):
        # Check that process exits with expected codes: either SIGINT, SIGABRT or SIGTERM codes
        # are fine
        launch_testing.asserts.assertExitCodes(
            proc_info, [-signal.SIGINT, -signal.SIGABRT, -signal.SIGTERM],
            process=global_velocity_planner)
