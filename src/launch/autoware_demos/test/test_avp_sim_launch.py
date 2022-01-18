# Copyright 2022 the Autoware Foundation
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

import os
import unittest

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import launch_testing
import launch_testing.actions

import pytest


@pytest.mark.launch_test
def generate_test_description():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    avp_sim_launch = os.path.join(this_dir, '../launch/avp_sim.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(avp_sim_launch),
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestAutowareReady(unittest.TestCase):
    def test_init_done(self, proc_output):
        # Wait for initialization to be complete
        for process in [
            'costmap_generator', 'lanelet2_map_provider', 'behavior_planner', 'freespace_planner'
        ]:
            proc_output.assertWaitFor('Waiting for', process=process, timeout=5, stream='stderr')


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that all processes in the launch file exit with code 0
        # Exception: costmap_generator_node exits with -6
        for process_name in proc_info.process_names():
            if "costmap_generator_node" in process_name:
                launch_testing.asserts.assertExitCodes(proc_info, [-6], process=process_name)
            else:
                launch_testing.asserts.assertExitCodes(proc_info, process=process_name)
