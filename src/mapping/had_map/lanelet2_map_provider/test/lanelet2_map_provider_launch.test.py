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

    lanelet2_map_provider_param_file = get_share_file(
        'lanelet2_map_provider', 'param/lanelet2_map_provider.param.yaml'
    )

    map_osm_file = LaunchConfiguration(
        'map_osm_file', default=[get_share_file(
            'autoware_auto_avp_demo', 'data/autonomoustuff_parking_lot.osm')]
    )

    lanelet2_map_provider = Node(
        package='lanelet2_map_provider',
        node_executable='lanelet2_map_provider_exe',
        node_namespace='had_maps',
        parameters=[LaunchConfiguration(
            'lanelet2_map_provider_param_file', default=[lanelet2_map_provider_param_file]),
            {'map_osm_file': map_osm_file}]
    )

    context = {'lanelet2_map_provider': lanelet2_map_provider}

    return LaunchDescription([
        lanelet2_map_provider,
        # Start tests right away - no need to wait for anything
        OpaqueFunction(function=lambda context: ready_fn())]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, lanelet2_map_provider):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=lanelet2_map_provider)
