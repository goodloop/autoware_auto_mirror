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
from launch_ros.actions import Node
import launch_testing

import os
import pytest
import unittest

import signal


@pytest.mark.launch_test
def generate_test_description():

    point_cloud_filter_transform_node = Node(
        package="point_cloud_filter_transform_nodes",
        executable="point_cloud_filter_transform_node_exe",
        name="point_cloud_filter_transform_node",
        namespace="lidar_front",
        parameters=[
            os.path.join(
                get_package_share_directory('point_cloud_filter_transform_nodes'),
                'param/test.param.yaml'
            )
        ]
    )

    context = {'point_cloud_filter_transform_node': point_cloud_filter_transform_node}

    return LaunchDescription([
        point_cloud_filter_transform_node,
        # Start tests right away - no need to wait for anything
        launch_testing.actions.ReadyToTest()]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, point_cloud_filter_transform_node):
        # Check that process exits with expected codes: either SIGINT, SIGABRT or SIGTERM codes
        # are fine
        launch_testing.asserts.assertExitCodes(
            proc_info, [-signal.SIGINT, -signal.SIGABRT, -signal.SIGTERM],
            process=point_cloud_filter_transform_node)
