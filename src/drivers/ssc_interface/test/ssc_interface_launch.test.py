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
from launch.actions import DeclareLaunchArgument
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
    # Prepare parameters as per launch file:
    control_command_param = DeclareLaunchArgument(
        'control_command',
        default_value="basic",  # use "raw", "basic" or "high_level"
        description='command control mode')

    ssc_interface_param = DeclareLaunchArgument(
        'ssc_interface_param',
        default_value=[
            get_share_file('ssc_interface', 'param/defaults.param.yaml')
        ],
        description='Path to config file for SSC interface')

    # Prepare node
    ssc_interface = Node(
        package='ssc_interface',
        node_name='ssc_interface_node',
        node_executable='ssc_interface_node_exe',
        node_namespace='vehicle',
        output='screen',
        parameters=[LaunchConfiguration('ssc_interface_param')],
        remappings=[
            ('gear_select', '/ssc/gear_select'),
            ('arbitrated_speed_commands', '/ssc/arbitrated_speed_commands'),
            ('arbitrated_steering_commands', '/ssc/arbitrated_steering_commands'),
            ('turn_signal_command', '/ssc/turn_signal_command'),
            ('dbw_enabled_feedback', '/ssc/dbw_enabled_fedback'),
            ('gear_feedback', '/ssc/gear_feedback'),
            ('velocity_accel_cov', '/ssc/velocity_accel_cov'),
            ('steering_feedback', '/ssc/steering_feedback'),
            ('vehicle_kinematic_state_cog', '/vehicle/vehicle_kinematic_state'),
            ('state_report_out', '/vehicle/vehicle_state_report'),
            ('state_command', '/vehicle/vehicle_state_command')
        ]
    )

    context = {'ssc_interface': ssc_interface}

    return LaunchDescription([
        control_command_param,
        ssc_interface_param,
        ssc_interface,
        # Start tests right away - no need to wait for anything
        OpaqueFunction(function=lambda context: ready_fn())]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, ssc_interface):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(proc_info, [-15], process=ssc_interface)
