from ament_index_python import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node
import launch_testing

import os
import pytest
import unittest


@pytest.mark.launch_test
def generate_test_description():

    emergency_handler = Node(
        package='emergency_handler',
        executable='emergency_handler_exe',
        namespace='test',
        parameters=[os.path.join(
            get_package_share_directory('emergency_handler'),
            'param/test.param.yaml'
        )]
    )

    context = {'emergency_handler': emergency_handler}

    return LaunchDescription([
        emergency_handler,
        launch_testing.actions.ReadyToTest()]
    ), context


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_output, proc_info, emergency_handler):
        # Check that process exits with code -15 code: termination request, sent to the program
        launch_testing.asserts.assertExitCodes(
            proc_info,
            [-15],
            process=emergency_handler
        )
