#!/usr/bin/env python3

# Copyright 2020 The Autoware Foundation
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

"""Launch Modules for testing pure pursuit with motion_model_testing_simulator."""

from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path

import os


context = LaunchContext()


def get_package_share_directory(package_name):
    """Return the absolute path to the share directory of the given package."""
    sub = FindPackageShare(package_name)
    package_prefix = Path(sub.perform(context))
    return package_prefix


def generate_launch_description():
    """Launch controller_testing_node and pure purusit controller."""
    controller_testing_pkg_prefix = get_package_share_directory("controller_testing")
    controller_testing_param_file = os.path.join(
        controller_testing_pkg_prefix, "param/defaults.param.yaml"
    )
    rviz_cfg_path = os.path.join(controller_testing_pkg_prefix, 'config/pure_pursuit_cotrols.rviz')

    pure_pursuit_pkg_prefix = get_package_share_directory("pure_pursuit_nodes")
    pure_pursuit_param_file = os.path.join(
        pure_pursuit_pkg_prefix, "param/pure_pursuit.param.yaml"
    )

    urdf_pkg_prefix = get_package_share_directory('lexus_rx_450h_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/lexus_rx_450h.urdf')

    # Arguments
    controller_testing_param = DeclareLaunchArgument(
        "controller_testing_param_file",
        default_value=controller_testing_param_file,
        description="Path to config file for Controller Testing",
    )

    pure_pursuit_controller_param = DeclareLaunchArgument(
        "pure_pursuit_param_file",
        default_value=pure_pursuit_param_file,
        description="Path to config file to Pure Pursuit Controller",
    )

    with_rviz_param = DeclareLaunchArgument(
        'with_rviz',
        default_value='True',
        description='Launch RVIZ2 in addition to other nodes'
    )

    real_time_sim_param = DeclareLaunchArgument(
        'real_time_sim',
        default_value='False',
        description='Launch RVIZ2 in addition to other nodes'
    )

    # Nodes

    controller_testing = Node(
        package="controller_testing",
        node_executable="controller_testing_main.py",
        node_namespace="control",
        node_name="controller_testing_node",
        output="screen",
        parameters=[LaunchConfiguration("controller_testing_param_file"), {
            'real_time_sim': LaunchConfiguration('real_time_sim')
        }],
        remappings=[
            ("vehicle_state", "/vehicle/vehicle_kinematic_state"),
            ("planned_trajectory", "/planning/trajectory"),
            ("control_command", "/vehicle/control_command"),
            ("control_diagnostic", "/control/control_diagnostic"),
        ],
        on_exit=Shutdown(),
        condition=UnlessCondition(LaunchConfiguration('with_rviz'))
    )
    controller_testing_delayed = Node(
        package="controller_testing",
        node_executable="controller_testing_main.py",
        node_namespace="control",
        node_name="controller_testing_node",
        output="screen",
        parameters=[LaunchConfiguration("controller_testing_param_file"), {
            'real_time_sim': LaunchConfiguration('real_time_sim')
        }],
        remappings=[
            ("vehicle_state", "/vehicle/vehicle_kinematic_state"),
            ("planned_trajectory", "/planning/trajectory"),
            ("control_command", "/vehicle/control_command"),
            ("control_diagnostic", "/control/control_diagnostic"),
        ],
        on_exit=Shutdown(),
        # delay added to allow rviz to be ready, better to start rviz separately, beforehand
        prefix="bash -c 'sleep 2.0; $0 $@'",
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )

    # pure pursuit controller
    pure_pursuit_controller = Node(
        package="pure_pursuit_nodes",
        node_executable="pure_pursuit_node_exe",
        # node_namespace="control",
        node_name="pure_pursuit_node",
        output="screen",
        parameters=[LaunchConfiguration("pure_pursuit_param_file"), {}],
        remappings=[
            ("current_pose", "/vehicle/vehicle_kinematic_state"),
            ("trajectory", "/planning/trajectory"),
            ("ctrl_cmd", "/vehicle/control_command"),
            ("ctrl_diag", "/control/control_diagnostic"),
        ],
    )

    rviz2 = Node(
        package='rviz2',
        node_executable='rviz2',
        node_name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )
    urdf_publisher = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        arguments=[str(urdf_path)]
    )

    return LaunchDescription(
        [
            real_time_sim_param,
            controller_testing_param,
            pure_pursuit_controller_param,
            with_rviz_param,
            rviz2,
            urdf_publisher,
            pure_pursuit_controller,
            controller_testing,          # if not with_rviz
            controller_testing_delayed   # with_rviz
        ]
    )
