#!/usr/bin/env python3

# Copyright 2020 StreetScooter GmbH, Aachen, Germany
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

"""Launch Modules for testing mpc with motion_model_testing_simulator."""

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch controller_testing_node and mpc_controller."""
    controller_testing_pkg_prefix = get_package_share_directory("controller_testing")
    controller_testing_param_file = os.path.join(
        controller_testing_pkg_prefix, "param/defaults.param.yaml"
    )
    rviz_cfg_path = os.path.join(controller_testing_pkg_prefix, 'config/mpc_cotrols.rviz')

    mpc_controller_pkg_prefix = get_package_share_directory("mpc_controller_nodes")
    mpc_controller_param_file = os.path.join(mpc_controller_pkg_prefix, "param/defaults.yaml")

    urdf_pkg_prefix = get_package_share_directory('lexus_rx_450h_description')
    urdf_path = os.path.join(urdf_pkg_prefix, 'urdf/lexus_rx_450h.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()

    # Arguments
    controller_testing_param = DeclareLaunchArgument(
        "controller_testing_param_file",
        default_value=controller_testing_param_file,
        description="Path to config file for Controller Testing",
    )

    mpc_controller_param = DeclareLaunchArgument(
        "mpc_controller_param_file",
        default_value=mpc_controller_param_file,
        description="Path to config file to MPC Controller",
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

    show_final_report_param = DeclareLaunchArgument(
        'show_final_report',
        default_value='False',
        description='Show graphical report of final results'
    )

    controller_testing_params = [
        LaunchConfiguration("controller_testing_param_file"),
        {
            'real_time_sim': LaunchConfiguration('real_time_sim'),
            'show_final_report': LaunchConfiguration('show_final_report')
        }
    ]

    # Nodes

    controller_testing = Node(
        package="controller_testing",
        executable="controller_testing_main.py",
        namespace="control",
        name="controller_testing_node",
        output="screen",
        parameters=controller_testing_params,
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
        executable="controller_testing_main.py",
        namespace="control",
        name="controller_testing_node",
        output="screen",
        parameters=controller_testing_params,
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

    # mpc_controller
    mpc_controller = Node(
        package="mpc_controller_nodes",
        executable="mpc_controller_node_exe",
        # namespace="control",
        name="mpc_controller",
        output="screen",
        parameters=[LaunchConfiguration("mpc_controller_param_file"), {}],
        remappings=[
            ("vehicle_kinematic_state", "/vehicle/vehicle_kinematic_state"),
            ("trajectory", "/planning/trajectory"),
            ("ctrl_cmd", "/vehicle/control_command"),
            ("control_diagnostic", "/control/control_diagnostic"),
        ],
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)],
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )
    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )

    return LaunchDescription(
        [
            real_time_sim_param,
            controller_testing_param,
            mpc_controller_param,
            with_rviz_param,
            show_final_report_param,
            rviz2,
            urdf_publisher,
            mpc_controller,
            controller_testing,          # if not with_rviz
            controller_testing_delayed   # with_rviz
        ]
    )
