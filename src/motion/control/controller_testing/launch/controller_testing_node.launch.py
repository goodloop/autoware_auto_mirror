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

import ament_index_python
import launch
import launch_ros.actions

def generate_launch_description():
    """Launch controller_testing_node and mpc_controller."""
    # -------------------------------- Nodes-----------------------------------
    ##########################
    ### controller_testing ###
    controller_testing_node = launch_ros.actions.Node(
        package='controller_testing',
        node_executable='controller_testing_main.py',
        node_name='controller_testing_node',
        #node_namespace='planning',
        output='screen',
        parameters=[
            "{}/defaults.param.yaml".format(
                ament_index_python.get_package_share_directory(
                    "controller_testing"
                )
            ),
            # overwrite parameters from yaml here
            {
                "state_frame":  'odom',
                "trajectory_frame":  'odom',
            }

        ],
        remappings=[
            ('vehicle_state', '/vehicle/vehicle_kinematic_state'),
            ('planned_trajectory', '/planning/trajectory'),
            ('control_command', '/vehicle/control_command')
        ]
    )

    ######################
    ### mpc_controller ###
    mpc_controller_node = launch_ros.actions.Node(
        package="mpc_controller_node",
        node_executable="mpc_controller_node_exe",
        node_name="mpc_controller",
        output='screen',
        parameters=[
            "{}/defaults.yaml".format(
                ament_index_python.get_package_share_directory(
                    "mpc_controller_node"
                )
            ),
            # overwrite parameters from yaml here
            {
                "controller.interpolation": True,
                "controller.control_lookahead_ms": 100,
                "controller.limits.min_longitudinal_velocity_mps": 0.01,
                "controller.limits.max_longitudinal_velocity_mps": 35.0,
                "controller.limits.min_acceleration_mps2": -3.0,
                "controller.limits.max_acceleration_mps2": 3.0,
                "controller.limits.min_steer_angle_rate_rps": -0.331,
                "controller.limits.max_steer_angle_rate_rps": 0.331,
                "controller.vehicle.cg_to_front_m": 1.2,
                "controller.vehicle.cg_to_rear_m": 1.5,
                "controller.weights.nominal.pose": 10.0,
                "controller.weights.nominal.heading": 10.0,
                "controller.weights.nominal.longitudinal_velocity": 10.0,
                "controller.weights.terminal.pose": 1000.0,
                "controller.weights.terminal.heading": 1000.0,
                "controller.weights.terminal.longitudinal_velocity": 1000.0,
            }
        ],
        remappings=[
            ('vehicle_kinematic_state', '/vehicle/vehicle_kinematic_state'),
            ('trajectory', '/planning/trajectory'),
            ('ctrl_cmd', '/vehicle/control_command')
        ]
    )

    ld = launch.LaunchDescription([
        controller_testing_node,
        mpc_controller_node]
    )
    return ld
