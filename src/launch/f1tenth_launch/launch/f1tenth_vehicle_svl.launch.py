# Copyright 2020-2022, The Autoware Foundation
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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

import os


def generate_launch_description():
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')

    # params
    lgsvl_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, 'param/lgsvl_interface.param.yaml')
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param_file',
        default_value=lgsvl_param_file,
        description='Path to config file for LGSVL Interface'
    )

    joy_translator_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, 'param/logitech_f310_basic.param.yaml')
    joy_translator_param = DeclareLaunchArgument(
        'joy_translator_param',
        default_value=joy_translator_param_file,
        description='Path to config file for LGSVL Interface'
    )

    with_joy_param = DeclareLaunchArgument(
        'with_joy',
        default_value='False',
        description='Launch joystick_interface in addition to other nodes'
    )

    # Nodes
    urdf_path = os.path.join(f1tenth_launch_pkg_prefix,
                             'urdf/f1tenth_svl.urdf')
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()
    urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': urdf_file}],
    )

    lgsvl_interface = Node(
        package='lgsvl_interface',
        executable='lgsvl_interface_exe',
        namespace='vehicle',
        output='screen',
        parameters=[
            LaunchConfiguration('lgsvl_interface_param_file'),
            {"lgsvl.publish_tf": True}
        ],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "/vehicle/state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom")
        ]
    )

    joystick_vehicle_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(f1tenth_launch_pkg_prefix,
                         'launch/f1tenth_joystick_controller.launch.py')
        ),
        launch_arguments={
            'control_command': 'basic',
            'joy_translator_param': LaunchConfiguration('joy_translator_param'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('with_joy'))
    )

    lgsvl_bridge = ExecuteProcess(cmd=["lgsvl_bridge"])

    return LaunchDescription([
        with_joy_param,
        joy_translator_param,
        joystick_vehicle_interface,
        lgsvl_interface_param,
        lgsvl_bridge,
        urdf_publisher,
        lgsvl_interface
    ])
