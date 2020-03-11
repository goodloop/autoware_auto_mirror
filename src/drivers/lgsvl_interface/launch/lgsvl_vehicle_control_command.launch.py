# Copyright 2020 Apex.AI, Inc.
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
import launch
import launch_ros.actions
import launch.substitutions
import launch.launch_description_sources
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument 
import ros2launch.api


def get_param(package_name, param_file):
    return ros2launch.api.get_share_file_path_from_package(
        package_name=package_name,
        file_name=param_file
    )


def generate_launch_description():
    """
    Launches a minimal joystick + LGSVL demo. The joystick_vehicle_interface and the lgsvl_interface
    are modified via parameter remapping to use VehicleControlCommand as an output. The vehicle can
    be controlled by manipulating the left joystick of the gamepad.
    """



    # CLI
    high_level_command_param = DeclareLaunchArgument(
        'high_level_command', 
        default_value="''",
        description='high_level_command control mode topic name')

    basic_command_param = DeclareLaunchArgument(
        'basic_command', 
        default_value="vehicle_command",
        description='basic_command control mode topic name')

    raw_command_param = DeclareLaunchArgument(
        'raw_command', 
        default_value="''", 
        description='raw_command control mode topic name')

    joy_translator_param = launch.actions.DeclareLaunchArgument(
        'joy_translator_param',
        default_value=[
            get_param('joystick_vehicle_interface', 'logitech_f310.default.param.yaml')
        ],
        description='Path to config file for joystick translator')
    
    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param',
        default_value=[ 
            get_param('lgsvl_interface', 'lgsvl.param.yaml')
        ],
        description='Path to config file for lgsvl interface')
    # Joystick stuff
    joystick_launch_file_path = get_param('joystick_vehicle_interface',
                                 'joystick_vehicle_interface.launch.py')
    joystick = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(joystick_launch_file_path),
        launch_arguments={
            "joy_translator_param": LaunchConfiguration("joy_translator_param"),
            "high_level_command": LaunchConfiguration('high_level_command'),
            "basic_command": LaunchConfiguration('basic_command'),
            "raw_command": LaunchConfiguration('raw_command')
        }.items()
    )

    # LGSVL stuff
    lgsvl_launch_file_path = get_param('lgsvl_interface',
                                 'lgsvl.launch.py')
    lgsvl = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(lgsvl_launch_file_path),
        launch_arguments={
            "lgsvl_interface_param": LaunchConfiguration("lgsvl_interface_param"),
            "high_level_command": LaunchConfiguration('high_level_command'),
            "basic_command": LaunchConfiguration('basic_command'),
            "raw_command": LaunchConfiguration('raw_command')
        }.items()
    )

    return launch.LaunchDescription([
      high_level_command_param,
      basic_command_param,
      raw_command_param,
      joy_translator_param,
      lgsvl_interface_param,
      joystick,
      lgsvl])
