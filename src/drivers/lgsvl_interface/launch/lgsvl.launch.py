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
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument 
import ament_index_python
import os


def get_param(package_name, param_file):
    return os.path.join(ament_index_python.get_package_share_directory(package_name), param_file)


def generate_launch_description():
    """
    Launches necessary dependencies for working with LGSVL simulator and ROS 2/Autoware.Auto:
    The LGSVL interface, which translates inputs and outputs to and from ROS standard coordinate
    systems, and the ros2 web bridge, which allows LGSVL to pick up ROS 2 topics.
    """
    # CLI

    high_level_command_param = DeclareLaunchArgument(
        'high_level_command', 
        default_value="''",
        description='high_level_command control mode topic name')

    basic_command_param = DeclareLaunchArgument(
        'basic_command', 
        default_value="''",
        description='basic_command control mode topic name')

    raw_command_param = DeclareLaunchArgument(
        'raw_command', 
        default_value='raw_command', 
        description='raw_command control mode topic name')

    lgsvl_interface_param = DeclareLaunchArgument(
        'lgsvl_interface_param',
        default_value=[
            get_param('lgsvl_interface', 'lgsvl.param.yaml')
        ],
        description='Path to config file for lgsvl interface')
    # Nodes
    lgsvl_interface = launch_ros.actions.Node(
        package='lgsvl_interface',
        node_executable='lgsvl_interface_exe',
        output='screen',
     
        parameters=[
            LaunchConfiguration('lgsvl_interface_param'),
            # overwrite parameters from yaml here
            {"high_level_command.name" : LaunchConfiguration('high_level_command')},
            {"basic_command.name" : LaunchConfiguration('basic_command')},
            {"raw_command.name" :  LaunchConfiguration('raw_command')}
        ]
    )
    # ros2 web bridge
    lgsvl_bridge = launch.actions.ExecuteProcess(cmd=["rosbridge"], shell=True)

    ld = launch.LaunchDescription([
        high_level_command_param,
        basic_command_param,
        raw_command_param,
        lgsvl_interface_param,
        lgsvl_interface])
        #lgsvl_bridge]) # TODO(c.ho) bring this back once ADE version of web bridge is correct
    return ld
