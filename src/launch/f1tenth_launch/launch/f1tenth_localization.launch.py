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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

import os


def generate_launch_description():
    nav2_pkg_prefix = get_package_share_directory('nav2_bringup')
    f1tenth_launch_pkg_prefix = get_package_share_directory('f1tenth_launch')

    localization_param_file = os.path.join(
        f1tenth_launch_pkg_prefix, 'param/amcl.param.yaml')
    localization_param = DeclareLaunchArgument(
        'localization_param_file',
        default_value=localization_param_file,
        description='Path to config file for localization nodes'
    )

    map_file_path = os.path.join(
        f1tenth_launch_pkg_prefix, 'data/red_bull_ring_racetrack.yaml')
    map_file = DeclareLaunchArgument(
        'map',
        default_value=map_file_path,
        description='Path to 2D map config file'
    )

    print(LaunchConfiguration("map_file"))
    nav2_localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg_prefix,
                         'launch/localization_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('localization_param_file'),
            'map': LaunchConfiguration('map')
        }.items()
    )

    return LaunchDescription([
        # localization_param_file,
        localization_param,
        map_file,
        nav2_localization,
    ])
