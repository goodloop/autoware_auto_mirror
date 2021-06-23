# Copyright 2021 Robotec.ai
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

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Launch the system modules.

     * emergency_handler
    """
    # Packages
    emergency_handler_pkg_prefix = get_package_share_directory('emergency_handler')

    # Launch
    emergency_handler_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [emergency_handler_pkg_prefix, '/launch/emergency_handler.launch.py']),
        launch_arguments={}.items()
    )

    return LaunchDescription([
        emergency_handler_launch,
    ])
