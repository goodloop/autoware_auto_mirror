# Copyright 2020-2021 the Autoware Foundation
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


"""
This launch file launch all the nodes necessary to produce object tracks based on raw lidar points
and 2d detections from two cameras in the SVL simulator. It also launches rviz. Use the
lgsvl-sensors-dual-camera.json sensor configuration file for the simulator.
"""


import os

from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch.substitutions
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import Shutdown
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration


def get_param_file(package_name, file_name):
    """Pass the given param file as a LaunchConfiguration."""
    file_path = os.path.join(
        get_package_share_directory(package_name),
        'param',
        file_name)
    return launch.substitutions.LaunchConfiguration(
        'params', default=[file_path])


def get_lexus_robot_description(filename):
    # Setup robot state publisher
    vehicle_description_pkg_path = get_package_share_directory(
        'lexus_rx_450h_description')
    urdf_path = os.path.join(vehicle_description_pkg_path, 'urdf',
                             filename)
    with open(urdf_path, 'r') as infp:
        urdf_file = infp.read()

    return urdf_file


def generate_launch_description():
    use_ndt = DeclareLaunchArgument(
        'use_ndt',
        default_value='True',
        description='Set this to "False" to use Ground truth odometry instead of odom from NDT.'
                    ' Note the option is case sensitive. Use only "True" or "False"'
    )

    lidar_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("autoware_demos"),
                                                   "launch/lidar_detection_core_lgsvl.launch.py")),
    )

    ndt_state_estimation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("autoware_demos"),
                                                   "launch/ndt_state_estimation_core_lgsvl.launch.py")),
        condition=IfCondition(LaunchConfiguration('use_ndt'))
    )

    lgsvl_interface = Node(
        executable='lgsvl_interface_exe',
        namespace='vehicle',
        on_exit=Shutdown(),
        output='screen',
        package='lgsvl_interface',
        parameters=[
            get_param_file('autoware_demos', 'lgsvl_interface.param.yaml'),
            {"lgsvl.publish_tf": True}  # Needed for RViz and NDT initialization
        ],
        remappings=[
            ("vehicle_control_cmd", "/lgsvl/vehicle_control_cmd"),
            ("vehicle_state_cmd", "/lgsvl/vehicle_state_cmd"),
            ("state_report", "/lgsvl/state_report"),
            ("state_report_out", "/vehicle/state_report"),
            ("gnss_odom", "/lgsvl/gnss_odom"),
            ("vehicle_odom", "/lgsvl/vehicle_odom")
        ])

    vision_detections_right = Node(
        name='vision_detections',
        executable='ground_truth_detections_node_exe',
        package='ground_truth_detections',
        on_exit=Shutdown(),
        remappings=[
            ("/simulator/ground_truth/detections2D",
             "/simulator/ground_truth/camera_right/detections2D"),
            ("/perception/ground_truth_detections_2d",
             "/perception/ground_truth/camera_right_2d")
        ],
        parameters=[
            {"vision_frame_id": "camera_right"}
        ]
    )

    vision_detections_left = Node(
        name='vision_detections',
        executable='ground_truth_detections_node_exe',
        package='ground_truth_detections',
        on_exit=Shutdown(),
        remappings=[
            ("/simulator/ground_truth/detections2D",
             "/simulator/ground_truth/camera_left/detections2D"),
            ("/perception/ground_truth_detections_2d",
             "/perception/ground_truth/camera_left_2d")
        ],
        parameters=[
            {"vision_frame_id": "camera_left"}
        ]
    )

    lidar_projector_right = Node(
        name='lidar_projector_right',
        package='cluster_projection_node',
        executable='cluster_projection_node_exe',
        parameters=[get_param_file('cluster_projection_node',
                                   'cluster_projection_node.param.yaml'),
                    {"camera_frame": "camera_right"}],
        remappings=[
            ("/clusters_in", "/perception/associated_detections"),
            ("/projected_clusters", "/projected_clusters_on_right_camera")
        ],
        on_exit=Shutdown()
    )

    lidar_projector_left = Node(
        name='lidar_projector_left',
        package='cluster_projection_node',
        executable='cluster_projection_node_exe',
        parameters=[get_param_file('cluster_projection_node',
                                   'cluster_projection_node.param.yaml'),
                    {"camera_frame": "camera_left"}],
        remappings=[
            ("/clusters_in", "/perception/associated_detections"),
            ("/projected_clusters", "/projected_clusters_on_left_camera")
        ],
        on_exit=Shutdown()
    )

    image_visualizer_right = Node(
        name='image_visualizer_right',
        package='detection_2d_visualizer',
        executable='detection_2d_visualizer_node_exe',
        on_exit=Shutdown(),
        parameters=[get_param_file('autoware_demos',
                                   'tracker_detection_visualization.param.yaml')],
        remappings=[
            ("/projections", "/projected_clusters_on_right_camera"),
            ("/simulator/main_camera", "/simulator/camera_right"),
            ("/perception/ground_truth_detections_2d", "/perception/ground_truth/camera_right_2d"),
            ("image_with_detections", "image_right_with_detections")
        ]
    )

    image_visualizer_left = Node(
        name='image_visualizer_left',
        package='detection_2d_visualizer',
        executable='detection_2d_visualizer_node_exe',
        on_exit=Shutdown(),
        remappings=[
            ("/projections", "/projected_clusters_on_left_camera"),
            ("/simulator/main_camera", "/simulator/camera_left"),
            ("/perception/ground_truth_detections_2d", "/perception/ground_truth/camera_left_2d"),
            ("image_with_detections", "image_left_with_detections")
        ]
    )

    multi_object_tracker_using_lgsvl = Node(
        executable='multi_object_tracker_node_exe',
        name='multi_object_tracker',
        namespace='perception',
        on_exit=Shutdown(),
        package='tracking_nodes',
        parameters=[
            get_param_file('autoware_demos', 'multi_object_tracker.param.yaml'),
            {
                'use_ndt': False,
                'track_frame_id': "odom",
            }
        ],
        remappings=[
            ("detected_objects", "/lidars/lidar_detected_objects"),
            ("ego_state", "/vehicle/odom_pose"),
            ("classified_rois1", "/perception/ground_truth/camera_left_2d"),
            ("classified_rois2", "/perception/ground_truth/camera_right_2d"),
            ("clusters", "/lidars/cluster_points")
        ],
        condition=LaunchConfigurationEquals('use_ndt', 'False')
    )

    multi_object_tracker_using_ndt = Node(
        executable='multi_object_tracker_node_exe',
        name='multi_object_tracker',
        namespace='perception',
        on_exit=Shutdown(),
        package='tracking_nodes',
        parameters=[
            get_param_file('autoware_demos', 'multi_object_tracker.param.yaml'),
            {
                'use_ndt': True,
                'track_frame_id': "map",
            }
        ],
        remappings=[
            ("detected_objects", "/lidars/lidar_detected_objects"),
            ("ego_state", "/localization/odometry"),
            ("classified_rois1", "/perception/ground_truth/camera_left_2d"),
            ("classified_rois2", "/perception/ground_truth/camera_right_2d"),
            ("clusters", "/lidars/cluster_points")
        ],
        condition=IfCondition(LaunchConfiguration('use_ndt'))
    )

    dual_camera_robot_state_publisher_runner = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {
                'robot_description': get_lexus_robot_description('lexus_rx_450h_dual_camera.urdf')
            }
        ],
    )

    # Run rviz
    examples_pkg_path = get_package_share_directory(
        'autoware_demos')
    rviz_cfg_path = os.path.join(examples_pkg_path, 'rviz2',
                                 'object_tracking_dual_camera.rviz')
    rviz_runner = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path)])

    return launch.LaunchDescription([
        use_ndt,
        lidar_detection_launch,
        ndt_state_estimation_launch,
        vision_detections_left,
        vision_detections_right,
        lgsvl_interface,
        multi_object_tracker_using_lgsvl,
        multi_object_tracker_using_ndt,
        dual_camera_robot_state_publisher_runner,
        rviz_runner,
        lidar_projector_right,
        lidar_projector_left,
        image_visualizer_right,
        image_visualizer_left
    ])
