# Copyright 2018 Open Source Robotics Foundation, Inc.
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

import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition, LaunchConfigurationEquals


def generate_launch_description():

    gps_wpf_dir = get_package_share_directory( "agri_bot")
    rl_params_file = os.path.join(gps_wpf_dir, "config", "dual_ekf_navsat_params.yaml")

    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time if true'))
    declared_arguments.append(DeclareLaunchArgument("odom_tf_from_controller", default_value="false", description="Get odom->base_link tf from controller"))
    declared_arguments.append(DeclareLaunchArgument('global_localizer', default_value='ekf', description='use extended kalman filter to get map->odom transform'))


    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    odom_tf_from_controller = LaunchConfiguration("odom_tf_from_controller", default='false')
    global_localizer = LaunchConfiguration('global_localizer', default='ekf')

    odom_to_base_link_tf_using_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "odometry/local")],
        condition=UnlessCondition(odom_tf_from_controller)
    )

    map_to_odom_tf_using_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "odometry/global")],
        condition=LaunchConfigurationEquals('global_localizer', 'ekf')
    )

    navsat_transform = Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[rl_params_file, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("imu", "/navheading"),
                    ("gps/fix", "gps/fix"),
                    ("gps/filtered", "gps/filtered"),
                    ("odometry/gps", "odometry/gps"),
                    ("odometry/filtered", "odometry/global"),
                ],
                condition=LaunchConfigurationEquals('global_localizer', 'ekf'),
                arguments=["--ros-args", "--log-level", "warn"]
    )


    nodes = [
        odom_to_base_link_tf_using_ekf,
        map_to_odom_tf_using_ekf,
        navsat_transform
    ]

    return LaunchDescription(declared_arguments+ nodes)

