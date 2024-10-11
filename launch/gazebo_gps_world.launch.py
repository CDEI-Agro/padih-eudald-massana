# Copyright (c) 2018 Intel Corporation
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

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution

def generate_launch_description():
    # Get the launch directory
    gps_wpf_dir = get_package_share_directory("padih_bot")
    launch_dir = os.path.join(gps_wpf_dir, 'launch')
    world = os.path.join(gps_wpf_dir, "worlds", "sonoma_raceway.world")

    # The robot state publisher node with the robot description
    robot_description_content = Command(
        # The idea is to run the whole thing as a command you would type in the terminal
        [FindExecutable(name="xacro"), # get path to the executable /opt/ros/humble/bin/xacro
        " ", # add space
        PathJoinSubstitution([gps_wpf_dir, "src/description", "agri_bot.urdf.xacro"]), # get path to the robot description xacro file
         " ",
         "simulation:=true"
         ]
    )


    # Specify the actions
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', world],
        cwd=[launch_dir], output='both')

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='both')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[{'robot_description': robot_description_content}])

    # Create the launch description and populate
    ld = LaunchDescription()

    # simulator launch
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)

    # robot state publisher launch
    ld.add_action(start_robot_state_publisher_cmd)

    return ld
