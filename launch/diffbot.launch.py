# Copyright 2020 ros2_control Development Team
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

from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    # setting this id will serve two purposes:
    # 1) identifying which robot is in use: red, green or yellow
    # 2) restricting communication between the selected robot and the control computer only

    ros_domain_id = os.getenv('ROS_DOMAIN_ID')
    if ros_domain_id not in ['1', '2', '10', '11']:
        raise ValueError("ros_domain_id must be either '1' or '2', '10' or '11'. Please set ros domain id"
                         "by running the following command: set_moby_model GREEN or set_moby_model RED or export ROS_DOMAIN_ID= ros_domain_id")

    package_path = FindPackageShare("agri_bot")
    params_file = PathJoinSubstitution([package_path, 'config/diffdrive_controllers.yaml'])

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "odom_tf_from_controller",
            default_value="false",
            description="Get odom->base_link tf from diff drive controller.",
        )
    )
    # Initialize Arguments
    odom_tf_from_controller = LaunchConfiguration("odom_tf_from_controller", default='false')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("agri_bot"), "src/description", "agri_bot.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=false",
            " ",
            "simulation:=false",
            " ",
            "ros_domain_id:=",
            ros_domain_id,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'enable_odom_tf': odom_tf_from_controller}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, configured_params],
        output="both",
        remappings=[
            ("/diffbot_base_controller/odom", "controller/odometry"),
        ],
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]

    return LaunchDescription(declared_arguments+ nodes)
