import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare


import xacro


def generate_launch_description():

    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file
    # using a command substitution here to run the xacro executable on our robot description file
    # to get the urdf (xml format) from the xacro file as a string
    robot_description_content = Command(
        [PathJoinSubstitution([FindExecutable(name="xacro")]), # get path to the executable /opt/ros/humble/bin/xacro
         # add space. The idea is the run the whole thing as a command you would on a command line
        " ",
         # get path to the robot description xacro file
        PathJoinSubstitution([FindPackageShare("padih_bot"), "src/description", "padih_bot.urdf.xacro"])
        ]
    )

    params = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'),
        node_robot_state_publisher
    ])

