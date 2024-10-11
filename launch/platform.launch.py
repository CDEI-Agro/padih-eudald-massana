import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, OrSubstitution
from launch.actions import IncludeLaunchDescription, SetLaunchConfiguration, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition, LaunchConfigurationEquals


def generate_launch_description():


    ros_domain_id = os.getenv('ROS_DOMAIN_ID')
    if ros_domain_id not in ['1', '2', '10', '11']:
        raise ValueError("ros_domain_id must be either '1' or '2', '10' or '11'. Please set ros domain id"
                         "by running the following command: set_moby_model GREEN or set_moby_model RED or export ROS_DOMAIN_ID= ros_domain_id")

    # Launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value='false',
            description="Use simulation mode",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="sonoma_raceway.world",
            description="world file for simulation",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ouster",
            default_value="false",
            description="Launch Ouster lidar",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ublox",
            default_value="false",
            description="Launch Ublox GPS",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "odom_tf_from_controller",
            default_value="false",
            description="Get odom->base_link tf from diff drive controller.",
        )
    )

    simulation_mode = LaunchConfiguration('sim', default='false')
    world_file = LaunchConfiguration('world', default='sonoma_raceway.world')
    ouster_arg = LaunchConfiguration("ouster", default='false')
    ublox_arg = LaunchConfiguration("ublox", default='false')
    odom_tf_from_controller = LaunchConfiguration("odom_tf_from_controller", default='false')

    # Include platform launch file
    sim_platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('padih_bot'), 'launch', 'sim.launch.py'])
        ]),
    launch_arguments={'world': world_file}.items(),
    condition=IfCondition(simulation_mode)
    )

    real_platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('padih_bot'), 'launch', 'diffbot.launch.py'])
        ]),
        launch_arguments={'odom_tf_from_controller':odom_tf_from_controller}.items(),
        condition=UnlessCondition(simulation_mode)
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('padih_bot'), 'launch', 'sensors.launch.py'])
        ]),
        launch_arguments={'ouster': ouster_arg, 'ublox': ublox_arg}.items(),
        condition=UnlessCondition(simulation_mode)
    )

    nodes = [
        sim_platform_launch,
        real_platform_launch,
        # sensors_launch
    ]

    # Launch them all!
    return LaunchDescription(nodes)