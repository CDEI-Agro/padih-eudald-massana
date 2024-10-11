import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from nav2_common.launch import RewrittenYaml
def generate_launch_description():


    ros_domain_id = os.getenv('ROS_DOMAIN_ID')
    if ros_domain_id not in ['1', '2', '10', '11']:
        raise ValueError("ros_domain_id must be either '1' or '2', '10' or '11'. Please set ros domain id"
                         "by running the following command: set_moby_model GREEN or set_moby_model RED or export ROS_DOMAIN_ID= ros_domain_id")

    # get paths
    package_path = FindPackageShare("padih_bot")
    general_params = PathJoinSubstitution([package_path, 'config/general_params.yaml'])
    slam_params = PathJoinSubstitution([package_path, 'config/mapper_params_online_async.yaml'])
    rviz_config_file = PathJoinSubstitution([package_path, 'rviz/navigation.rviz'])

    # Launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution([package_path, 'maps/sonoma_map.yaml']),
            description="default map yaml file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="whether to run in simulation mode",
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
            "global_localizer",
            default_value="ekf",
            description="Global localizer to get map->odom transform. ekf or amcl",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ublox",
            default_value="true",
            description="Enable Ublox GPS sensor",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ouster",
            default_value="true",
            description="Enable Ouster Lidar sensor",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "odom_tf_from_controller",
            default_value="false",
            description="Get odom->base_link tf from controller",
        )
    )

    map_file_path = LaunchConfiguration('map', default=PathJoinSubstitution([package_path, 'maps/sonoma_map.yaml']))
    simulation_mode = LaunchConfiguration('sim', default='false')
    world_file = LaunchConfiguration('world', default='sonoma_raceway.world')
    global_localizer = LaunchConfiguration('global_localizer', default='ekf')
    ublox_arg = LaunchConfiguration("ublox", default='true')
    ouster_arg = LaunchConfiguration("ouster", default='true')
    slam_mode = LaunchConfiguration("slam_mode", default='localization')
    odom_tf_from_controller = LaunchConfiguration("odom_tf_from_controller", default='false')

    set_ublox_false_for_amcl_true = SetLaunchConfiguration(
        'ublox', 'false',
        condition=LaunchConfigurationEquals('global_localizer', 'amcl')
    )

    set_ublox_false_for_slam_true = SetLaunchConfiguration(
        'ublox', 'false',
        condition=LaunchConfigurationEquals('global_localizer', 'slam')
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'mode': slam_mode,
        'map_file_name': map_file_path}

    configured_slam_params =RewrittenYaml(
            source_file=slam_params,
            param_rewrites=param_substitutions,
            convert_types=True)

    # Include platform launch file
    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('padih_bot'), 'launch', 'platform.launch.py'])
        ]),
        launch_arguments={'sim': simulation_mode,
                        'world': world_file,
                        'ublox': ublox_arg,
                        'ouster': ouster_arg,
                        'odom_tf_from_controller': odom_tf_from_controller}.items()
         )

        # Include map launch file
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('padih_bot'), 'launch', 'map.launch.py'])
        ]),
        launch_arguments={'map': map_file_path,
                          'use_sim_time': simulation_mode}.items(),
        condition=LaunchConfigurationNotEquals('global_localizer', 'slam')
        )

    # Include dual_ekf launch file
    ekf_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('padih_bot'), 'launch', 'dual_ekf_navsat.launch.py']),
        ]),
        launch_arguments={'use_sim_time': simulation_mode,
                          'global_localizer': global_localizer,
                          'odom_tf_from_controller': odom_tf_from_controller}.items(),
    )

    amcl_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('padih_bot'), 'launch', 'amcl_launch.py']),
        ]),
        launch_arguments={'use_sim_time': simulation_mode}.items(),
        condition=LaunchConfigurationEquals('global_localizer', 'amcl')
    )

    slam_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']),
        ]),
        launch_arguments={'use_sim_time': simulation_mode,
                          'slam_params_file': configured_slam_params}.items(),
        condition=LaunchConfigurationEquals('global_localizer', 'slam')
    )


    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('padih_bot'), 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={'use_sim_time': simulation_mode}.items(),
    )

    turret_joy = Node(
        package="padih_bot",
        executable="turret_joy.py",
        name='turret_joy',
        parameters=[general_params,]
    )
    # Launch joystick driver and teleop node
    joy_driver = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        parameters=[{
            'dev': "/dev/input/js0",
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }]
    )
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[PathJoinSubstitution([package_path, 'config/general_params.yaml'])],
        remappings={('/cmd_vel', 'cmd_vel_joy')},
    )

    # Twist mulitplexer: sets priotities on command velocities coming from different sources
    # useful to have control over the robot with both joystick and the nav2 stack
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[general_params],
        remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    nodes = [
        platform_launch,
        set_ublox_false_for_amcl_true,
        set_ublox_false_for_slam_true,
        ekf_localizer_launch,
        # amcl_localizer_launch,
        # slam_localizer_launch,
        # map_launch,
        navigation_launch,
        joy_driver,
        turret_joy,
        teleop_twist_joy_node,
        twist_mux,
        # rviz_node
    ]

    # Launch them all!
    return LaunchDescription(nodes+declared_arguments)