import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import UnlessCondition
def generate_launch_description():


    ros_domain_id = os.getenv('ROS_DOMAIN_ID')
    if ros_domain_id not in ['1', '2', '10', '11']:
        raise ValueError("ros_domain_id must be either '1' or '2', '10' or '11'. Please set ros domain id"
                         "by running the following command: set_moby_model GREEN or set_moby_model RED or export ROS_DOMAIN_ID= ros_domain_id")

    # get paths
    package_path = FindPackageShare("agri_bot")
    general_params = PathJoinSubstitution([package_path, 'config/general_params.yaml'])
    slam_mapping_params_file = PathJoinSubstitution([package_path, 'config/mapper_params_online_async.yaml'])
    rviz_config_file = PathJoinSubstitution([package_path, 'rviz/mapping.rviz'])

    # Launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="simulation mode",
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
            "odom_tf_from_controller",
            default_value="false",
            description="Get odom->base_link tf from diff drive controller.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "global_localizer",
            default_value="None",
            description="Localizer to get map->odom transform",
        )
    )

    simulation_mode = LaunchConfiguration('sim', default='false')
    world_file = LaunchConfiguration('world', default='sonoma_raceway.world')
    global_localizer = LaunchConfiguration('global_localizer', default='None')
    odom_tf_from_controller = LaunchConfiguration("odom_tf_from_controller", default='false')

    ublox_launch = LaunchConfiguration('ublox', default='false')
    ouster_launch = LaunchConfiguration('ouster', default='true')

    # Include platform launch file
    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('agri_bot'), 'launch', 'platform.launch.py'])
        ]),
        launch_arguments={'sim': simulation_mode,
                            'world': world_file,
                          'odom_tf_from_controller':odom_tf_from_controller,
                          'ublox': ublox_launch,
                          'ouster': ouster_launch}.items()
         )

    # Include dual_ekf launch file
    ekf_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('agri_bot'), 'launch', 'dual_ekf_navsat.launch.py']),
        ]),
        launch_arguments={'use_sim_time': simulation_mode,
                          'odom_tf_from_controller': odom_tf_from_controller,
                          'global_localizer': global_localizer}.items(),

    )

    slam_mapping_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py'])
        ]),
        launch_arguments={'use_sim_time': simulation_mode,
                          'slam_params_file': slam_mapping_params_file }.items(),
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
        ekf_localizer_launch,
        slam_mapping_node,
        joy_driver,
        teleop_twist_joy_node,
        twist_mux,
        # rviz_node
    ]

    # Launch them all!
    return LaunchDescription(nodes+declared_arguments)