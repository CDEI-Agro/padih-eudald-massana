import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import FindExecutable, Command, LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess

from launch_ros.actions import Node



def generate_launch_description():

    package_path = get_package_share_directory("padih_bot")
    launch_dir = os.path.join(package_path, 'launch')

    # Declaring arguments of the launch file
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('world', default_value='sonoma_raceway.world', description='path to world file'))
    world_file = LaunchConfiguration('world')


    # The robot state publisher node with the robot description
    robot_description_content = Command(
        # The idea is to run the whole thing as a command you would type in the terminal
        [FindExecutable(name="xacro"), # get path to the executable /opt/ros/humble/bin/xacro
        " ", # add space
        PathJoinSubstitution([package_path, "src/description", "padih_bot.urdf.xacro"]), # get path to the robot description xacro file
         " ",
         "simulation:=true"
         ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # Gazebo server and client
    start_gazebo_server_cmd = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so', PathJoinSubstitution([package_path, 'worlds', world_file])],
        cwd=[launch_dir], output='both')

    start_gazebo_client_cmd = ExecuteProcess(
        cmd=['gzclient'],
        cwd=[launch_dir], output='both')

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    )

    turret_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["turret_controller", "--controller-manager", "/controller_manager"],
    )
    
    # Run the spawner node from the gazebo_ros package.
    spawn_padih_bot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                    '-entity', 'padih_bot',
                   '-timeout', '200',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.40'],
        output='screen'
    )

    nodes = [
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_padih_bot,
        joint_state_broadcaster_spawner,
        robot_state_publisher,
        diff_drive_spawner,
        turret_controller_spawner,
    ]
    
    # Launch them all!
    return LaunchDescription(declared_arguments + nodes)
