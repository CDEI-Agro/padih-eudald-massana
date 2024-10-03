from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ouster",
            default_value="true"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ublox",
            default_value="true"
        )
    )

    # Initialize Arguments
    ouster_arg = LaunchConfiguration("ouster")
    ublox_arg = LaunchConfiguration("ublox")

    # Include the Ouster launch file
    ouster_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ouster_ros'), 'launch', 'driver.launch.py'])
        ]),
        launch_arguments = {'viz' : 'false'}.items(),
        condition=IfCondition(ouster_arg),
    )

    # Include the Ublox GNSS launch file
    ublox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('ublox_gps'), 'launch', 'ublox_gps_node-launch.py'])
        ]),
        condition=IfCondition(ublox_arg),
    )

    sensors = [
        ublox_launch,
        ouster_launch,
    ]

    # Launch them all!
    return LaunchDescription(declared_arguments + sensors)