from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    log_level_arg = DeclareLaunchArgument(
                'log_level',
                default_value='info',
                description='Logging level (debug, info, warn, error, fatal)'
        )

    node = Node(
            name='robot_comm_serial',
            package='robot_comm_serial',
            executable='robot_comm_serial',
            # namespace='robot_1',
            output='screen',
                ros_arguments=['--log-level', LaunchConfiguration('log_level')],
            parameters=[
            PathJoinSubstitution([get_package_share_directory(
                'robot_comm_serial'), 'config', 'robot_serial.yaml'])
            ]
    )

    ld = LaunchDescription([log_level_arg, node])
    return ld
