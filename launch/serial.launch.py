from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    node = Node(
            name='robot_comm_serial',
            package='robot_comm_serial',
            executable='robot_comm_serial',
            # namespace='robot_1',
            output='screen',
            parameters=[
            PathJoinSubstitution([get_package_share_directory(
                'robot_comm_serial'), 'config', 'robot_serial.yaml'])
            ]
    )

    ld = LaunchDescription([node])
    return ld
