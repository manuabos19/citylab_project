import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('robot_patrol'),
        'rviz',
        'patrol.rviz'
    )

    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='direction_service_executable',
            name='direction_service',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='robot_patrol',
            executable='patrol_with_service_executable',
            name='patrol_with_service',
            output='screen',
            emulate_tty=True
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        ),
    ])