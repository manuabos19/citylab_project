import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='robot_patrol',
            executable='test_service_executable',
            name='test_service',
            output='screen',
            emulate_tty=True
        )
    ])