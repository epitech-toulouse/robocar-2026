"""Suivi des flèches blanches ; la sécurité LiDAR est dans la brique control."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory('robocar_bringup') + '/config/autonomy.yaml'
    return LaunchDescription([
        Node(
            package='robocar_autonomy',
            executable='arrow_follow_node',
            name='arrow_follow_node',
            parameters=[config],
            output='screen',
        ),
    ])
