"""Suivi des flèches blanches ; la sécurité LiDAR est dans la brique control."""
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = get_package_share_directory('robocar_bringup') + '/config/autonomy.yaml'
    use_behavioral_cloning = LaunchConfiguration('use_behavioral_cloning')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_behavioral_cloning',
            default_value=EnvironmentVariable(
                'ROBOCAR_BEHAVIORAL_CLONING', default_value='false'
            ),
        ),
        Node(
            package='robocar_autonomy',
            executable='arrow_follow_node',
            name='arrow_follow_node',
            parameters=[config],
            output='screen',
            condition=UnlessCondition(use_behavioral_cloning),
        ),
        Node(
            package='robocar_autonomy',
            executable='behavioral_cloning_node',
            name='behavioral_cloning_node',
            parameters=[config],
            output='screen',
            condition=IfCondition(use_behavioral_cloning),
        ),
    ])
