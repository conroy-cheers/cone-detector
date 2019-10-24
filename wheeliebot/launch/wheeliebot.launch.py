import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
import launch.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package='wheeliebot',
            node_executable='wheeliebot_node',
            node_name='wheeliebot_node',
            arguments=[],
            output='screen'),
        launch_ros.actions.Node(
            package='cone_detector',
            node_executable='cone_detector',
            node_name='cone_detector',
            arguments=[],
            output='screen'
        ),
        launch_ros.actions.Node(
            package='wheelie_serial',
            node_executable='wheelie_serial_node',
            node_name='wheelie_serial_node',
            arguments=[],
            output='screen'
        ),
    ])
