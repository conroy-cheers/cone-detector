import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
import launch.actions


def generate_launch_description():
    enable_align_depth = launch.substitutions.LaunchConfiguration('enable_aligned_depth', default="false")
    output_frame = launch.substitutions.LaunchConfiguration('output_frame', default="base_scan")
    range_max = launch.substitutions.LaunchConfiguration('range_max', default="2.0")
    range_min = launch.substitutions.LaunchConfiguration('range_min', default="0.2")
    return LaunchDescription([
        launch_ros.actions.Node(
            package='image_transport',
            node_executable='republish',
            node_name='transport_republish',
            arguments=["compressed", "in:=/robot/image", "raw", "out:=/image"],
            output='screen'),
        launch_ros.actions.Node(
            package='image_tools',
            node_executable='showimage',
            node_name='stream_showimage',
            arguments=[],
            output='screen'
        )
    ])