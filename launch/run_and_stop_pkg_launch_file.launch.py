from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='run_and_stop_pkg',
            executable='run_and_stop',
            output='screen'),
    ])