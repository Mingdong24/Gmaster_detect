from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='detect',
            executable='detect_node',
            name='detect_node'
        )
    ])