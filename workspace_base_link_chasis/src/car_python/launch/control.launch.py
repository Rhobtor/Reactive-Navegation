from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['steering_controller_front'],
        ),
        Node(
            package='car',
            executable='control_node.py',
            name='control_node',
        ),
    ])
