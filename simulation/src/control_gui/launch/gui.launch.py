from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='control_gui',
            executable='control_gui',
            name='control_gui',
            output='screen'
        )
    ])