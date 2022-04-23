from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zed_tracking',
            name='zed_tracking',
            executable='zed_tracking_node'
        )
    ]
)
