from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='video_streaming',
            name='video_streaming',
            executable='video_streaming_node'
        )
    ]
)
