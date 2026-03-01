from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='zed_tracking',
            name='zed_tracking',
            executable='zed_tracking_node',
            parameters=[
                {"xOffset": 1.25},
                {"kill_key": 58},
                {"resolution": "VGA"},
                {"print_data": True},
                {"aruco_marker_size_m": 0.125}
            ],
            respawn=True
        )
        ,
        Node(
            package='video_streaming',
            name='video_streaming',
            executable='video_streaming_node',
            respawn=True
        )
    ]
)
