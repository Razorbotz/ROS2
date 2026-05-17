from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    interface_name_arg = DeclareLaunchArgument(
        'interface_name', default_value='wlP1p1s0',
        description='Network interface for broadcast (eth1 on WSL)'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='Shovel',
        description='Robot name for broadcast'
    )
    port_arg = DeclareLaunchArgument(
        'port', default_value='31338',
        description='UDP port for video streaming'
    )
    zed_image_topic_arg = DeclareLaunchArgument(
        'zed_image_topic', default_value='/d455f/color/image_raw',
        description='Image topic for the ZED camera'
    )

    return LaunchDescription([
        interface_name_arg,
        robot_name_arg,
        port_arg,
        zed_image_topic_arg,

        Node(
            package='video_streaming',
            name='video_streaming',
            executable='video_streaming_node',
            parameters=[{
                'interface_name': LaunchConfiguration('interface_name'),
                'robot_name': LaunchConfiguration('robot_name'),
                'port': LaunchConfiguration('port'),
                'zed_image_topic': LaunchConfiguration('zed_image_topic'),
            }],
            remappings=[
                ('/zed_image', '/zed2i/left/image_raw')
            ],
            output='screen',
            respawn=True,
        ),
    ])