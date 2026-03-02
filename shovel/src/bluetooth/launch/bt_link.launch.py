from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    peer_mac = LaunchConfiguration("peer_mac")
    channel = LaunchConfiguration("channel")
    enable_stdio = LaunchConfiguration("enable_stdio")

    return LaunchDescription([
        DeclareLaunchArgument("peer_mac", default_value="", description="Peer Bluetooth MAC (controller MAC)"),
        DeclareLaunchArgument("channel", default_value="1", description="RFCOMM channel"),
        DeclareLaunchArgument("enable_stdio", default_value="false", description="Enable stdin manual testing"),

        Node(
            package="bt_rfcomm_link",
            executable="bt_server_node.py",
            name="bt_server",
            output="screen",
            parameters=[{
                "peer_mac": peer_mac,
                "channel": channel,
                "enable_stdio": enable_stdio,
                # allow_incoming stays True; client tie-break ensures only one dialer
                "allow_incoming": True,
            }],
        ),

        Node(
            package="bt_rfcomm_link",
            executable="bt_client_node.py",
            name="bt_client",
            output="screen",
            parameters=[{
                "peer_mac": peer_mac,
                "channel": channel,
                "enable_stdio": enable_stdio,
            }],
        ),
    ])