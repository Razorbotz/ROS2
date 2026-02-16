from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='communication',
            name='communication',
            executable='communication_node',
            parameters=[
                {"robot_name": "Shovel"},
                {"debug": False},
                {"local": True}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'},
            respawn=True
        )
    ]
)
