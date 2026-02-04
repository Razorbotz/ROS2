from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nano_communication',
            name='nano_communication',
            executable='nano_communication_node',
            parameters=[
                {"robot_name": "Shovel"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'},
            respawn=True
        )
    ]
)
