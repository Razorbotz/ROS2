from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='logic',
            name='logic',
            executable='logic_node',
            parameters=[
                {"map": "NASA"},
                {"turnLeft": True}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
    ]
)
