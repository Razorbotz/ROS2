from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='autonomy',
            name='autonomy',
            executable='autonomy_node',
            parameters=[
                {"map": "NASA"},
                {"turnLeft": True},
                {"print_data": True}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
    ]
)
