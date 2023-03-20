from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='excavation',
            name='excavation',
            executable='excavation_node',
            output={'stderr': 'screen', 'stdout': 'screen'}
        ),
        Node(
            package='test',
            name='test',
            executable='test_node'
        )
    ]
)