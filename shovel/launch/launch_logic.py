from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='logic',
#            namespace='',
            name='logic',
            executable='logic_node'
#            node_executable='logic_node',
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
    ]
)
