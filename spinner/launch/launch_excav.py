from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
       Node(
            package='excavation',
            name='excavation',
            node_executable='excavation_node'
        )
    ]
)
