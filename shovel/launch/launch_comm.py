from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='communication',
            namespace='',
            name='communication',
            executable='communication_node',
            parameters=[
                {"robot_name": "Shovel"}
            ]
        )
    ]
)
