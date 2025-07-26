from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
       Node(
            package='status_monitor',
            name='status_monitor',
            executable='status_monitor_node',
            parameters=[
                {"print_data": False}
            ],
        )
    ]
)
