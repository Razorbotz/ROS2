from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
       Node(
            package='excavation',
            name='excavation',
            executable='excavation_node',
            parameters=[
                {"single_arm": True},
                {"actuator_mode": "2_arm_actuator"}
            ],
            respawn=True
        )
    ]
)
