from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drivetrain',
            name='drivetrain',
            executable='drivetrain_node',
            respawn=True,
            parameters=[{
                'wheel_diameter': 0.2,
                'gear_reduction': 100.0,
                'track_width': 0.6,
                'motor0_type': 'phoenix6',
                'motor1_type': 'phoenix6',
                'motor2_type': 'phoenix6',
                'motor3_type': 'phoenix6',
                'use_cmd_vel': False,      # set True when ready for Nav2
            }]
        )
    ]
)
