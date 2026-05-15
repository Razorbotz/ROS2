from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talon',
#            namespace='',
            name='front_left_motor',
            executable='talon_node',
#            node_executable='talon_node',
            parameters=[
                {"motor_number": 18},
                {"diagnostics_port": 72340},
                {"invert_motor": True},
                {"speed_topic": "vibes_speed"},
                {"info_topic": "talon_18_info"},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
            ]
        )
    ]
)
