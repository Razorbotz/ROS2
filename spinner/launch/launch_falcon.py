from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talon',
#            namespace='',
            name='right_motors',
#            executable='talon_node',
            node_executable='talon_node',
            parameters=[
                {"motor_number": 10},
                {"motor_number2": 11},
                {"diagnostics_port": 72340},
                {"invert_motor": True},
                {"speed_topic": "drive_left_speed"},
                {"info_topic": "talon_10_info"},
                {"info_topic2": "talon_11_info"},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
            ]
        )
    ]
)
