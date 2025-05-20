from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='falcon',
            name='Falcon10',
            executable='falcon_node',
            parameters=[
                {"motor_number": 10},
                {"diagnostics_port": 72340},
                {"invert_motor": False},
                {"speed_topic": "drive_right_speed"},
                {"user_topic": "user_right_speed"},
                {"reset_topic": "1"},
                {"info_topic": "talon_10_info"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 100},
                {"kill_key": 54},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can0"}
            ]
        )
    ]
)
