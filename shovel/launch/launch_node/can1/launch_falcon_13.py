from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='falcon',
            name='Falcon13',
            executable='falcon_node',
            parameters=[
                {"motor_number": 13},
                {"diagnostics_port": 72343},
                {"invert_motor": True},
                {"speed_topic": "drive_left_speed"},
                {"user_topic": "user_left_speed"},
                {"reset_topic": "4"},
                {"info_topic": "talon_13_info"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 100},
                {"kill_key": 57},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can1"}
            ]
        )
    ]
)
