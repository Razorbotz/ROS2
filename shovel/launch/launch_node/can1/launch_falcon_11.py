from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='falcon',
            name='Falcon11',
            executable='falcon_node',
            parameters=[
                {"motor_number": 11},
                {"diagnostics_port": 72341},
                {"invert_motor": True},
                {"speed_topic": "drive_left_speed"},
                {"user_topic": "user_left_speed"},
                {"reset_topic": "2"},
                {"info_topic": "talon_11_info"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 100},
                {"kill_key": 55},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can1"}
            ]
        )
    ]
)
