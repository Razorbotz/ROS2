from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talon',
            name='bucket_1',
            executable='talon_node',
            parameters=[
                {"motor_number": 16},
                {"diagnostics_port": 56713},
                {"invert_motor": True},
                {"speed_topic": "talon_16_speed"},
                {"info_topic": "talon_16_info"},
                {"position_topic": "talon_16_position"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 15},
                {"kill_key": 52},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can0"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
    ]
)
