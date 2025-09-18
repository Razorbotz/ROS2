from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talon',
            name='bucket_2',
            executable='talon_node',
            parameters=[
                {"motor_number": 17},
                {"diagnostics_port": 56712},
                {"invert_motor": True},
                {"speed_topic": "talon_17_speed"},
                {"info_topic": "talon_17_info"},
                {"position_topic": "talon_17_position"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 15},
                {"kill_key": 53},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can1"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
    ]
)
