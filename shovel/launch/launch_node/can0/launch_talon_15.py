from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talon',
            name='arm_2',
            executable='talon_node',
            parameters=[
                {"motor_number": 15},
                {"diagnostics_port": 56714},
                {"invert_motor": True},
                {"speed_topic": "talon_15_speed"},
                {"info_topic": "talon_15_info"},
                {"position_topic": "talon_15_position"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 15},
                {"kill_key": 51},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can0"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
    ]
)
