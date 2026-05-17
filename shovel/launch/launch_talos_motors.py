from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='power_distribution_panel',
            name='power_distribution_panel',
            executable='power_distribution_panel_node',
            parameters=[
                {"can_interface": "can0"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'},
            respawn=True
        )
        ,
        Node(
            package='talon',
            name='Talon14',
            executable='talon_node',
            parameters=[
                {"motor_number": 14},
                {"diagnostics_port": 56715},
                {"invert_motor": True},
                {"speed_topic": "talon_14_speed"},
                {"info_topic": "talon_14_info"},
                {"position_topic": "talon_14_position"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 15},
                {"kill_key": 50},
                {"op_mode": 0},
                {"print_data": True},
                {"stop_topic": "talon_14_stop"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'},
            respawn=True
        )
        ,
        Node(
            package='talon',
            name='Talon16',
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
                {"print_data": True},
                {"can_interface": "can0"},
                {"stop_topic": "talon_16_stop"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'},
            respawn=True
        )
        ,
        Node(
            package='talon',
            name='Talon18',
            executable='talon_node',
            parameters=[
                {"motor_number": 18},
                {"diagnostics_port": 56714},
                {"invert_motor": False},
                {"speed_topic": "vibes_speed"},
                {"info_topic": "talon_18_info"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 15},
                {"kill_key": 52},
                {"op_mode": 0},
                {"print_data": True},
                {"can_interface": "can0"},
                {"stop_topic": "talon_16_stop"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'},
            respawn=True
        )
    ]
)
