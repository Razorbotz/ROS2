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
            package='kraken',
            name='Kalcon10',
            executable='kraken_node',
            parameters=[
                {"motor_number": 10},
                {"diagnostics_port": 72340},
                {"invert_motor": False},
                {"speed_topic": "falcon_10_speed"},
                {"user_topic": "falcon_10_user_speed"},
                {"reset_topic": "1"},
                {"info_topic": "talon_10_info"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 15},
                {"kill_key": 54},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can0"},
                {"stop_topic": "falcon_10_stop"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'},
            respawn=True
        )
    ]
)
