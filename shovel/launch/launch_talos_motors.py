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
                {"print_data": False},
                {"can_interface": "can0"},
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
                {"print_data": False},
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
                {"print_data": False},
                {"can_interface": "can0"},
                {"stop_topic": "talon_16_stop"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'},
            respawn=True
        )
        ,
        Node(
            package='kraken',
            name='Kraken10',
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
	    ,
        Node(
            package='kraken',
            name='Kraken11',
            executable='kraken_node',
            parameters=[
                {"motor_number": 11},
                {"diagnostics_port": 72341},
                {"invert_motor": True},
                {"speed_topic": "falcon_11_speed"},
                {"user_topic": "falcon_11_user_speed"},
                {"reset_topic": "2"},
                {"info_topic": "talon_11_info"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 15},
                {"kill_key": 55},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can0"},
                {"stop_topic": "falcon_11_stop"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'},
            respawn=True
        )
        ,
        Node(
            package='kraken',
            name='Kraken12',
            executable='kraken_node',
            parameters=[
                {"motor_number": 12},
                {"diagnostics_port": 72342},
                {"invert_motor": False},
                {"speed_topic": "falcon_12_speed"},
                {"user_topic": "falcon_12_user_speed"},
                {"reset_topic": "3"},
                {"info_topic": "talon_12_info"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 15},
                {"kill_key": 56},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can0"},
                {"stop_topic": "falcon_12_stop"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'},
            respawn=True
        )
	    ,
        Node(
            package='kraken',
            name='Kraken13',
            executable='kraken_node',
            parameters=[
                {"motor_number": 13},
                {"diagnostics_port": 72343},
                {"invert_motor": True},
                {"speed_topic": "falcon_13_speed"},
                {"user_topic": "falcon_13_user_speed"},
                {"reset_topic": "4"},
                {"info_topic": "talon_13_info"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 15},
                {"kill_key": 57},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can0"},
                {"stop_topic": "falcon_13_stop"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'},
            respawn=True
        )
    ]
)
