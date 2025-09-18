from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='power_distribution_panel',
            name='power_distribution_panel',
            executable='power_distribution_panel_node',
            parameters=[
                {"can_interface": "can1"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='talon',
            name='arm_1',
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
                {"can_interface": "can1"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
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
                {"can_interface": "can1"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
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
                {"can_interface": "can1"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
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
        ,
        Node(
            package='falcon',
            name='Falcon10',
            executable='falcon_node',
            parameters=[
                {"motor_number": 10},
                {"diagnostics_port": 72340},
                {"invert_motor": False},
                {"speed_topic": "drive_right_speed"},
                {"info_topic": "talon_10_info"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 100},
                {"kill_key": 54},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can1"}
            ]
        )
	    ,
        Node(
            package='falcon',
            name='Falcon11',
            executable='falcon_node',
            parameters=[
                {"motor_number": 11},
                {"diagnostics_port": 72341},
                {"invert_motor": True},
                {"speed_topic": "drive_left_speed"},
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
        ,
        Node(
            package='falcon',
            name='Falcon12',
            executable='falcon_node',
            parameters=[
                {"motor_number": 12},
                {"diagnostics_port": 72342},
                {"invert_motor": False},
                {"speed_topic": "drive_right_speed"},
                {"info_topic": "talon_12_info"},
                {"kP": 10.0},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0},
                {"publishing_delay": 100},
                {"kill_key": 56},
                {"op_mode": 0},
                {"print_data": False},
                {"can_interface": "can1"}
            ]
        )
	    ,
        Node(
            package='falcon',
            name='Falcon13',
            executable='falcon_node',
            parameters=[
                {"motor_number": 13},
                {"diagnostics_port": 72343},
                {"invert_motor": True},
                {"speed_topic": "drive_left_speed"},
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
