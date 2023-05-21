from launch import LaunchDescription
from launch_ros.actions import Node

## @file
# Launch file that contains all the nodes necessary
# to run the robot

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arduino',
            name='arduino',
            executable='arduino_node'
        )
        ,
        Node(
            package='stepper',
            name='stepper',
            executable='stepper_node'
        )
        ,
        Node(
            package='logic',
            name='logic',
            executable='logic_node',
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='communication2',
            name='communication2',
            executable='communication2_node',
            parameters=[
                {"robot_name": "Scoop"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='power_distribution_panel',
            name='power_distribution_panel',
            executable='power_distribution_panel_node',
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='talon',
            name='shoulder_1',
            executable='talon_node',
            parameters=[
                {"motor_number": 14},
                {"diagnostics_port": 56715},
                {"invert_motor": True},
                {"speed_topic": "talon_14_speed"},
                {"info_topic": "talon_14_info"},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='talon',
            name='shoulder_2',
            executable='talon_node',
            parameters=[
                {"motor_number": 15},
                {"diagnostics_port": 56714},
                {"invert_motor": True},
                {"speed_topic": "talon_15_speed"},
                {"info_topic": "talon_15_info"},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='talon',
            name='dump',
            executable='talon_node',
            parameters=[
                {"motor_number": 16},
                {"diagnostics_port": 56713},
                {"invert_motor": True},
                {"speed_topic": "talon_16_speed"},
                {"info_topic": "talon_16_info"},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='talon',
            name='dump',
            executable='talon_node',
            parameters=[
                {"motor_number": 17},
                {"diagnostics_port": 56712},
                {"invert_motor": True},
                {"speed_topic": "talon_17_speed"},
                {"info_topic": "talon_17_info"},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='excavation',
            name='excavation',
            executable='excavation_node'
        )
        ,
        Node(
            package='falcon',
            name='Falcon10',
            executable='falcon_node',
            parameters=[
                {"motor_number": 10},
                {"diagnostics_port": 72340},
                {"invert_motor": True},
                {"speed_topic": "drive_left_speed"},
                {"info_topic": "talon_10_info"},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
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
                {"invert_motor": False},
                {"speed_topic": "drive_right_speed"},
                {"info_topic": "talon_11_info"},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
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
                {"invert_motor": True},
                {"speed_topic": "drive_left_speed"},
                {"info_topic": "talon_12_info"},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
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
                {"invert_motor": False},
                {"speed_topic": "drive_right_speed"},
                {"info_topic": "talon_13_info"},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kF": 0.0}
            ]
        )
        ,
#        Node(
#            package='zed_tracking',
#            name='zed_tracking',
#            executable='zed_tracking_node'
#        )
#        ,
        Node(
            package='neo',
            name='neo',
            executable='neo_node',
            parameters=[
                {"motor_number": 18},
                {"info_topic": "neo_out"},
                {"speed_topic": "neo_speed"}
            ]
        )
    ]
)
