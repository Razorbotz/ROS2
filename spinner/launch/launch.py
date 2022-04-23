from launch import LaunchDescription
from launch_ros.actions import Node

## @file
# Launch file that contains all the nodes necessary
# to run the robot

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='neo',
#            namespace='',
            name='neo_node',
            executable='neo_node',
            parameters=[
                {"motor_number_front":10},
                {"motor_number_back":11},
                {"speed_topic":"drive_right_speed"},
                {"info_topic":"neo_10_info"},
                {"invert_motor":True},
                {"use_velocity":False},
                {"velocirty_multiplier":3000},
                {"test_speed":100},
                {"kP":0.2},
                {"kI":0.0002},
                {"kD":2.0},
                {"kIz":0.0},
                {"kFF":0.0},
                {"kMinOutput":-1.0},
                {"kMaxOutput":1.0}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,   
        Node(
            package='neo',
#            namespace='',
            name='neo_node',
            executable='neo_node',
            parameters=[
                {"motor_number_front":12},
                {"motor_number_back":13},
                {"speed_topic":"drive_right_speed"},
                {"info_topic":"neo_10_info"},
                {"invert_motor":True},
                {"use_velocity":False},
                {"velocirty_multiplier":3000},
                {"test_speed":100},
                {"kP":0.2},
                {"kI":0.0002},
                {"kD":2.0},
                {"kIz":0.0},
                {"kFF":0.0},
                {"kMinOutput":-1.0},
                {"kMaxOutput":1.0}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,   
        Node(
            package='logic',
#            namespace='',
            name='logic',
            executable='logic_node',
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='communication2',
#            namespace='',
            name='communication2',
            executable='communication2_node',
            parameters=[
                {"robot_name": "Spinner"}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='power_distribution_panel',
#            namespace='',
            name='power_distribution_panel',
            executable='power_distribution_panel_node',
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='talon',
#            namespace='',
            name='shoulder',
            executable='talon_node',
            parameters=[
                {"motor_number": 15},
                {"diagnostics_port": 56715},
                {"invert_motor": True},
                {"speed_topic": "shoulder"},
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
#            namespace='',
            name='dump',
            executable='talon_node',
            parameters=[
                {"motor_number": 14},
                {"diagnostics_port": 56714},
                {"invert_motor": True},
                {"speed_topic": "dump_bin_speed"},
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
            package='excavation',
            name='excavation',
            executable='excavation_node'
        )
    ]
)
