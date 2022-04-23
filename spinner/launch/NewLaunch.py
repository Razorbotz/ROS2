from launch import LaunchDescription
from launch_ros.actions import Node

## @file
# Launch file that contains all the nodes necessary
# to run the robot

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talon',
#            namespace='',
            name='front_left_motor',
#            executable='talon_node',
            node_executable='talon_node',
            parameters=[
                {"motor_number": 10},
                {"diagnostics_port": 56710},
                {"invert_motor": False},
                {"speed_topic": "drive_right_speed"},
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
            package='talon',
#            namespace='',
            name='rear_left_motor',
#            executable='talon_node',
            node_executable='talon_node',
            parameters=[
                {"motor_number": 11},
                {"diagnostics_port": 56711},
                {"invert_motor": True},
                {"speed_topic": "drive_left_speed"},
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
            package='talon',
#            namespace='',
            name='front_right_motor',
#            executable='talon_node',
            node_executable='talon_node',
            parameters=[
                {"motor_number": 12},
                {"diagnostics_port": 56712},
                {"invert_motor": False},
                {"speed_topic": "drive_right_speed"},
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
            package='talon',
#            namespace='',
            name='rear_right_motor',
#            executable='talon_node',
            node_executable='talon_node',
            parameters=[
                {"motor_number": 13},
                {"diagnostics_port": 56713},
                {"invert_motor": True},
                {"speed_topic": "drive_left_speed"},
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
        Node(
            package='logic',
#            namespace='',
            name='logic',
#            executable='logic_node'
            node_executable='logic_node'
#            node_executable='logic_node',
#            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='communication',
#            namespace='',
            name='communication',
#            executable='communication_node',
            node_executable='communication_node',
            parameters=[
                {"robot_name": "Skinny"}
            ]
        )
        ,
        Node(
            package='power_distribution_panel',
#            namespace='',
            name='power_distribution_panel',
#            executable='power_distribution_panel_node'
            node_executable='power_distribution_panel_node'
        )
#        ,
#        Node(
#            package='autonomy',
#            name='autonomy',
#            node_executable='autonomy_node'
#        )
#        ,
#        Node(
#            package='zed_tracking',
#            name='zed_tracking',
#            node_executable='zed_tracking_node'
#        )
#        ,
#        Node(
#            package='excavation',
#            name='excavation',
#            node_executable='excavation_node'
#        )
    ]
)
