from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='talon',
#            namespace='',
            name='front_left_motor',
            executable='talon_node',
#            node_executable='talon_node',
            parameters=[
                {"motor_number": 10},
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
            executable='talon_node',
#            node_executable='talon_node',
            parameters=[
                {"motor_number": 11},
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
            package='logic',
#            namespace='',
            name='logic',
            executable='logic_node',
#            node_executable='logic_node',
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='communication',
#            namespace='',
            name='communication',
            executable='communication_node',
#            node_executable='communication_node',
            parameters=[
                {"robot_name": "Skinny"}
            ]
        )
#        ,
#        Node(
#            package='power_distribution_panel',
##            namespace='',
#            name='power_distribution_panel',
#            executable='power_distribution_panel_node'
##            node_executable='power_distribution_panel_node'
#        )
##        ,
#        Node(
#            package='victor',
#            executable='victor_node',
#            parameters=[
#                {"motor_number": 14},
#                {"info_topic": "victor_14_info"},
#                {"speed_topic": "victor_speed"},
#                {"invert_motor": False}
#            ]
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
