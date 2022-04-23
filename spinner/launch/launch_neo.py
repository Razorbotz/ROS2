from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='neo',
#            namespace='',
            name='neo_node',
#            executable='neo_node',
            node_executable='neo_node'
            parameters=[
                {"motor_number_front":10},
                {"motor_number_back":11},
                {"speed_topic":"drive_right_speed"},
                {"info_topic":"neo_10_info"},
                {"invert_motor":True},
                {"use_velocity":False},
                {"velocirty_multiplier":3000},
                {"test_speed":100}
                {"kP":0.1},
                {"kI":0.0001},
                {"kD":1.0},
                {"kIz":0.0},
                {"kFF":0.0},
                {"kMinOutput":-1.0},
                {"kMaxOutput":1.0}
                ]
            output={'stderr': 'screen', 'stdout': 'screen'}

        )
    ]
)
