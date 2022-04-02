from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='neo',
#            namespace='',
            name='left_motors',
#            executable='talon_node',
            node_executable='neo_node',
            parameters=[
                {"motor_number_front": 10},
                {"motor_nubmer_back": 11},
                {"speed_topic": "drive_left_speed"},
                {"info_topic": "neo_10_info"},
                {"invert_motor": True},
                {"use_velocity": False},
                {"velocity_multiplier": 3000},
                {"test_speed": 100},
                {"kP": 0.20},
                {"kI": 0.000001},
                {"kD": 0.000001},
                {"kIz": 0.0},
                {"kFF": 0.0},
                {"kMinOutput": 0.0},
                {"kMaxOutput": 0.0}        
                
            ]
        )
    ]
)
