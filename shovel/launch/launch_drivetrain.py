from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
)

def generate_launch_description():
    robot = LaunchConfiguration('robot')

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='talos',
            description='Robot configuration to check if sim'
        ),
        DeclareLaunchArgument(
            'use_sim', default_value='false',
            description='Set true when running in Gazebo simulation'
        ),

        Node(
            package='drivetrain',
            name='drivetrain',
            executable='drivetrain_node',
            respawn=True,
            parameters=[{
                'wheel_diameter': 0.4,
                'gear_reduction': 100.0,
                'track_width': 0.85,
                'max_linear_speed': 0.65,
                'motor0_type': 'phoenix6',
                'motor1_type': 'phoenix6',
                'motor2_type': 'phoenix6',
                'motor3_type': 'phoenix6',
                'use_cmd_vel': False,
                'publish_odom': True,
                'use_sim': LaunchConfiguration('use_sim'),
            }]
        )
    ])