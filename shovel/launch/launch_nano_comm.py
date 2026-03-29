from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    role_arg = DeclareLaunchArgument(
        'role', default_value='nano',
        description='Communication node role: orin, nano, or sim'
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name', default_value='Sierra',
        description='Robot name for network broadcast'
    )
    local_arg = DeclareLaunchArgument(
        'local', default_value='false',
        description='Use localhost for Aegis networking'
    )
    motor10_type_arg = DeclareLaunchArgument(
        'motor10_type', default_value='falcon',
        description='Motor 10 type: falcon or kraken'
    )
    motor11_type_arg = DeclareLaunchArgument(
        'motor11_type', default_value='falcon',
        description='Motor 11 type: falcon or kraken'
    )
    motor12_type_arg = DeclareLaunchArgument(
        'motor12_type', default_value='falcon',
        description='Motor 12 type: falcon or kraken'
    )
    motor13_type_arg = DeclareLaunchArgument(
        'motor13_type', default_value='falcon',
        description='Motor 13 type: falcon or kraken'
    )

    return LaunchDescription([
        role_arg,
        robot_name_arg,
        local_arg,
        motor10_type_arg,
        motor11_type_arg,
        motor12_type_arg,
        motor13_type_arg,

        Node(
            package='communication',
            name='communication',
            executable='communication_node',
            parameters=[{
                'role': LaunchConfiguration('role'),
                'robot_name': LaunchConfiguration('robot_name'),
                'local': LaunchConfiguration('local'),
                'motor10_type': LaunchConfiguration('motor10_type'),
                'motor11_type': LaunchConfiguration('motor11_type'),
                'motor12_type': LaunchConfiguration('motor12_type'),
                'motor13_type': LaunchConfiguration('motor13_type'),
            }],
            output='screen',
            respawn=True,
        ),
    ])