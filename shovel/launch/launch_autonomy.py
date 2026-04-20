from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_arg = DeclareLaunchArgument(
        'robot', default_value='talos',
        description='Robot name to determine autonomy logic'
    )

    return LaunchDescription([
        robot_arg,
        Node(
            package='autonomy',
            name='autonomy',
            executable='autonomy_node',
            parameters=[
                {"map": "NASA"},
                {"turnLeft": True},
                {"print_data": True},
                {"robot": LaunchConfiguration('robot')}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'},
            respawn=True
        )
    ])