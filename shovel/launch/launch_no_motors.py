from launch import LaunchDescription
from launch_ros.actions import Node

## @file
# Launch file that contains all the nodes necessary
# to run the robot

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='logic',
            name='logic',
            executable='logic_node',
            parameters=[
                {"map": "lab"},
                {"xOffset": 1.4},
                {"turnLeft": True}
            ],
            output={'stderr': 'screen', 'stdout': 'screen'}
        )
        ,
        Node(
            package='communication',
            name='communication',
            executable='communication_node',
            parameters=[
                {"robot_name": "Shovel"}
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
            package='excavation',
            name='excavation',
            executable='excavation_node'
        )
        ,
        Node(
            package='zed_tracking',
            name='zed_tracking',
            executable='zed_tracking_node',
            parameters=[
                {"resolution": "VGA"}
            ]
        )
        ,
        Node(
            package='video_streaming',
            name='video_streaming',
            executable='video_streaming_node'
        )
    ]
)
