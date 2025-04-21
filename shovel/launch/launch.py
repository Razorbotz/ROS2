import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource



## @file
# Launch file that contains all the nodes necessary
# to run the robot

def generate_launch_description():
    launch_dir = os.getcwd()

    # Define the path to the included launch files
    motors_launch_file = os.path.join(launch_dir, 'launch', 'launch_motors.py')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(motors_launch_file)
        ),
        Node(
            package='logic',
            name='logic',
            executable='logic_node',
            parameters=[
                {"map": "NASA"},
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
                {"xOffset": 1.4},
                {"resolution": "VGA"}
            ]
        )
        ,
        Node(
            package='video_streaming',
            name='video_streaming',
            executable='video_streaming_node'
        )
        #,
        #ExecuteProcess(
        #   cmd=['ros2', 'bag', 'record', '-a'],
        #    output='screen'
        #)
    ]
)
