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
    logic_launch_file = os.path.join(launch_dir, 'launch', 'launch_logic.py')
    comm_launch_file = os.path.join(launch_dir, 'launch', 'launch_comm.py')
    excav_launch_file = os.path.join(launch_dir, 'launch', 'launch_excav.py')
    cam_launch_file = os.path.join(launch_dir, 'launch', 'launch_cam.py')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(logic_launch_file)
        )
        ,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(comm_launch_file)
        )
        ,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(excav_launch_file)
        )
        ,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cam_launch_file)
        )
        ,
        Node(
            package='drivetrain',
            name='drivetrain',
            executable='drivetrain_node'
        )
        #,
        #ExecuteProcess(
        #   cmd=['ros2', 'bag', 'record', '-a'],
        #    output='screen'
        #)
    ]
)
