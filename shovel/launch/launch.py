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
    autonomy_launch_file = os.path.join(launch_dir, 'launch', 'launch_autonomy.py')
    logic_launch_file = os.path.join(launch_dir, 'launch', 'launch_logic.py')
    comm_launch_file = os.path.join(launch_dir, 'launch', 'launch_comm.py')
    excav_launch_file = os.path.join(launch_dir, 'launch', 'launch_excav.py')
    cam_launch_file = os.path.join(launch_dir, 'launch', 'launch_cam.py')
    drivetrain_launch_file = os.path.join(launch_dir, 'launch', 'launch_drivetrain.py')
    status_monitor_launch_file = os.path.join(launch_dir, 'launch', 'launch_status_monitor.py')
    reset_launch_file = os.path.join(launch_dir, 'launch', 'launch_reset.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(motors_launch_file)
        )
        ,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(autonomy_launch_file)
        )
        ,
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
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(reset_launch_file)
        #)
        #,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drivetrain_launch_file)
        )
        ,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(status_monitor_launch_file)
        )
        #,
        #ExecuteProcess(
        #   cmd=['ros2', 'bag', 'record', '-a'],
        #    output='screen'
        #)
    ]
)
