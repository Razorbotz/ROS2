import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


## @file
# Launch file that contains all the nodes necessary
# to run the robot

def generate_launch_description():
    launch_dir = os.getcwd()

    gazebo_launch_path = os.path.join(launch_dir, 'launch', 'artemis_sim.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        )
    ]
)