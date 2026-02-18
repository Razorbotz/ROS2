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

    # Define the path to the included launch files
    autonomy_launch_file = os.path.join(launch_dir, 'launch', 'launch_autonomy.py')
    logic_launch_file = os.path.join(launch_dir, 'launch', 'launch_logic.py')
    comm_launch_file = os.path.join(launch_dir, 'launch', 'launch_comm.py')
    excav_launch_file = os.path.join(launch_dir, 'launch', 'launch_excav.py')
    cam_launch_file = os.path.join(launch_dir, 'launch', 'launch_cam.py')
    drivetrain_launch_file = os.path.join(launch_dir, 'launch', 'launch_drivetrain.py')
    status_monitor_launch_file = os.path.join(launch_dir, 'launch', 'launch_status_monitor.py')
#    reset_launch_file = os.path.join(launch_dir, 'launch', 'launch_reset.py')
    gazebo_launch_path = os.path.join(launch_dir, 'launch', 'artemis_sim.launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_path)
        )
        ,
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(motors_launch_file)
#        )
#        ,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(autonomy_launch_file)
        )
        ,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(logic_launch_file)
        )
        ,
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(comm_launch_file)
#        )
#        ,
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
#        ,
#        IncludeLaunchDescription(
#            PythonLaunchDescriptionSource(realsense_launch_file)
#        )
        ,
        Node(
            package='falcon', 
            executable='falcon_node',
            name='falcon_sim_node',
            output='screen'
        )
        ,
        Node(
            package='talon', 
            executable='talon_node',
            name='talon_sim_node',
            output='screen'
        )
        ,
        Node(
            package='aruco_ros',
            executable='single',
            name='aruco_single',
            parameters=[{
                'marker_id': 7,
                'marker_size': 0.3,
                'ref_frame': 'zed2i_left_optical_frame',
                'marker_frame': 'aruco_marker_frame',
                'camera_frame': 'zed2i_left_optical_frame',
            }],
            output='screen',
            remappings=[
                ('/image', '/zed2i/left/image_raw'),
                ('/camera_info', '/zed2i/left/camera_info'),
            ]
        )
        ,
        Node(
            package='aruco_bridge',
            executable='aruco_bridge_main',
            name='aruco_pose_localization',
            output='screen',
            parameters=[{
                'pose_topic': '/aruco_single/pose',
                'map_frame': 'map',
                'base_frame': 'base_link',
                'known_marker_frame': 'wall_marker_7',
            }]
        )
    ]
)