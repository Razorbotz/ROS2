import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_name = 'sim'
    
    # 1. Start Gazebo with the untouched artemis_arena.world
    world_file_path = os.path.join(get_package_share_directory(pkg_name),  'worlds', 'high_resolution', 'artemis', 'artemis_arena.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )

    # 2. Process the Xacro file
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', 'camera.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # 3. Start Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 4. Start the GUI Sliders for live height/pitch tuning
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # 5. Spawn the floating rig into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'ghost_camera_rig',
                   '-x', '0.0', '-y', '0.0', '-z', '1.0'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        spawn_entity
    ])