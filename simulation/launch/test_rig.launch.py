import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

def generate_launch_description():
    pkg_name = 'sim'
    pkg_path = get_package_share_directory(pkg_name)

    # 1. Critical Environment Variables (Fixes the hanging/empty world issue)
    env_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value='/usr/share/gazebo-11/models:' + os.path.join(pkg_path, 'models')
    )
    
    env_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value='/usr/share/gazebo-11:' + os.path.join(pkg_path, 'worlds')
    )

    # 2. Start Gazebo explicitly with the ROS 2 Factory Plugins
    world_file_path = os.path.join(pkg_path, 'worlds', 'high_resolution', 'artemis', 'artemis_arena.world')
    

    # 3. Process the Xacro file
    xacro_file = os.path.join(pkg_path, 'urdf', 'camera.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description = {'robot_description': doc.toxml()}

    # 4. Start Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 5. Headless Joint State Publisher
    node_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'zeros.pitch_adjustment_joint': 0.69,  # Maximum downward tilt
            'zeros.height_adjustment_joint': 1.0
        }]
    )

    # 6. Spawn the floating rig into Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'ghost_camera_rig',
                   '-x', '2.0', '-y', '2.0', '-z', '1.0'],
        output='screen'
    )

    return LaunchDescription([
        env_model_path,
        env_resource_path,
        node_robot_state_publisher,
        node_joint_state_publisher,
        spawn_entity
    ])