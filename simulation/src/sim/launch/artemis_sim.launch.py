from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Paths ---
    try:
        pkg_path = get_package_share_directory('sim')
    except Exception as e:
        print("Error: Package 'sim' not found. Did you run 'colcon build' and source your setup file?")
        raise e
    # Configs
    config_path = os.path.join(pkg_path, 'config', 'controllers.yaml')
    urdf_path = os.path.join(pkg_path, 'urdf', 'my_robot_tf.urdf')
    model_sdf_path = os.path.join(pkg_path, 'models', 'model', 'model.sdf')
    world_path = os.path.join(pkg_path, 'worlds', 'high_resolution', 'artemis', 'artemis_arena.world')

    # --- Environment Variables ---
    env_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value='/usr/share/gazebo-11/models:' + os.path.join(pkg_path, 'models')
    )
    
    env_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value='/usr/share/gazebo-11:' + os.path.join(pkg_path, 'worlds')
    )

    # 1. Robot State Publisher (Publishes URDF for ros2_control to find)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    # 2. Static TF
    node_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # 3. Gazebo
    cmd_gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', world_path, 
             '-s', 'libgazebo_ros_factory.so', 
             '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    # 4. Spawn Robot
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-file', model_sdf_path,
            '-x', '1.5', '-y', '1.5', '-z', '0.2',
            '-timeout', '120'
        ],
        output='screen',
    )

    # 5. Controller Spawners
    spawn_joint_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    spawn_falcon_10 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["falcon_10_controller", "--param-file", config_path]
    )

    spawn_falcon_11 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["falcon_11_controller", "--param-file", config_path]
    )

    spawn_falcon_12 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["falcon_12_controller", "--param-file", config_path]
    )

    spawn_falcon_13 = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["falcon_13_controller", "--param-file", config_path]
    )

    # --- Event Handlers ---
    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_spawn_entity,
            on_exit=[
                spawn_joint_broadcaster,
                spawn_falcon_10,
                spawn_falcon_11,
                spawn_falcon_12,
                spawn_falcon_13
            ],
        )
    )

    return LaunchDescription([
        env_model_path,
        env_resource_path,
        node_robot_state_publisher,
        node_static_tf,
        cmd_gazebo,
        node_spawn_entity,
        load_controllers, # This triggers the controllers sequentially
    ])