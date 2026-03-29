from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
import os
import tempfile
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # --- Paths ---
    try:
        pkg_path = get_package_share_directory('sim')
    except Exception as e:
        print("Error: Package 'sim' not found. Did you run 'colcon build' and source your setup file?")
        raise e
    
    config_path = os.path.join(pkg_path, 'config', 'controllers.yaml')
    urdf_path = os.path.join(pkg_path, 'urdf', 'my_robot_tf.urdf')
    model_sdf_path = os.path.join(pkg_path, 'models', 'model', 'model.sdf')
    world_path = os.path.join(pkg_path, 'worlds', 'high_resolution', 'artemis', 'artemis_arena.world')

    with open(model_sdf_path, 'r') as f:
        sdf_content = f.read()
        
    # --- Launch Arguments ---
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='true',
        description='Set to "true" to run gzserver only. Set to "false" to launch the Gazebo GUI.'
    )
    headless = LaunchConfiguration('headless')

    # --- Environment Variables ---
    env_gazebo_db = SetEnvironmentVariable(
        name='GAZEBO_MODEL_DATABASE_URI',
        value=''
    )
    
    env_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value='/usr/share/gazebo-11/models:' + os.path.join(pkg_path, 'models')
    )
    
    processed_sdf = sdf_content.replace('REPLACE_WITH_CONTROLLER_YAML', config_path)
    temp_sdf_path = os.path.join(tempfile.gettempdir(), 'processed_robot.sdf')
    with open(temp_sdf_path, 'w') as f:
        f.write(processed_sdf)
    
    env_resource_path = SetEnvironmentVariable(
        name='GAZEBO_RESOURCE_PATH',
        value='/usr/share/gazebo-11:' + os.path.join(pkg_path, 'worlds')
    )

    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(urdf_path).read(),
            'use_sim_time': True # Always true since this is a sim-only launch file
        }]
    )

    # 2. Static TF: Map -> World Bridge
    node_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_world',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'world']
    )

    # 3. Static TF: Wall Marker Position
    node_wall_marker = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_wall_marker',
        arguments=[
            '3.4', '1.8', '0.4',    
            '0', '-1.57', '0',      
            'map',                  
            'wall_marker_7'         
        ]
    )

    # 4A. Gazebo Headless (gzserver)
    cmd_gzserver = ExecuteProcess(
        condition=IfCondition(headless),
        cmd=['gzserver', '--verbose', world_path, 
             '-s', 'libgazebo_ros_factory.so', 
             '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    # 4B. Gazebo GUI (gazebo)
    cmd_gazebo_gui = ExecuteProcess(
        condition=UnlessCondition(headless),
        cmd=['gazebo', '--verbose', world_path, 
             '-s', 'libgazebo_ros_factory.so', 
             '-s', 'libgazebo_ros_init.so'],
        output='screen'
    )

    # 5. Spawn Robot 
    node_spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_robot',
            '-file', temp_sdf_path,
            '-x', '1.9', '-y', '1.75', '-z', '0.2'
        ],
        output='screen',
        prefix=['bash -c \'sleep 4; $0 $@\' '],
    )

    # 6. Controller Spawners
    spawners = [
        Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"]),
        Node(package="controller_manager", executable="spawner", arguments=["falcon_10_controller", "--param-file", config_path]),
        Node(package="controller_manager", executable="spawner", arguments=["falcon_11_controller", "--param-file", config_path]),
        Node(package="controller_manager", executable="spawner", arguments=["falcon_12_controller", "--param-file", config_path]),
        Node(package="controller_manager", executable="spawner", arguments=["falcon_13_controller", "--param-file", config_path]),
        Node(package="controller_manager", executable="spawner", arguments=["arm_position_controller", "--param-file", config_path]),
        Node(package="controller_manager", executable="spawner", arguments=["bucket_position_controller", "--param-file", config_path])
    ]

    # --- Event Handlers ---
    load_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=node_spawn_entity,
            on_exit=spawners,
        )
    )

    return LaunchDescription([
        headless_arg,
        env_gazebo_db,
        env_model_path,
        env_resource_path,
        node_robot_state_publisher,
        node_static_tf,     
        node_wall_marker,   
        cmd_gzserver,
        cmd_gazebo_gui,
        node_spawn_entity,
        load_controllers,
    ])