from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
import os
from launch.actions import TimerAction

def generate_launch_description():
    # Paths
    pkg_path = '/home/team/SoftwareDevelopment/ROS2/shovel/src/sim'
    world_path = os.path.join(pkg_path, 'worlds', 'high_resolution', 'artemis', 'artemis_arena.world')
    urdf_path = os.path.join(pkg_path, 'urdf', 'my_robot_tf.urdf')
    model_sdf_path = '/home/team/SoftwareDevelopment/ROS2/shovel/src/sim/models/model/model.sdf'

    return LaunchDescription([
        # Ensure Gazebo finds models
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=os.path.join(pkg_path, 'models')),

        # Robot state publisher (TFs)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        # Static transform odom -> base_link (optional redundancy)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # Launch Gazebo with your world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot into Gazebo
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'my_robot',
                '-file', model_sdf_path,
                '-x', '1.5', '-y', '1.5', '-z', '0.2'
            ],
            output='screen'
        ),

        TimerAction(
            period=10.0,  # wait 10 seconds for Gazebo to publish
            actions=[
                Node(
                    package='rtabmap_ros',
                    executable='rtabmap',
                    name='rtabmap',
                    output='screen',
                    parameters=[{
                        'frame_id': 'base_link',
                        'odom_frame_id': 'odom',
                        'map_frame': 'map',
                        'subscribe_rgb': False,
                        'subscribe_rgbd': False,
                        'subscribe_stereo': False,
                        'subscribe_depth': False,
                        'subscribe_scan': False,
                        'subscribe_scan_cloud': True,
                        'scan_cloud_topic': '/my_robot/d455i/points',
                        'use_sim_time': True,
                        'publish_tf': True,
                        'publish_map_tf': True,
                        'queue_size': 10,
                        'approx_sync': True
                    }],
                    remappings=[
                        ('odom', '/my_robot/odom'),
                        ('scan_cloud', '/my_robot/d455i/points')
                    ]
                )
            ]
        )

    ])
