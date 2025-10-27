from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Paths
    pkg_path = '/home/team/SoftwareDevelopment/ROS2/shovel/src/sim'
    world_path = os.path.join(pkg_path, 'worlds', 'high_resolution', 'artemis', 'artemis_arena.world')
    urdf_path = os.path.join(pkg_path, 'urdf', 'my_robot_tf.urdf')
    model_sdf_path = '/home/team/SoftwareDevelopment/ROS2/shovel/src/sim/models/model/model.sdf'

    return LaunchDescription([
        # Ensure Gazebo finds models
        SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=os.path.join(pkg_path, 'models')),

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),

        # Static transform odom -> base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_odom_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # Launch Gazebo
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world_path, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn robot
        ExecuteProcess(
            cmd=[
                'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                '-entity', 'my_robot',
                '-file', model_sdf_path,
                '-x', '1.5', '-y', '1.5', '-z', '0.2'
            ],
            output='screen'
        ),

        # RTAB-Map node (subscribe to reliable relay)
        TimerAction(
            period=10.0,  # wait for Gazebo to start publishing
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
                        'use_sim_time': True,
                        'publish_tf': True,
                        'publish_map_tf': True,

                        # --- Enable RGB-D input ---
                        'subscribe_rgbd': True,
                        'subscribe_depth': True,
                        'subscribe_rgb': True,
                        'subscribe_scan_cloud': True,

                        # --- Topics ---
                        'rgb_topic': '/my_robot/d455i/color/image_raw',
                        'depth_topic': '/my_robot/d455i/depth/image_raw',
                        'camera_info_topic': '/my_robot/d455i/color/camera_info',
                        'scan_cloud_topic': '/my_robot/d455i/points_reliable',

                        # --- QoS settings ---
                        'qos_image': 'best_effort',
                        'qos_camera_info': 'best_effort',
                        'qos_scan_cloud': 'reliable',

                        # --- Mapping parameters ---
                        'queue_size': 10,
                        'approx_sync': True,
                        'RGBD/ProximityBySpace': 'true',
                        'RGBD/ProximityPathMaxNeighbors': '1',
                        'Reg/Strategy': '1',
                        'Mem/IncrementalMemory': 'true'
                    }],
                    remappings=[
                        ('odom', '/my_robot/odom'),
                        ('rgb/image', '/my_robot/d455i/color/image_raw'),
                        ('depth/image', '/my_robot/d455i/depth/image_raw'),
                        ('rgb/camera_info', '/my_robot/d455i/color/camera_info'),
                        ('scan_cloud', '/my_robot/d455i/points_reliable')
                    ]
                )
            ]
        )
    ])
