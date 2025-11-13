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
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value='/usr/share/gazebo-11/models:' + os.path.join(pkg_path, 'models')
        ),

        SetEnvironmentVariable(
            name='GAZEBO_RESOURCE_PATH',
            value='/usr/share/gazebo-11:' + os.path.join(pkg_path, 'worlds')
        ),


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
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                        '-entity', 'my_robot',
                        '-file', model_sdf_path,
                        '-x', '1.5', '-y', '1.5', '-z', '0.2'
                    ],
                    output='screen'
                )
            ]
        )
    ])
