import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


## @file
# AprilTag detection and localization launch file.
#
# Replaces the previous aruco_ros + aruco_bridge setup with AprilTag 3
# detection and a static TF publisher for known tag positions.
#
# Usage:
#   CPU detector (sim/dev):   ros2 launch . launch_apriltag.py
#   GPU detector (Orin):      ros2 launch . launch_apriltag.py use_gpu:=true
#
# Parameters:
#   use_gpu (bool, default false) — use isaac_ros_apriltag GPU backend
#   tag_family (string, default "36h11") — AprilTag family
#   tag_size (double, default 0.3) — tag size in meters
#   camera_name (string) — camera namespace
#   image_topic (string) — image topic under camera namespace
#
# The static TF publisher defines the known position of each tag in the
# arena map frame. When the detector sees a tag and publishes its pose
# relative to the camera, the TF tree becomes:
#   map -> tag_<id> (static, known)
#   camera_frame -> tag36h11:<id> (detected, from apriltag_ros)
#
# With both transforms available, any node can look up map -> base_link
# via the camera chain.


def generate_launch_description():
    launch_dir = os.getcwd()


    # =========================================================================
    #  Arguments
    # =========================================================================
    use_gpu_arg = DeclareLaunchArgument(
        'use_gpu', default_value='false',
        description='Use isaac_ros_apriltag GPU backend instead of CPU'
    )
    tag_family_arg = DeclareLaunchArgument(
        'tag_family', default_value='36h11',
        description='AprilTag family (36h11 is ArUco-compatible)'
    )
    tag_size_arg = DeclareLaunchArgument(
        'tag_size', default_value='0.3',
        description='Tag size in meters'
    )
    camera_name_arg = DeclareLaunchArgument(
        'camera_name', default_value='/zed2i/left',
        description='Camera namespace'
    )
    image_topic_arg = DeclareLaunchArgument(
        'image_topic', default_value='image_raw',
        description='Image topic name under camera namespace'
    )
    camera_frame_arg = DeclareLaunchArgument(
        'camera_frame', default_value='zed2i_left_optical_frame',
        description='Optical frame of the camera (check your Gazebo plugin / URDF)'
    )
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame', default_value='odom',
        description='Odometry frame'
    )

    use_gpu = LaunchConfiguration('use_gpu')
    tag_family = LaunchConfiguration('tag_family')
    tag_size = LaunchConfiguration('tag_size')
    camera_name = LaunchConfiguration('camera_name')
    image_topic = LaunchConfiguration('image_topic')
    camera_frame = LaunchConfiguration('camera_frame')
    odom_frame = LaunchConfiguration('odom_frame')

    is_gpu = IfCondition(PythonExpression(["'", use_gpu, "' == 'true'"]))
    is_cpu = UnlessCondition(PythonExpression(["'", use_gpu, "' == 'true'"]))

    return LaunchDescription([
        use_gpu_arg,
        tag_family_arg,
        tag_size_arg,
        camera_name_arg,
        image_topic_arg,
        camera_frame_arg,
        odom_frame_arg,

        # =================================================================
        #  CPU detector: christianrauch/apriltag_ros
        #  Publishes detected tag poses to /tf as tag36h11:<id> frames
        # =================================================================
        GroupAction(
            condition=is_cpu,
            actions=[
                LogInfo(msg='Using CPU AprilTag detector (apriltag_ros)'),
                Node(
                    package='apriltag_ros',
                    executable='apriltag_node',
                    name='apriltag_node',
                    remappings=[
                        ('image_rect', [camera_name, '/', image_topic]),
                        ('camera_info', [camera_name, '/camera_info']),
                    ],
                    parameters=[{
                        'image_transport': 'raw',
                        'family': tag_family,
                        'max_hamming': 0,
                        'detector.threads': 2,
                        'detector.decimate': 1.0,
                        'publish_tf': True, 
                        'tag.ids': [7, 11],
                        'tag.sizes': [0.3, 0.3]
                    }],
                    output='screen',
                ),
            ],
        ),

        # =================================================================
        #  GPU detector: isaac_ros_apriltag (Jetson/NVIDIA GPU)
        #  Publishes to /tag_detections topic
        # =================================================================
        GroupAction(
            condition=is_gpu,
            actions=[
                LogInfo(msg='Using GPU AprilTag detector (isaac_ros_apriltag)'),
                Node(
                    package='isaac_ros_apriltag',
                    executable='isaac_ros_apriltag',
                    name='apriltag_node',
                    remappings=[
                        ('image', [camera_name, '/', image_topic]),
                        ('camera_info', [camera_name, '/camera_info']),
                    ],
                    parameters=[{
                        'family': tag_family,
                        'size': tag_size,
                    }],
                    output='screen',
                ),
            ],
        ),

        # =================================================================
        #  AprilTag to EKF Translator
        # =================================================================
        ExecuteProcess(
            cmd=[
                'python3',
                os.path.join(launch_dir, 'src', 'apriltag', 'scripts', 'apriltag_to_ekf.py'),
                '--ros-args',
                '-p', 'use_sim_time:=true',
            ],
            output='screen',
        ),
    ])