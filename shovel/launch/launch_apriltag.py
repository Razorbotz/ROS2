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
                        'family': tag_family,
                        'size': 0.3,
                        'max_hamming': 0,
                        'detector.threads': 2,
                        'detector.decimate': 1.0,
                        'image_transport': 'raw',
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
        #  Static TF: known tag positions in the arena
        #
        #  This replaces aruco_bridge. Instead of computing the robot's
        #  position from the detected marker, we define where each tag IS
        #  in the map frame. The TF tree then provides map->base_link
        #  through the camera chain automatically.
        #
        #  To add a new tag, add another static_transform_publisher node
        #  with the tag's known position/orientation in the map frame.
        #
        #  Frame naming convention:
        #    apriltag_ros publishes: camera_optical_frame -> tag36h11:<id>
        #    We publish:             map -> tag36h11:<id> (static, known)
        #
        #  Arguments: x y z qx qy qz qw parent_frame child_frame
        # =================================================================

        # Tag ID 7 — on the arena wall
        # Pose from artemis_arena.world: (3.4, 1.8, 0.4) rpy(0, -1.58, 0)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tag7_static_tf',
            arguments=[
                '--x', '3.4', '--y', '1.8', '--z', '0.4',
                '--qx', '0.0', '--qy', '-0.7068', '--qz', '0.0', '--qw', '0.7074',
                '--frame-id', 'map', '--child-frame-id', 'tag36h11:7',
            ],
            output='screen',
        ),

        # Tag ID 11 — second tag in the arena
        # Pose from artemis_arena.world: (2.5, 2.5, 0.4) rpy(1.58, 1.58, 0)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tag11_static_tf',
            arguments=[
                '--x', '2.5', '--y', '2.5', '--z', '0.4',
                '--qx', '0.5', '--qy', '0.5', '--qz', '0.5', '--qw', '0.5',
                '--frame-id', 'map', '--child-frame-id', 'tag36h11:11',
            ],
            output='screen',
        ),

        # =================================================================
        #  Localization: compute map -> odom from AprilTag detections
        #
        #  apriltag_ros publishes camera_optical_frame -> tag36h11:N
        #  Static TFs above define map -> tag36h11:N
        #  URDF + odometry provides odom -> ... -> camera_optical_frame
        #
        #  This node combines them to publish map -> odom.
        # =================================================================
        ExecuteProcess(
            cmd=[
                'python3',
                os.path.join(launch_dir, 'src', 'apriltag', 'scripts', 'apriltag_localization_node.py'),
                '--ros-args',
                '-p', 'tag_frame:=tag36h11:7',
                '-p', ['camera_frame:=', camera_frame],
                '-p', ['odom_frame:=', odom_frame],
                '-p', 'map_frame:=map',
                '-p', 'publish_rate:=10.0',
            ],
            output='screen',
        ),

        # =================================================================
        #  Tag Visualization: publishes MarkerArray on /apriltag_markers
        #  so known tag positions are visible in Foxglove / RViz.
        #  Add a Marker panel in Foxglove subscribed to /apriltag_markers.
        # =================================================================
        ExecuteProcess(
            cmd=[
                'python3',
                os.path.join(launch_dir, 'src', 'apriltag', 'scripts', 'tag_visualizer_node.py'),
                '--ros-args',
                '-p', 'map_frame:=map',
                '-p', 'tag_frames:=[tag36h11:7, tag36h11:11]',
            ],
            output='screen',
        ),
    ])