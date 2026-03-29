from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo
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

    use_gpu = LaunchConfiguration('use_gpu')
    tag_family = LaunchConfiguration('tag_family')
    tag_size = LaunchConfiguration('tag_size')
    camera_name = LaunchConfiguration('camera_name')
    image_topic = LaunchConfiguration('image_topic')

    is_gpu = IfCondition(PythonExpression(["'", use_gpu, "' == 'true'"]))
    is_cpu = UnlessCondition(PythonExpression(["'", use_gpu, "' == 'true'"]))

    return LaunchDescription([
        use_gpu_arg,
        tag_family_arg,
        tag_size_arg,
        camera_name_arg,
        image_topic_arg,

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
                        'size': tag_size,
                        'max_hamming': 0,
                        'detector.threads': 2,
                        'detector.decimate': 1.0,
                        'image_transport': 'raw',
                        'qos_overrides./image_rect.subscription.reliability': 'best_effort',
                        'qos_overrides./camera_info.subscription.reliability': 'best_effort',
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

        # Tag ID 7 — on the arena wall (same position as your old ArUco marker 7)
        # Adjust x, y, z and orientation to match your arena layout.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tag7_static_tf',
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '1.0',
                '--qx', '0.0', '--qy', '0.0', '--qz', '0.0', '--qw', '1.0',
                '--frame-id', 'map', '--child-frame-id', 'tag36h11:7',
            ],
            output='screen',
        ),

        # Add more tags as needed:
        # Tag ID 0 — example for a second tag
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='tag0_static_tf',
        #     arguments=[
        #         '5.0', '0.0', '1.0', '0.0', '0.0', '0.707', '0.707',
        #         'map', 'tag36h11:0',
        #     ],
        #     output='screen',
        # ),
    ])