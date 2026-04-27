"""
launch_recorder.py
 
CPU-minimal telemetry recorder for the Jetson Orin Nano.
 
Design priorities (in order):
    1. Minimize CPU usage during recording
    2. Capture enough fidelity for post-run analysis and ML training
    3. Keep storage reasonable (it's the third priority - 500GB SSD has room)
 
Key choices:
    - No compression during record. zstd would dominate CPU. Compress
      offline after the run if you want smaller archives.
    - Cameras/lidar are NOT recorded here - capture those client-side.
    - Heartbeats are NOT recorded - infer node liveness from gaps in
      the status streams.
    - Commanded speeds and motor status both recorded so you can build
      command-response pairs for ML.
 
Expected size: ~100-200 MB for a 30-min run, uncompressed.
Expected CPU:  <2% of one core on the Orin Nano.
 
Usage (standalone):
    ros2 launch . launch_recorder.py robot:=talos
 
Usage (from launch.py): see snippet at the bottom of this file.
"""
 
import os
from datetime import datetime
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
 
 
# =============================================================================
#  Topic groups
# =============================================================================
#
# `ros2 bag record` silently ignores topics that don't exist on a given
# robot, so it's safe to list the union of every motor across all configs.
# Adjust the numeric ranges below to match your actual motor IDs.
 
# Motor telemetry. This is the bulk of the data - and the bulk of the value.
# Capture rate is controlled by `publishing_delay` in each motor node.
# For minimum CPU, set publishing_delay >= 40 (ms) -> 25 Hz.
MOTOR_INFO_TOPICS = (
    [f'/talon_{i}_info' for i in range(1, 14)]
    + [f'/kraken_{i}_info' for i in range(1, 14)]
    + [f'/falcon_{i}_info' for i in range(1, 14)]
)
 
# Commanded speeds. These are event-driven (only publish on joystick
# change), so they cost almost nothing to record but capture every
# operator intent transition.
COMMAND_TOPICS = [
    '/drive_left_speed', '/drive_right_speed',
    '/user_left_speed', '/user_right_speed',
    '/arm_speed', '/bucket_speed',
]
 
# Operator inputs. Event-driven, low rate.
OPERATOR_TOPICS = [
    '/joystick_axis', '/joystick_button', '/joystick_hat', '/key',
]
 
# State transitions - very low rate, high information value.
STATE_TOPICS = [
    '/STOP', '/GO', '/reset_topic',
]
 
# Localization output. The EKF/AprilTag/ZED pose is what makes this
# data useful for replay and ML - without pose you can't reconstruct
# where the robot was. Adjust topic names to match your actual setup.
LOCALIZATION_TOPICS = [
    '/odometry/filtered',
    '/apriltag_pose',
    '/zed_position',
    '/tf', '/tf_static',
]
 
 
def _build_record_command(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    log_dir = LaunchConfiguration('log_dir').perform(context)
 
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    output_path = os.path.join(log_dir, f'{robot}_{timestamp}')
 
    topics = (
        MOTOR_INFO_TOPICS
        + COMMAND_TOPICS
        + OPERATOR_TOPICS
        + STATE_TOPICS
        + LOCALIZATION_TOPICS
    )
 
    cmd = [
        'ros2', 'bag', 'record',
        '-o', output_path,
        '--storage', 'mcap',
        # No compression during record - the CPU savings dwarf the disk cost.
        # Compress offline with `mcap compress` or `zstd` after the run.
 
        # Modest cache. Smaller = less RAM held, faster crash recovery,
        # and on SSD the write batching benefit drops off quickly above ~10 MB.
        '--max-cache-size', '10000000',
 
        # No rotation - one file per run keeps things simple.
        '--max-bag-size', '0',
 
        *topics,
    ]
 
    return [
        LogInfo(msg=f'[recorder] Writing to: {output_path}'),
        LogInfo(msg=f'[recorder] Recording {len(topics)} topic patterns, no compression'),
        ExecuteProcess(cmd=cmd, output='screen'),
    ]
 
 
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', default_value='talos',
            description='Robot name - used in the output directory name'
        ),
        DeclareLaunchArgument(
            'log_dir', default_value=os.path.expanduser('~/robot_logs'),
            description='Parent directory for run recordings'
        ),
        DeclareLaunchArgument(
            'enable_recording', default_value='true',
            description='Master switch - set to false to skip recording entirely.'
        ),
        OpaqueFunction(
            function=_build_record_command,
            condition=IfCondition(LaunchConfiguration('enable_recording')),
        ),
    ])
 