import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    LogInfo,
)
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


## @file
# Unified launch file for all robot configurations.
#
# Usage:
#   Talos hardware:      ros2 launch . launch.py robot:=talos
#   Sierra hardware:     ros2 launch . launch.py robot:=sierra
#   Sisyphus hardware:   ros2 launch . launch.py robot:=sisyphus
#   Simulation:          ros2 launch . launch.py robot:=sim
#
# The robot parameter controls:
#   - Which motor configuration to load (launch_<robot>_motors.py or unified sim)
#   - Whether Gazebo is launched
#   - Communication node role (orin/nano/sim)
#   - Which peripherals are started (RealSense, ArUco, etc.)
#   - BT wrapper target


def generate_launch_description():
    launch_dir = os.getcwd()

    # =========================================================================
    #  Launch arguments
    # =========================================================================
    robot_arg = DeclareLaunchArgument(
        'robot', default_value='talos',
        description='Robot configuration: talos, sierra, sisyphus, or sim'
    )
    role_arg = DeclareLaunchArgument(
        'role', default_value='orin',
        description='Communication node role: orin, nano, or sim'
    )
    print_data_arg = DeclareLaunchArgument(
        'print_data', default_value='false',
        description='Enable verbose logging'
    )
    use_foxglove_arg = DeclareLaunchArgument(
        'use_foxglove', default_value='true',
        description='Set to "true" to launch the Foxglove bridge, "false" to disable it.'
    )
    use_perception_arg = DeclareLaunchArgument(
        'use_perception', default_value='true',
        description='Set to "true" to launch the lunar perception node.'
    )

    robot = LaunchConfiguration('robot')
    role = LaunchConfiguration('role')
    use_foxglove = LaunchConfiguration('use_foxglove')
    use_perception = LaunchConfiguration('use_perception')

    # Convenience conditions
    is_sim = IfCondition(PythonExpression(["'", robot, "' == 'sim'"]))
    is_not_sim = UnlessCondition(PythonExpression(["'", robot, "' == 'sim'"]))
    is_sierra = IfCondition(PythonExpression(["'", robot, "' == 'sierra'"]))
    is_talos = IfCondition(PythonExpression(["'", robot, "' == 'talos'"]))
    is_sisyphus = IfCondition(PythonExpression(["'", robot, "' == 'sisyphus'"]))

    # =========================================================================
    #  Sub-launch file paths
    # =========================================================================
    autonomy_launch = os.path.join(launch_dir, 'launch', 'launch_autonomy.py')
    logic_launch = os.path.join(launch_dir, 'launch', 'launch_logic.py')
    comm_launch = os.path.join(launch_dir, 'launch', 'launch_comm.py')
    excav_launch = os.path.join(launch_dir, 'launch', 'launch_excav.py')
    cam_launch = os.path.join(launch_dir, 'launch', 'launch_cam.py')
    drivetrain_launch = os.path.join(launch_dir, 'launch', 'launch_drivetrain.py')
    status_monitor_launch = os.path.join(launch_dir, 'launch', 'launch_status_monitor.py')
    bt_wrapper_launch = os.path.join(launch_dir, 'launch', 'bt_config.py')
    gazebo_launch = os.path.join(launch_dir, 'launch', 'artemis_sim.launch.py')
    video_launch = os.path.join(launch_dir, 'launch', 'launch_video_streaming.py')
    lidar_launch = os.path.join(launch_dir, 'launch', 'launch_lidar.py')

    # Per-robot motor launch files (hardware)
    talos_motors_launch = os.path.join(launch_dir, 'launch', 'launch_talos_motors.py')
    sierra_motors_launch = os.path.join(launch_dir, 'launch', 'launch_sierra_motors.py')
    sisyphus_motors_launch = os.path.join(launch_dir, 'launch', 'launch_sisyphus_motors.py')

    # Unified sim motors launch file
    sim_motors_launch = os.path.join(launch_dir, 'launch', 'launch_motors.py')

    return LaunchDescription([
        robot_arg,
        role_arg,
        print_data_arg,
        use_foxglove_arg,
        use_perception_arg,

        LogInfo(msg=['Launching robot configuration: ', robot]),

        # =================================================================
        #  Gazebo (sim only)
        # =================================================================
        GroupAction(
            condition=is_sim,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(gazebo_launch),
                ),
            ],
        ),

        # =================================================================
        #  Motors — per-robot hardware configs
        # =================================================================
        GroupAction(
            condition=is_talos,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(talos_motors_launch),
                ),
            ],
        ),
        GroupAction(
            condition=is_sierra,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(sierra_motors_launch),
                ),
            ],
        ),
        GroupAction(
            condition=is_sisyphus,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(sisyphus_motors_launch),
                ),
            ],
        ),
        GroupAction(
            condition=is_sim,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(sim_motors_launch),
                    launch_arguments={
                        'use_sim': 'true',
                        'drive_motor_type': 'sim',
                    }.items(),
                ),
            ],
        ),

        # =================================================================
        #  Shared nodes (all configurations)
        # =================================================================
        
        # Lunar Perception Node
        Node(
            condition=IfCondition(use_perception),
            package='perception', 
            executable='perception_node',
            name='perception_node',
            output='screen',
            remappings=[
                ('/camera/points', '/my_robot/d455i/points') 
            ]
        ),

        # Foxglove Bridge Node
        Node(
            condition=IfCondition(use_foxglove),
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            parameters=[{
                'port': 8765,
                'address': '0.0.0.0'
            }]
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(autonomy_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(logic_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(comm_launch),
            launch_arguments={
                'role': PythonExpression([
                    "'sim' if '", robot, "' == 'sim' else '", role, "'"
                ]),
                'local': PythonExpression([
                    "'true' if '", robot, "' == 'sim' else 'false'"
                ]),
                'interface_name': PythonExpression([
                    "'eth1' if '", robot, "' == 'sim' else 'wlP1p1s0'"
                ]),
                'motor10_type': PythonExpression([
                    "'falcon' if '", robot, "' in ('sisyphus', 'sim') else 'kraken'"
                ]),
                'motor11_type': PythonExpression([
                    "'falcon' if '", robot, "' in ('sisyphus', 'sim') else 'kraken'"
                ]),
                'motor12_type': PythonExpression([
                    "'falcon' if '", robot, "' in ('sisyphus', 'sim') else 'kraken'"
                ]),
                'motor13_type': PythonExpression([
                    "'falcon' if '", robot, "' in ('sisyphus', 'sim') else 'kraken'"
                ]),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(excav_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(drivetrain_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(status_monitor_launch),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(video_launch),
            launch_arguments={
                'interface_name': PythonExpression([
                    "'wlP1p1s0' if '", robot, "' != 'sim' else 'eth1'"
                ]),
            }.items(),
        ),
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource(
        #        os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        #    ),
        #    launch_arguments={
        #        'use_sim_time': PythonExpression(["'true' if '", robot, "' == 'sim' else 'false'"]),
        #        'params_file': os.path.join(launch_dir, 'src', 'autonomy', 'config', 'nav2_params.yaml'), # <-- Update this path to where your yaml lives
        #        'map': '' # Leave blank if you are building the map dynamically
        #    }.items(),
        #),

        # =================================================================
        #  Cameras (hardware only)
        # =================================================================
        GroupAction(
            condition=is_not_sim,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(cam_launch),
                ),
            ],
        ),

        GroupAction(
            condition=is_not_sim,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(lidar_launch),
                ),
            ],
        ),

        # Sierra-specific: RealSense camera
        GroupAction(
            condition=is_sierra,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('realsense2_camera'),
                            'launch', 
                            'rs_launch.py'
                        ])
                    ),
                ),
            ],
        ),

        # =================================================================
        #  BT wrapper — target depends on robot
        # =================================================================
        # Talos and Sierra target Sisyphus
        GroupAction(
            condition=is_talos,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(bt_wrapper_launch),
                    launch_arguments={'target': 'Sisyphus'}.items(),
                ),
            ],
        ),
        GroupAction(
            condition=is_sierra,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(bt_wrapper_launch),
                    launch_arguments={'target': 'Sisyphus'}.items(),
                ),
            ],
        ),
        # Sisyphus targets Sierra
        GroupAction(
            condition=is_sisyphus,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(bt_wrapper_launch),
                    launch_arguments={'target': 'Sierra'}.items(),
                ),
            ],
        ),

        # =================================================================
        #  AprilTag detection + localization (sim only)
        # =================================================================
        GroupAction(
            condition=is_sim,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'launch', 'launch_apriltag.py')
                    ),
                    launch_arguments={
                        'use_gpu': 'false', 
                        'tag_family': '36h11',
                        'tag_size': '0.3',
                        'camera_name': '/zed2i/left',
                        'image_topic': 'image_raw',
                    }.items(),
                ),
            ],
        ),
        # =================================================================
        #  Robot Localization (Global EKF)
        # =================================================================
        Node(
            condition=is_sim,
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global_filter_node',
            output='screen',
            parameters=[
                os.path.join(launch_dir, 'src', 'autonomy', 'config', 'ekf_global.yaml'),
                {'use_sim_time': True}
            ],
            remappings=[
                ('pose0', '/apriltag_pose') 
            ]
        ),
    ])