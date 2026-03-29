from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare a launch argument to switch between hardware and simulation
    use_sim_arg = DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='Set to true for Gazebo simulation mode'
    )

    # Motor type for drive motors: "falcon", "kraken", or "sim"
    drive_motor_type_arg = DeclareLaunchArgument(
        'drive_motor_type', default_value='falcon',
        description='Drive motor type: falcon, kraken, or sim'
    )

    use_sim = LaunchConfiguration('use_sim')
    drive_motor_type = LaunchConfiguration('drive_motor_type')

    return LaunchDescription([
        use_sim_arg,
        drive_motor_type_arg,

        # =====================================================================
        #  Talon SRX Actuators (arm + bucket)
        # =====================================================================

        # Talon 14 — Arm actuator
        Node(
            package='motors',
            name='Talon14',
            executable='talon_node',
            parameters=[{
                'use_sim': use_sim,
                'motor_number': 14,
                'diagnostics_port': 56715,
                'invert_motor': True,
                'speed_topic': 'talon_14_speed',
                'info_topic': 'talon_14_info',
                'position_topic': 'talon_14_position',
                'kP': 10.0,
                'kI': 0.000001,
                'kD': 0.000001,
                'kF': 0.0,
                'publishing_delay': 15,
                'kill_key': 50,
                'op_mode': 0,
                'print_data': False,
                'can_interface': 'can0',
                'stop_topic': 'talon_14_stop',
            }],
            output='screen',
            respawn=True,
        ),

        # Talon 16 — Bucket actuator
        Node(
            package='motors',
            name='Talon16',
            executable='talon_node',
            parameters=[{
                'use_sim': use_sim,
                'motor_number': 16,
                'diagnostics_port': 56713,
                'invert_motor': True,
                'speed_topic': 'talon_16_speed',
                'info_topic': 'talon_16_info',
                'position_topic': 'talon_16_position',
                'kP': 10.0,
                'kI': 0.000001,
                'kD': 0.000001,
                'kF': 0.0,
                'publishing_delay': 15,
                'kill_key': 52,
                'op_mode': 0,
                'print_data': False,
                'can_interface': 'can0',
                'stop_topic': 'talon_16_stop',
            }],
            output='screen',
            respawn=True,
        ),

        # =====================================================================
        #  Drive Motors (Falcon 500 / Kraken x60 / Sim)
        # =====================================================================

        # Motor 10 — Right front
        Node(
            package='motors',
            name='DriveMotor10',
            executable='drive_motor_node',
            additional_env={'PHOENIX_DIAGNOSTICS_PORT': '72340'},
            parameters=[{
                'motor_type': drive_motor_type,
                'motor_number': 10,
                'diagnostics_port': 72340,
                'invert_motor': False,
                'speed_topic': 'falcon_10_speed',
                'user_topic': 'falcon_10_user_speed',
                'reset_topic': '1',
                'info_topic': 'talon_10_info',
                'kP': 10.0,
                'kI': 0.000001,
                'kD': 0.000001,
                'kF': 0.0,
                'supply_current_limit': 70.0,
                'publishing_delay': 15,
                'kill_key': 54,
                'op_mode': 0,
                'print_data': False,
                'can_interface': 'can0',
                'stop_topic': 'falcon_10_stop',
            }],
            output='screen',
            respawn=True,
        ),

        # Motor 11 — Left front
        Node(
            package='motors',
            name='DriveMotor11',
            executable='drive_motor_node',
            additional_env={'PHOENIX_DIAGNOSTICS_PORT': '72341'},
            parameters=[{
                'motor_type': drive_motor_type,
                'motor_number': 11,
                'diagnostics_port': 72341,
                'invert_motor': True,
                'speed_topic': 'falcon_11_speed',
                'user_topic': 'falcon_11_user_speed',
                'reset_topic': '2',
                'info_topic': 'talon_11_info',
                'kP': 10.0,
                'kI': 0.000001,
                'kD': 0.000001,
                'kF': 0.0,
                'supply_current_limit': 70.0,
                'publishing_delay': 15,
                'kill_key': 55,
                'op_mode': 0,
                'print_data': False,
                'can_interface': 'can0',
                'stop_topic': 'falcon_11_stop',
            }],
            output='screen',
            respawn=True,
        ),

        # Motor 12 — Right rear
        Node(
            package='motors',
            name='DriveMotor12',
            executable='drive_motor_node',
            additional_env={'PHOENIX_DIAGNOSTICS_PORT': '72342'},
            parameters=[{
                'motor_type': drive_motor_type,
                'motor_number': 12,
                'diagnostics_port': 72342,
                'invert_motor': False,
                'speed_topic': 'falcon_12_speed',
                'user_topic': 'falcon_12_user_speed',
                'reset_topic': '3',
                'info_topic': 'talon_12_info',
                'kP': 10.0,
                'kI': 0.000001,
                'kD': 0.000001,
                'kF': 0.0,
                'supply_current_limit': 70.0,
                'publishing_delay': 15,
                'kill_key': 56,
                'op_mode': 0,
                'print_data': False,
                'can_interface': 'can0',
                'stop_topic': 'falcon_12_stop',
            }],
            output='screen',
            respawn=True,
        ),

        # Motor 13 — Left rear
        Node(
            package='motors',
            name='DriveMotor13',
            executable='drive_motor_node',
            additional_env={'PHOENIX_DIAGNOSTICS_PORT': '72343'},
            parameters=[{
                'motor_type': drive_motor_type,
                'motor_number': 13,
                'diagnostics_port': 72343,
                'invert_motor': True,
                'speed_topic': 'falcon_13_speed',
                'user_topic': 'falcon_13_user_speed',
                'reset_topic': '4',
                'info_topic': 'talon_13_info',
                'kP': 10.0,
                'kI': 0.000001,
                'kD': 0.000001,
                'kF': 0.0,
                'supply_current_limit': 70.0,
                'publishing_delay': 15,
                'kill_key': 57,
                'op_mode': 0,
                'print_data': False,
                'can_interface': 'can0',
                'stop_topic': 'falcon_13_stop',
            }],
            output='screen',
            respawn=True,
        ),
    ])