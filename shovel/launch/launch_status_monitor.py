from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
       Node(
            package='status_monitor',
            name='status_monitor',
            executable='status_monitor_node',
            parameters=[
                {"print_data": False},
                {"simulation": False},
                {"num_motors": 6},
                # Motor CAN IDs in physical wiring order along the daisy chain.
                # Index 0 = closest to CAN0 interface, last = closest to CAN1.
                # These are decimal values for the hex CAN IDs:
                #   0xA=10, 0xB=11, 0xD=13, 0xC=12, 0x10=16, 0xE=14
                {"motor_wiring_order": [10, 11, 13, 12, 16, 14]},
            ],
            respawn=True
        )
    ]
)