import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Define the input argument (The Nickname)
    target_robot_arg = DeclareLaunchArgument(
        'target',
        default_value='sierra',
        description='Nickname of the target Jetson (Sierra, Sisyphus, Talos)'
    )

    # 2. MAC Address Lookup Logic
    # Replace these with your actual Jetson MAC addresses
    mac_address = PythonExpression([
        "'00:A5:54:7A:B9:2F' if '", LaunchConfiguration('target'), "' == 'Sierra' else ",
        "'F8:3D:C6:57:3C:FA' if '", LaunchConfiguration('target'), "' == 'Sisyphus' else ",
        "'F8:3D:C6:57:3C:FA' if '", LaunchConfiguration('target'), "' == 'Talos' else ",
        "''" # Default empty
    ])

    # 3. Path to your original Bluetooth launch file
    # Using your os.getcwd() style to match your current setup
    bt_launch_path = os.path.join(os.getcwd(), 'src', 'bluetooth', 'launch', 'bt_link.launch.py')

    return LaunchDescription([
        target_robot_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bt_launch_path),
            launch_arguments={
                'peer_mac': mac_address,
                'channel': '1',
                'enable_stdio': 'false'
            }.items()
        )
    ])