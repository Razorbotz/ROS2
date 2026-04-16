from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Resolve the launch configurations into standard Python strings
    robot_name = LaunchConfiguration('robot').perform(context).lower()
    is_sim_str = LaunchConfiguration('simulation').perform(context).lower()
    
    is_sim = (is_sim_str == 'true')

    # Assign CAN IDs based on physical wiring order
    # Motor CAN IDs in physical wiring order along the daisy chain.
    # Index 0 = closest to CAN0 interface, last = closest to CAN1.
    # These are decimal values for the hex CAN IDs:
    #   0xA=10, 0xB=11, 0xD=13, 0xC=12, 0x10=16, 0xE=14
    if robot_name == 'sisyphus':
        num_motors = 4
        wiring_order = [10, 11, 13, 12]
    else: 
        # Defaults to Talos or Sierra configurations
        num_motors = 6
        wiring_order = [10, 11, 13, 12, 16, 14]

    # Construct the node with native Python integers and lists
    status_node = Node(
        package='status_monitor',
        name='status_monitor',
        executable='status_monitor_node',
        parameters=[
            {"print_data": False},
            {"simulation": is_sim},
            {"num_motors": num_motors},
            {"motor_wiring_order": wiring_order},
        ],
        respawn=True
    )

    return [status_node]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot', 
            default_value='talos', 
            description='Name of the robot (sisyphus, talos, sierra)'
        ),
        DeclareLaunchArgument(
            'simulation', 
            default_value='false', 
            description='Run in simulation mode'
        ),
        OpaqueFunction(function=launch_setup)
    ])