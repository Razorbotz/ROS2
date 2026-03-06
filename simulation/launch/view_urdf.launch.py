from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os


def generate_launch_description():

    urdf_path = os.path.join(
        os.path.dirname(__file__),
        "..",
        "src",
        "sim",
        "urdf",
        "my_robot_tf.urdf"
    )

    robot_description = ParameterValue(
        Command(["cat ", urdf_path]),
        value_type=str
    )

    return LaunchDescription([

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description
            }]
        ),

        # Joint State Publisher GUI (for moving joints)
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            output="screen"
        ),

        # RViz
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen"
        )
    ])