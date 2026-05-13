from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Path to xacro file
    urdf_path = PathJoinSubstitution([
        FindPackageShare('iai_table_robot_description'),
        'robots',
        'ur5_table.urdf.xacro'
    ])

    # Process xacro file to generate robot_description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # Robot state publisher node (publishes the robot_description parameter and TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])