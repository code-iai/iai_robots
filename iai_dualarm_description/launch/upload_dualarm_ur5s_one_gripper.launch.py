from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    urdf_name_arg = DeclareLaunchArgument(
        'urdf_name',
        default_value='dualarm_ur5s_one_gripper.urdf.xacro',
        description='Name of the URDF/xacro file'
    )

    param_name_arg = DeclareLaunchArgument(
        'param_name',
        default_value='robot_description',
        description='Name of the robot description parameter'
    )

    transmission_hw_interface_arg = DeclareLaunchArgument(
        'transmission_hw_interface',
        default_value='hardware_interface/PositionJointInterface',
        description='Hardware interface for transmissions'
    )

    kinematics_config_left_arg = DeclareLaunchArgument(
        'kinematics_config_left',
        default_value=PathJoinSubstitution([
            FindPackageShare('iai_ur_description'),
            'config',
            'ur5_default.yaml'
        ]),
        description='Kinematics config file for left arm'
    )

    kinematics_config_right_arg = DeclareLaunchArgument(
        'kinematics_config_right',
        default_value=PathJoinSubstitution([
            FindPackageShare('iai_ur_description'),
            'config',
            'ur5_default.yaml'
        ]),
        description='Kinematics config file for right arm'
    )

    # Build the xacro command with arguments
    urdf_path = PathJoinSubstitution([
        FindPackageShare('iai_dualarm_description'),
        'robots',
        LaunchConfiguration('urdf_name')
    ])

    robot_description = ParameterValue(
        Command([
            'xacro ',
            urdf_path,
            ' transmission_hw_interface:=',
            LaunchConfiguration('transmission_hw_interface'),
            ' kinematics_config_left:=',
            LaunchConfiguration('kinematics_config_left'),
            ' kinematics_config_right:=',
            LaunchConfiguration('kinematics_config_right')
        ]),
        value_type=str
    )

    # Robot state publisher node to publish the robot description
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{LaunchConfiguration('param_name'): robot_description}]
    )

    return LaunchDescription([
        urdf_name_arg,
        param_name_arg,
        transmission_hw_interface_arg,
        kinematics_config_left_arg,
        kinematics_config_right_arg,
        robot_state_publisher_node
    ])