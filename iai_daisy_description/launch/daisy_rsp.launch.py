from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    # Configurations
    left_ur_type = LaunchConfiguration("left_ur_type")
    right_ur_type = LaunchConfiguration("right_ur_type")
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")
    left_use_mock_hardware = LaunchConfiguration("left_use_mock_hardware")
    right_use_mock_hardware = LaunchConfiguration("right_use_mock_hardware")
    headless_mode = LaunchConfiguration("headless_mode")

    # Path to your custom kinematics/calibrations
    left_kinematics = PathJoinSubstitution([FindPackageShare("iai_daisy_description"), "config", "ur5_left_arm_calibration.yaml"])
    right_kinematics = PathJoinSubstitution([FindPackageShare("iai_daisy_description"), "config", "ur5_right_arm_calibration.yaml"])
    rviz_config_file = PathJoinSubstitution([FindPackageShare("iai_daisy_description"), "config",'urdf.rviz'])

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ",
        PathJoinSubstitution([FindPackageShare("iai_daisy_description"), "robots", "daisy.urdf.xacro"]),
        " ",
        "left_robot_ip:=", left_robot_ip, " ",
        "right_robot_ip:=", right_robot_ip, " ",
        "left_ur_type:=", left_ur_type, " ",
        "right_ur_type:=", right_ur_type, " ",
        "left_use_mock_hardware:=", left_use_mock_hardware, " ",
        "right_use_mock_hardware:=", right_use_mock_hardware, " ",
        "left_kinematics_parameters_file:=", left_kinematics, " ",
        "right_kinematics_parameters_file:=", right_kinematics, " ",
        "headless_mode:=", headless_mode,
    ])

    robot_description = {"robot_description": robot_description_content}

    declared_arguments = [
        DeclareLaunchArgument("left_ur_type", default_value="ur5"),
        DeclareLaunchArgument("right_ur_type", default_value="ur5"),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.102.44"),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.102.43"),
        DeclareLaunchArgument("left_use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("right_use_mock_hardware", default_value="true"),
        DeclareLaunchArgument("headless_mode", default_value="false"),
    ]

    return LaunchDescription(declared_arguments + [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="both",
            arguments=["-d",rviz_config_file],
        ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            name="joint_state_publisher",
            output="both",
            parameters=[robot_description],
        ),
    ])