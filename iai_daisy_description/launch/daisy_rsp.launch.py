from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    rviz_config_file = PathJoinSubstitution([FindPackageShare("iai_daisy_description"), "config", "urdf.rviz"])

    # Accept the parameter passed from bringup
    robot_description_content = LaunchConfiguration("robot_description")
    robot_description = {"robot_description": robot_description_content}

    declared_arguments = [
        DeclareLaunchArgument("robot_description", description="URDF content passed from the parent bringup file"),
        DeclareLaunchArgument("left_ur_type", default_value="ur5"),
        DeclareLaunchArgument("right_ur_type", default_value="ur5"),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.102.44"),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.102.43"),
        DeclareLaunchArgument("left_use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("right_use_mock_hardware", default_value="false"),
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
            arguments=["-d", rviz_config_file],
        )
    ])