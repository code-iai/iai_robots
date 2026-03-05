from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    urdf_name = LaunchConfiguration("urdf_name")
    param_name = LaunchConfiguration("param_name")
    transmission_hw_interface = LaunchConfiguration("transmission_hw_interface")

    # Default paths
    urdf_file = os.path.join(
        get_package_share_directory("iai_dualarm_description"),
        "robots",
        "dualarm_ur5s_one_gripper_table.urdf.xacro",
    )
    kinematics_file = os.path.join(
        get_package_share_directory("iai_ur_description"), "config", "ur5_default.yaml"
    )

    robot_description = Command(
        [
            "xacro ",
            urdf_file,
            " transmission_hw_interface:=",
            "hardware_interface/PositionJointInterface",
            " kinematics_config_left:=",
            kinematics_file,
            " kinematics_config_right:=",
            kinematics_file,
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "urdf_name", default_value="dualarm_ur5s_one_gripper.urdf.xacro"
            ),
            DeclareLaunchArgument("param_name", default_value="robot_description"),
            DeclareLaunchArgument(
                "transmission_hw_interface",
                default_value="hardware_interface/PositionJointInterface",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
        ]
    )
