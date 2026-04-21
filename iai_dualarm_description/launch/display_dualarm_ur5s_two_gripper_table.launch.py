from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():

    robot_xacro_file = os.path.join(
        get_package_share_directory("iai_dualarm_description"),
        "robots",
        "dualarm_ur5s_two_gripper_table.urdf.xacro",
    )

    rviz_config_file = os.path.join(
        get_package_share_directory("iai_dualarm_description"), "config", "urdf.rviz"
    )

    return LaunchDescription(
        [
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(["xacro ", robot_xacro_file]), value_type=str
                        )
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config_file],
            ),
        ]
    )
