from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Declare launch arguments
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start GUI (joint_state_publisher_gui and rviz)",
    )

    # Include the upload launch file
    upload_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("iai_dualarm_description"),
                        "launch",
                        "upload_dualarm_ur5s_one_gripper_table.launch.py",
                    ]
                )
            ]
        )
    )

    joint_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
    )

    # Joint state publisher GUI node
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    # Robot state publisher node (already included in upload launch, but keeping if needed separately)

    # RViz node
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("iai_dualarm_description"), "urdf.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("gui")),
    )

    return LaunchDescription(
        [gui_arg, upload_launch, joint_state_publisher_gui_node, rviz_node]
    )
