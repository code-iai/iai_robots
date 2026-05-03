import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)

from launch.conditions import IfCondition, UnlessCondition

from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
)

from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import (
    Node,
    PushRosNamespace,
)

from launch_ros.parameter_descriptions import (
    ParameterFile,
)

from launch_ros.substitutions import (
    FindPackageShare,
)

from ament_index_python.packages import (
    get_package_share_directory,
)


def launch_setup():

    left_ur_type = LaunchConfiguration("left_ur_type")
    right_ur_type = LaunchConfiguration("right_ur_type")

    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")

    controllers_file = LaunchConfiguration("controllers_file")

    controller_spawner_timeout = LaunchConfiguration(
        "controller_spawner_timeout"
    )

    description_launchfile = LaunchConfiguration(
        "description_launchfile"
    )

    launch_rviz = LaunchConfiguration("launch_rviz")

    rviz_config_file = LaunchConfiguration(
        "rviz_config_file"
    )

    headless_mode = LaunchConfiguration("headless_mode")

    left_use_mock_hardware = LaunchConfiguration(
        "left_use_mock_hardware"
    )

    left_mock_sensor_commands = LaunchConfiguration(
        "left_mock_sensor_commands"
    )

    left_initial_joint_controller = LaunchConfiguration(
        "left_initial_joint_controller"
    )

    left_activate_joint_controller = LaunchConfiguration(
        "left_activate_joint_controller"
    )

    left_launch_dashboard_client = LaunchConfiguration(
        "left_launch_dashboard_client"
    )

    right_use_mock_hardware = LaunchConfiguration(
        "right_use_mock_hardware"
    )

    right_mock_sensor_commands = LaunchConfiguration(
        "right_mock_sensor_commands"
    )

    right_initial_joint_controller = LaunchConfiguration(
        "right_initial_joint_controller"
    )

    right_activate_joint_controller = LaunchConfiguration(
        "right_activate_joint_controller"
    )

    right_launch_dashboard_client = LaunchConfiguration(
        "right_launch_dashboard_client"
    )

    left_reverse_port = LaunchConfiguration(
        "left_reverse_port"
    )

    left_script_sender_port = LaunchConfiguration(
        "left_script_sender_port"
    )

    left_trajectory_port = LaunchConfiguration(
        "left_trajectory_port"
    )

    left_script_command_port = LaunchConfiguration(
        "left_script_command_port"
    )

    right_reverse_port = LaunchConfiguration(
        "right_reverse_port"
    )

    right_script_sender_port = LaunchConfiguration(
        "right_script_sender_port"
    )

    right_trajectory_port = LaunchConfiguration(
        "right_trajectory_port"
    )

    right_script_command_port = LaunchConfiguration(
        "right_script_command_port"
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(
                controllers_file,
                allow_substs=True,
            ),
        ],
        output="screen",
    )

    left_dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        namespace="left",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "robot_ip": left_robot_ip,
            }
        ],
        condition=IfCondition(
            left_launch_dashboard_client
        ),
    )

    right_dashboard_client_node = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        namespace="right",
        name="dashboard_client",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "robot_ip": right_robot_ip,
            }
        ],
        condition=IfCondition(
            right_launch_dashboard_client
        ),
    )

    left_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        namespace="left",
        output="screen",
        parameters=[
            {
                "robot_ip": left_robot_ip,
            }
        ],
        condition=UnlessCondition(
            left_use_mock_hardware
        ),
    )

    right_urscript_interface = Node(
        package="ur_robot_driver",
        executable="urscript_interface",
        namespace="right",
        output="screen",
        parameters=[
            {
                "robot_ip": right_robot_ip,
            }
        ],
        condition=UnlessCondition(
            right_use_mock_hardware
        ),
    )

    left_controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        namespace="left",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(
            left_use_mock_hardware
        ),
        parameters=[
            {
                "headless_mode": headless_mode,
            },
            {
                "joint_controller_active":
                left_activate_joint_controller,
            },
            {
                "consistent_controllers": [
                    "joint_state_broadcaster",

                    "left_io_and_status_controller",
                    "right_io_and_status_controller",

                    "left_force_torque_sensor_broadcaster",
                    "right_force_torque_sensor_broadcaster",

                    "left_speed_scaling_state_broadcaster",
                    "right_speed_scaling_state_broadcaster",
                ]
            },
        ],
    )

    right_controller_stopper_node = Node(
        package="ur_robot_driver",
        executable="controller_stopper_node",
        namespace="right",
        name="controller_stopper",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(
            right_use_mock_hardware
        ),
        parameters=[
            {
                "headless_mode": headless_mode,
            },
            {
                "joint_controller_active":
                right_activate_joint_controller,
            },
            {
                "consistent_controllers": [
                    "joint_state_broadcaster",

                    "left_io_and_status_controller",
                    "right_io_and_status_controller",

                    "left_force_torque_sensor_broadcaster",
                    "right_force_torque_sensor_broadcaster",

                    "left_speed_scaling_state_broadcaster",
                    "right_speed_scaling_state_broadcaster",
                ]
            },
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=[
            "-d",
            rviz_config_file,
        ],
        condition=IfCondition(launch_rviz),
    )

    def controller_spawner(
        controller_name,
        active=True,
    ):

        inactive_flags = []

        if not active:
            inactive_flags.append("--inactive")

        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                controller_name,

                "--controller-manager",
                "/controller_manager",

                "--controller-manager-timeout",
                controller_spawner_timeout,
            ] + inactive_flags,
        )

    controllers_active = [

        "joint_state_broadcaster",

        "left_io_and_status_controller",
        "right_io_and_status_controller",

        "left_speed_scaling_state_broadcaster",
        "right_speed_scaling_state_broadcaster",

        "left_force_torque_sensor_broadcaster",
        "right_force_torque_sensor_broadcaster",
    ]

    controllers_inactive = [

        "left_forward_position_controller",
        "right_forward_position_controller",
    ]

    controller_spawners = (

        [controller_spawner(name)
         for name in controllers_active]

        +

        [controller_spawner(name, active=False)
         for name in controllers_inactive]
    )

    left_initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            left_initial_joint_controller,

            "-c",
            "/controller_manager",

            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
        condition=IfCondition(
            left_activate_joint_controller
        ),
    )

    left_initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            left_initial_joint_controller,

            "-c",
            "/controller_manager",

            "--controller-manager-timeout",
            controller_spawner_timeout,

            "--inactive",
        ],
        condition=UnlessCondition(
            left_activate_joint_controller
        ),
    )

    right_initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            right_initial_joint_controller,

            "-c",
            "/controller_manager",

            "--controller-manager-timeout",
            controller_spawner_timeout,
        ],
        condition=IfCondition(
            right_activate_joint_controller
        ),
    )

    right_initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            right_initial_joint_controller,

            "-c",
            "/controller_manager",

            "--controller-manager-timeout",
            controller_spawner_timeout,

            "--inactive",
        ],
        condition=UnlessCondition(
            right_activate_joint_controller
        ),
    )

    rsp = IncludeLaunchDescription(

        AnyLaunchDescriptionSource(
            description_launchfile
        ),

        launch_arguments={

            "left_robot_ip": left_robot_ip,
            "right_robot_ip": right_robot_ip,

            "left_ur_type": left_ur_type,
            "right_ur_type": right_ur_type,

            "left_reverse_port": left_reverse_port,
            "left_script_sender_port":
            left_script_sender_port,

            "left_trajectory_port":
            left_trajectory_port,

            "left_script_command_port":
            left_script_command_port,

            "right_reverse_port":
            right_reverse_port,

            "right_script_sender_port":
            right_script_sender_port,

            "right_trajectory_port":
            right_trajectory_port,

            "right_script_command_port":
            right_script_command_port,

        }.items(),
    )

    left_gripper = GroupAction([

        PushRosNamespace("left_gripper"),

        IncludeLaunchDescription(

            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        "griplink"
                    ),
                    "launch",
                    "launch.py",
                )
            ),

            launch_arguments={

                "ip": "192.168.1.40",
                "port": "10001",

            }.items(),
        )
    ])

    right_gripper = GroupAction([

        PushRosNamespace("right_gripper"),

        IncludeLaunchDescription(

            AnyLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        "griplink"
                    ),
                    "launch",
                    "launch.py",
                )
            ),

            launch_arguments={

                "ip": "192.168.1.41",
                "port": "10001",

            }.items(),
        )
    ])

    nodes_to_start = [

        control_node,

        left_dashboard_client_node,
        right_dashboard_client_node,

        left_controller_stopper_node,
        right_controller_stopper_node,

        left_urscript_interface,
        right_urscript_interface,

        rsp,

        left_gripper,
        right_gripper,

        rviz_node,

        left_initial_joint_controller_spawner_stopped,
        right_initial_joint_controller_spawner_stopped,

        left_initial_joint_controller_spawner_started,
        right_initial_joint_controller_spawner_started,

    ] + controller_spawners

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "left_ur_type",
            default_value="ur5",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "right_ur_type",
            default_value="ur5",
            choices=[
                "ur3",
                "ur3e",
                "ur5",
                "ur5e",
                "ur10",
                "ur10e",
                "ur16e",
                "ur20",
                "ur30",
            ],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "left_robot_ip",
            default_value="192.168.102.44",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "right_robot_ip",
            default_value="192.168.102.43",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution([
                FindPackageShare(
                    "iai_dualarm_description"
                ),
                "config",
                "combined_controllers.yaml",
            ]),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "description_launchfile",
            default_value=PathJoinSubstitution([
                FindPackageShare(
                    "iai_dualarm_description"
                ),
                "launch",
                "dualarm_rsp.launch.py",
            ]),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "left_use_mock_hardware",
            default_value="false",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "right_use_mock_hardware",
            default_value="false",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "left_mock_sensor_commands",
            default_value="false",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "right_mock_sensor_commands",
            default_value="false",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_spawner_timeout",
            default_value="10",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "left_initial_joint_controller",
            default_value=
            "left_scaled_joint_trajectory_controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "right_initial_joint_controller",
            default_value=
            "right_scaled_joint_trajectory_controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "left_activate_joint_controller",
            default_value="true",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "right_activate_joint_controller",
            default_value="true",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare(
                    "iai_dualarm_description"
                ),
                "config",
                "urdf.rviz",
            ]),
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "left_launch_dashboard_client",
            default_value="true",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "right_launch_dashboard_client",
            default_value="true",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "left_reverse_port",
            default_value="50012",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "left_script_sender_port",
            default_value="50011",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "left_trajectory_port",
            default_value="50013",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "left_script_command_port",
            default_value="50014",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "right_reverse_port",
            default_value="50001",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "right_script_sender_port",
            default_value="50002",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "right_trajectory_port",
            default_value="50003",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "right_script_command_port",
            default_value="50004",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "update_rate_config_file",
            default_value=[
                PathJoinSubstitution([
                    FindPackageShare(
                        "iai_dualarm_description"
                    ),
                    "config",
                ]),
                "update_rate.yaml",
            ],
        )
    )

    return LaunchDescription(
        declared_arguments + launch_setup()
    )