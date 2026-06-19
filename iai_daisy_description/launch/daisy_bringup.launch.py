from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile

def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    left_ur_type = LaunchConfiguration("left_ur_type")
    right_ur_type = LaunchConfiguration("right_ur_type")
    left_robot_ip = LaunchConfiguration("left_robot_ip")
    right_robot_ip = LaunchConfiguration("right_robot_ip")

    controllers_file = LaunchConfiguration("controllers_file")
    controller_spawner_timeout = LaunchConfiguration("controller_spawner_timeout")
    description_launchfile = LaunchConfiguration("description_launchfile")
    headless_mode = LaunchConfiguration("headless_mode")
    
    left_use_mock_hardware = LaunchConfiguration("left_use_mock_hardware")
    right_use_mock_hardware = LaunchConfiguration("right_use_mock_hardware")
    left_activate_joint_controller = LaunchConfiguration("left_activate_joint_controller")
    right_activate_joint_controller = LaunchConfiguration("right_activate_joint_controller")
    left_initial_joint_controller = LaunchConfiguration("left_initial_joint_controller")
    right_initial_joint_controller = LaunchConfiguration("right_initial_joint_controller")

    # --- Generate URDF via Xacro ---
    left_kinematics = PathJoinSubstitution([FindPackageShare("iai_daisy_description"), "config", "ur5_left_arm_calibration.yaml"])
    right_kinematics = PathJoinSubstitution([FindPackageShare("iai_daisy_description"), "config", "ur5_right_arm_calibration.yaml"])

    robot_description_command = Command([
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

    robot_description_param = ParameterValue(robot_description_command, value_type=str)

    # Main Control Node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description_param}, 
            LaunchConfiguration("update_rate_config_file"),
            ParameterFile(controllers_file, allow_substs=True),
        ],
        output="screen",
        remappings=[("joint_states", "arms/joint_states")],
    )

    left_dashboard = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        namespace="left",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": left_robot_ip}],
        condition=UnlessCondition(left_use_mock_hardware),
    )

    right_dashboard = Node(
        package="ur_robot_driver",
        executable="dashboard_client",
        name="dashboard_client",
        namespace="right",
        output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": right_robot_ip}],
        condition=UnlessCondition(right_use_mock_hardware),
    )

    left_gripper = Node(
        package='griplink',
        namespace='left_gripper',  
        executable='griplink_node',
        name='griplink_node',
        parameters=[
            {"ip": LaunchConfiguration("left_gripper_ip")},
            {"port": LaunchConfiguration("griplink_network_port")},
            {"joint_name": "left_gripper_finger_joint"}
        ],
    )

    right_gripper= Node(
        package='griplink',
        namespace='right_gripper', 
        executable='griplink_node',
        name='griplink_node',
        parameters=[
            {"ip": LaunchConfiguration("right_gripper_ip")},
            {"port": LaunchConfiguration("griplink_network_port")},
            {"joint_name": "right_gripper_finger_joint"}
        ],
    )

    # Controller Spawner Helper
    def controller_spawner(controllers, active=True):
        inactive_flags = ["--inactive"] if not active else []
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", controller_spawner_timeout,
            ] + inactive_flags + controllers,
        )

    # Broadcasters and Status Controllers
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

    left_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[left_initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(left_activate_joint_controller),
    )
    
    left_spawner_inactive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[left_initial_joint_controller, "-c", "/controller_manager", "--inactive"],
        condition=UnlessCondition(left_activate_joint_controller),
    )

    right_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[right_initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(right_activate_joint_controller),
    )

    right_spawner_inactive = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[right_initial_joint_controller, "-c", "/controller_manager", "--inactive"],
        condition=UnlessCondition(right_activate_joint_controller),
    )

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(description_launchfile),
        launch_arguments={
            "robot_description": robot_description_command, # <--- Update this
            "left_robot_ip": left_robot_ip,
            "right_robot_ip": right_robot_ip,
            "left_ur_type": left_ur_type,
            "right_ur_type": right_ur_type,
            "left_use_mock_hardware": left_use_mock_hardware,
            "right_use_mock_hardware": right_use_mock_hardware,
            "headless_mode": headless_mode,
        }.items(),
    )
    
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': [
                '/arms/joint_states',
                '/left_gripper/joint_states',
                '/right_gripper/joint_states'
            ],
            'rate': 100.0,
        }]
    )
    nodes_to_start = [
        control_node,
        left_dashboard,
        right_dashboard,
        rsp,
        joint_state_publisher_node,
        left_gripper,
        right_gripper,
        controller_spawner(controllers_active),
        controller_spawner(controllers_inactive, active=False),
        left_spawner,
        left_spawner_inactive,
        right_spawner,
        right_spawner_inactive,
    ]

    return nodes_to_start

def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument("left_ur_type", default_value="ur5"),
        DeclareLaunchArgument("right_ur_type", default_value="ur5"),
        DeclareLaunchArgument("left_robot_ip", default_value="192.168.102.44"),
        DeclareLaunchArgument("right_robot_ip", default_value="192.168.102.43"),
        DeclareLaunchArgument("left_gripper_ip", default_value="192.168.102.42"),
        DeclareLaunchArgument("right_gripper_ip", default_value="192.168.102.41"),
        DeclareLaunchArgument("griplink_network_port", default_value="10001"), 
        DeclareLaunchArgument("controllers_file", default_value=PathJoinSubstitution([FindPackageShare("iai_daisy_description"), "config", "combined_controllers.yaml"])),
        DeclareLaunchArgument("description_launchfile", default_value=PathJoinSubstitution([FindPackageShare("iai_daisy_description"), "launch", "daisy_rsp.launch.py"])),
        DeclareLaunchArgument("left_use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("right_use_mock_hardware", default_value="false"),
        DeclareLaunchArgument("headless_mode", default_value="false"),
        DeclareLaunchArgument("controller_spawner_timeout", default_value="30"),
        DeclareLaunchArgument("left_initial_joint_controller", default_value="left_forward_velocity_controller"),
        DeclareLaunchArgument("right_initial_joint_controller", default_value="right_forward_velocity_controller"),
        DeclareLaunchArgument("left_activate_joint_controller", default_value="true"),
        DeclareLaunchArgument("right_activate_joint_controller", default_value="true"),
        DeclareLaunchArgument("update_rate_config_file", default_value=[PathJoinSubstitution([FindPackageShare("ur_robot_driver"), "config"]), "/", LaunchConfiguration("left_ur_type"), "_update_rate.yaml"]),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
