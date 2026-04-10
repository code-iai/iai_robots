from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Launch args for IPs
    left_ip = LaunchConfiguration('left_robot_ip')
    right_ip = LaunchConfiguration('right_robot_ip')

    description_pkg = get_package_share_directory('iai_dualarm_description')

    rviz_config = os.path.join(
        description_pkg,
        'config',
        'urdf.rviz'
    )

    dual_arm_urdf_file = os.path.join(
        description_pkg,
        'robots',
        'dualarm_ur5_two_gripper.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            dual_arm_urdf_file,
        ]),
        value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument('left_robot_ip', default_value='192.168.101.1'),
        DeclareLaunchArgument('right_robot_ip', default_value='192.168.101.171'),

        # LEFT ARM
        GroupAction([
            PushRosNamespace('left_arm'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('ur_robot_driver'),
                        'launch',
                        'ur_control.launch.py'
                    )
                ]),
                launch_arguments={
                    'robot_ip': left_ip,
                    'use_fake_hardware': 'true',
                    'ur_type': 'ur5',
                    'tf_prefix': 'left_',
                    'initial_joint_controller': 'forward_velocity_controller',
                    'launch_rviz': 'false',
                    'reverse_port': '50008',
                    'script_sender_port': '50011',
                    'trajectory_port': '50010',
                    'script_command_port': '50012',
                    # 'controllers_file': os.path.join(
                    #     get_package_share_directory('iai_dualarm_description'),
                    #     'config',
                    #     'ur5_real_left_arm_control.yaml'
                    # ),
                }.items()
            ),
        ]),

        # RIGHT ARM
        GroupAction([
            PushRosNamespace('right_arm'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('ur_robot_driver'),
                        'launch',
                        'ur_control.launch.py'
                    )
                ]),
                launch_arguments={
                    'robot_ip': right_ip,
                    'use_fake_hardware': 'true',
                    'ur_type': 'ur5',
                    'tf_prefix': 'right_',
                    #'initial_joint_controller': 'forward_velocity_controller',
                    'launch_rviz': 'false',
                    'reverse_port': '50000',
                    'script_sender_port': '50002',
                    'trajectory_port': '50001',
                    'script_command_port': '50003',
                    # 'controllers_file': os.path.join(
                    #     get_package_share_directory('iai_dualarm_description'),
                    #     'config',
                    #     'ur5_real_right_arm_control.yaml'
                    # ),
                }.items()
            ),
        ]),
        GroupAction([
            PushRosNamespace('left_gripper'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('griplink'),
                        'launch',
                        'launch.py'
                    )
                ]),
                launch_arguments={
                    'ip': '192.168.1.40',
                    'port':'10001',
                    #'tf_prefix': 'left_',
                    #'controllers_file': os.path.join(get_package_share_directory('wpg_300_120_description'),'config','gripper_controllers_left.yaml'),
                }.items(),
            )
        ]),
        GroupAction([
            PushRosNamespace('right_gripper'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('griplink'),
                        'launch',
                        'launch.py'
                    )
                ]),
                launch_arguments={
                    'ip': '192.168.1.40',
                    'port':'10001',
                    #'tf_prefix': 'right_',
                    # 'controllers_file': os.path.join(get_package_share_directory('wpg_300_120_description'),'config','gripper_controllers_right.yaml'),
                }.items(),
            )
        ]),

        # JOINT STATE PUBLISHER (merged)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{
                'source_list': [
                    '/left_arm/joint_states',
                    '/right_arm/joint_states',
                    '/left_gripper/griplink_node/device_states',
                    '/right_gripper/griplink_node/device_states'
                ],
                'rate': 120.0,
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d',rviz_config],
            output='screen',
        )
    ])