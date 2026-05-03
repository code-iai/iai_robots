from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch args for IPs
    left_ip = LaunchConfiguration('left_robot_ip')
    right_ip = LaunchConfiguration('right_robot_ip')

    tracy_xacro_file = os.path.join(get_package_share_directory('iai_dualarm_description'), 'robots',
                                     'dualarm_ur5_two_gripper.urdf.xacro')

    left_kinematics = os.path.join(
        get_package_share_directory('iai_dualarm_description'), 'config', 'left_calibration.yaml')

    right_kinematics = os.path.join(
        get_package_share_directory('iai_dualarm_description'), 'config', 'right_calibration.yaml')

    robot_description = Command([
        FindExecutable(name='xacro'), ' ', tracy_xacro_file,
        ' kinematics_config_left:=', left_kinematics,
        ' kinematics_config_right:=', right_kinematics,
    ])



    return LaunchDescription([
        DeclareLaunchArgument('left_robot_ip', default_value='192.168.102.43'),
        DeclareLaunchArgument('right_robot_ip', default_value='192.168.102.44'),

        # LEFT ARM
        GroupAction([
            PushRosNamespace('left_arm'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('iai_dualarm_description'),
                        'launch',
                        'iai_ur_control.launch.py'
                    )
                ]),
                launch_arguments={
                    'robot_ip': left_ip,
                    'use_fake_hardware': 'false',
                    'ur_type': 'ur5',
                    'tf_prefix': 'left_',
                    'initial_joint_controller': 'forward_velocity_controller',
                    'launch_rviz': 'false',
                    'reverse_port': '50012',
                    'script_sender_port': '50011',
                    'trajectory_port': '50013',
                    'script_command_port': '50014',
                    'kinematics_params_file': left_kinematics,
                    'controllers_file': os.path.join(
                        get_package_share_directory('iai_dualarm_description'),
                        'config',
                        'ur5_left_tracy.yaml'
                    ),
                }.items()
            ),
        ]),

        # RIGHT ARM
        GroupAction([
            PushRosNamespace('right_arm'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    os.path.join(
                        get_package_share_directory('iai_dualarm_description'),
                        'launch',
                        'iai_ur_control.launch.py'
                    )
                ]),
                launch_arguments={
                    'robot_ip': right_ip,
                    'use_fake_hardware': 'false',
                    'ur_type': 'ur5',
                    'tf_prefix': 'right_',
                    'initial_joint_controller': 'forward_velocity_controller',
                    'launch_rviz': 'false',
                    'reverse_port': '50001',
                    'script_sender_port': '50002',
                    'trajectory_port': '5003',
                    'script_command_port': '50004',
                    'kinematics_params_file': right_kinematics,
                    'controllers_file': os.path.join(
                        get_package_share_directory('iai_dualarm_description'),
                        'config',
                        'ur5_right_tracy.yaml'
                    ),
                }.items()
            ),
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
                    '/right_arm/joint_states'
                ],
                'rate': 100.0,
            }]
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            #remappings=[('/joint_states', '/asdf')],# remapping to asdf because the RSP should only publish static transforms
            parameters=[{'robot_description': robot_description}]
        )
    ])