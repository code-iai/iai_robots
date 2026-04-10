import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Launch Configurations

    left_ip = LaunchConfiguration('left_robot_ip')
    right_ip = LaunchConfiguration('right_robot_ip')
    left_gripper_ip = LaunchConfiguration('left_gripper_ip')
#    right_gripper_ip = LaunchConfiguration('right_gripper_ip')

    # Paths

    description_pkg = get_package_share_directory('iai_dualarm_description')

    urdf_file = os.path.join(
        description_pkg,
        'robots',
        'dualarm_ur5s_one_gripper.urdf.xacro'
    )

    kinematics_file_left = os.path.join(
        description_pkg,
        'config',
        'ur5_left_arm_calibration.yaml'
    )

    kinematics_file_right = os.path.join(
        description_pkg,
        'config',
        'ur5_right_arm_calibration.yaml'
    )

    ur_bringup_launch = os.path.join(
        get_package_share_directory('ur_robot_driver'),
        'launch',
        'ur_control.launch.py'
    )

    rviz_config = os.path.join(
        description_pkg,
        'config',
        'urdf.rviz'
    )

    # Robot Description

    robot_description = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            urdf_file,
            ' kinematics_config_left:=', kinematics_file_left,
            ' kinematics_config_right:=', kinematics_file_right
        ]),
        value_type=str
    )

    return LaunchDescription([

        # Launch Arguments

        DeclareLaunchArgument('left_robot_ip', default_value='192.168.101.1'),
        DeclareLaunchArgument('right_robot_ip', default_value='192.168.101.171'),
        DeclareLaunchArgument('left_gripper_ip', default_value='192.168.1.40'),
#        DeclareLaunchArgument('right_gripper_ip', default_value='#TODO'),

        # Static Transforms

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'world']
        ),

        # LEFT ARM

        GroupAction([
            PushRosNamespace('left_arm'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_bringup_launch),
                launch_arguments={
                    'robot_ip': left_ip,
                    'tf_prefix': 'left_',
                    'kinematics_config': kinematics_file_left,
                }.items()
            )
        ]),

        # RIGHT ARM

        GroupAction([
            PushRosNamespace('right_arm'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_bringup_launch),
                launch_arguments={
                    'robot_ip': right_ip,
                    'tf_prefix': 'right_',
                    'kinematics_config': kinematics_file_right,
                }.items()
            )
        ]),

        # LEFT GRIPPER

        GroupAction([
            PushRosNamespace('left_gripper'),

            Node(
                package='griplink',
                executable='griplink_node',
                name='griplink_node',
                namespace='griplink_node',
                output='screen',
                parameters=[
                    {"ip": left_gripper_ip},
                    {"port": 10001}
                ]
            )
        ]),

        # RIGHT GRIPPER

        # GroupAction([
        #     PushRosNamespace('right_gripper'),
        #
        #     Node(
        #         package='griplink',
        #         executable='griplink_node',
        #         name='griplink_node',
        #         namespace='griplink_node',
        #         output='screen',
        #         parameters=[
        #             {"ip": right_gripper_ip},
        #             {"port": 10001}
        #         ]
        #     )
        # ]),
        # Joint State Merger

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_merger',
            parameters=[{
                'source_list': [
                    '/left_arm/joint_states',
                    '/right_arm/joint_states',
                    '/left_gripper/griplink_node/device_states'
                ],
                'rate': 120.0,
                'use_gui': False
            }]
        ),

        # Robot State Publisher

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description
            }],
            output='screen'
        ),

        # RViz

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[rviz_config],
            output='screen'
        )

    ])
