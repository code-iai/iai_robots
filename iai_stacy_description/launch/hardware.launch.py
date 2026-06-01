from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Launch args for IPs
    ip = LaunchConfiguration('robot_ip')

    orbbec_share_dir = get_package_share_directory('orbbec_camera')
    launch_dir = os.path.join(orbbec_share_dir, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('robot_ip', default_value='192.168.1.2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory('ur_robot_driver'),
                    'launch',
                    'ur_control.launch.py'
                )
            ]),
            launch_arguments={
                'robot_ip': ip,
                'use_fake_hardware': 'false',
                'ur_type': 'ur5',
                'initial_joint_controller': 'forward_velocity_controller',
                'launch_rviz': 'false',
                'description_launchfile': os.path.join(
                    get_package_share_directory('iai_stacy_description'),
                    'launch',
                    'rsp.launch.py'
                ),
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'femto_bolt.launch.py')
            ),
            launch_arguments={
                'color_width': '1920',
                'color_height': '1080',
                'depth_registration': 'True',
                'enable_colored_point_cloud': 'True',
                'enable_noise_removal_filter': 'True',
                'noise_removal_filter_min_diff': '3'
            }.items()
        ),
    ])