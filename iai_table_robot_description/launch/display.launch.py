from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Include the upload launch file
    upload_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('iai_table_robot_description'),
                'launch',
                'upload.launch.py'
            ])
        ])
    )

    # Joint state publisher node with parameters
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_gui': True,
            'zeros': {
                'shoulder_pan_joint': 0.7854,
                'shoulder_lift_joint': -0.78,
                'elbow_joint': 0.78,
                'wrist_1_joint': -1.57,
                'wrist_2_joint': -1.57
            }
        }]
    )

    # RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('iai_table_robot_description'),
        'config',
        'test.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config_file],
        output='screen',
        # Note: 'required=true' in ROS1 means if this node dies, shutdown launch
        # In ROS2, use on_exit=Shutdown() for similar behavior if needed
    )

    return LaunchDescription([
        upload_launch,
        joint_state_publisher_node,
        rviz_node
    ])