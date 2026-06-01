from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_type = LaunchConfiguration('ur_type')
    robot_ip = LaunchConfiguration('robot_ip')
    kinematics_parameters_file = LaunchConfiguration('kinematics_parameters_file')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    mock_sensor_commands = LaunchConfiguration('mock_sensor_commands')
    headless_mode = LaunchConfiguration('headless_mode')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([FindPackageShare('iai_stacy_description'), 'urdf', 'stacy_controlled.urdf.xacro']),
        ' robot_ip:=', robot_ip,
        ' ur_type:=', ur_type,
        ' kinematics_parameters_file:=', kinematics_parameters_file,
        ' use_mock_hardware:=', use_mock_hardware,
        ' mock_sensor_commands:=', mock_sensor_commands,
        ' headless_mode:=', headless_mode,
    ])

    return LaunchDescription([
        DeclareLaunchArgument('ur_type', default_value='ur5',
                              description='UR robot type'),
        DeclareLaunchArgument('robot_ip',
                              description='IP address of the UR robot'),
        DeclareLaunchArgument(
            'kinematics_parameters_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('ur_description'), 'config', ur_type, 'default_kinematics.yaml'
            ]),
            description='Kinematics calibration file (use robot-specific file for real hardware)'),
        DeclareLaunchArgument('use_mock_hardware', default_value='false',
                              description='Use mock hardware instead of real robot'),
        DeclareLaunchArgument('mock_sensor_commands', default_value='false'),
        DeclareLaunchArgument('headless_mode', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='both',
            parameters=[{'robot_description': robot_description_content}]),
    ])
