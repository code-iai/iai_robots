from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    declared_arguments = []
    
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument("headless_mode", default_value="false", description="Enable headless mode for robot control")
    )

    # ---------------------------------------------------------
    # Helper to generate duplicate arguments for both arms
    # ---------------------------------------------------------
    def add_arm_args(prefix, default_type, default_ip, ports_offset=0, default_tf_prefix=""):
        # Base settings
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_ur_type", description=f"Type/series of used UR robot ({prefix}).", choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e", "ur20", "ur30"], default_value=default_type))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_robot_ip", default_value=default_ip, description=f"IP address by which {prefix} can be reached."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_use_mock_hardware", default_value="false", description=f"Start {prefix} with mock hardware mirroring command to its states."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_mock_sensor_commands", default_value="false", description=f"Enable mock command interfaces for {prefix} sensors."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_kinematics_parameters_file", default_value=PathJoinSubstitution([FindPackageShare("iai_daisy_description"), "config", f"{prefix}_calibration.yaml"]), description=f"The calibration configuration of {prefix}."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_tf_prefix", default_value=default_tf_prefix, description=f"tf_prefix for {prefix} arm."))

        # Safety & Limits
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_safety_limits", default_value="true", description="Enables the safety limits controller if true."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_safety_pos_margin", default_value="0.15", description="The margin to lower and upper limits in the safety controller."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_safety_k_position", default_value="20", description="k-position factor in the safety controller."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_joint_limit_params_file", default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config", LaunchConfiguration(f"{prefix}_ur_type"), "joint_limits.yaml"]), description="Config file containing the joint limits of the robot."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_physical_params_file", default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config", LaunchConfiguration(f"{prefix}_ur_type"), "physical_parameters.yaml"]), description="Config file containing physical parameters."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_visual_params_file", default_value=PathJoinSubstitution([FindPackageShare("ur_description"), "config", LaunchConfiguration(f"{prefix}_ur_type"), "visual_parameters.yaml"]), description="Config file containing visual parameters."))

        # Tool Communication
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_use_tool_communication", default_value="false", description="Only available for e series!"))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_tool_parity", default_value="0", description="Parity configuration for serial communication."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_tool_baud_rate", default_value="115200", description="Baud rate configuration for serial communication."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_tool_stop_bits", default_value="1", description="Stop bits configuration for serial communication."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_tool_rx_idle_chars", default_value="1.5", description="RX idle chars configuration."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_tool_tx_idle_chars", default_value="3.5", description="TX idle chars configuration."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_tool_device_name", default_value=f"/tmp/ttyUR_{prefix}", description="File descriptor generated for the tool communication device."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_tool_tcp_port", default_value=str(54321 + ports_offset), description="Remote port for bridging the tool's serial device."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_tool_voltage", default_value="0", description="Tool voltage that will be setup."))

        # Networking & Ports (Offset avoids port collisions between arms)
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_reverse_ip", default_value="0.0.0.0", description="IP used by the robot controller to communicate back to the driver."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_script_command_port", default_value=str(50004 + ports_offset), description="Port to forward URScript commands."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_reverse_port", default_value=str(50001 + ports_offset), description="Port to send cyclic instructions."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_script_sender_port", default_value=str(50002 + ports_offset), description="Port to query the external_control URScript."))
        declared_arguments.append(DeclareLaunchArgument(f"{prefix}_trajectory_port", default_value=str(50003 + ports_offset), description="Port for trajectory control."))

    # Register arguments for Left and Right robots
    add_arm_args("left", default_type="ur5", default_ip="192.168.102.44", ports_offset=0, default_tf_prefix="left_")
    add_arm_args("right", default_type="ur5", default_ip="192.168.102.43", ports_offset=10, default_tf_prefix="right_")

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            ),
        ]
    )
