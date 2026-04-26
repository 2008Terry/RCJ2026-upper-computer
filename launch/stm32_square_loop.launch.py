from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("rcj_localization"))
    gateway_launch_file = package_share / "launch" / "stm32_serial_gateway.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_gateway",
                default_value="true",
                description=(
                    "Start stm32_serial_gateway_node. Set false if it is already running."
                ),
            ),
            DeclareLaunchArgument(
                "side_cm",
                default_value="20.0",
                description="Square side length in centimeters.",
            ),
            DeclareLaunchArgument(
                "pause_sec",
                default_value="0.2",
                description="Pause after each acknowledged cmd_dis command.",
            ),
            DeclareLaunchArgument(
                "start_delay_sec",
                default_value="1.0",
                description="Delay before sending the first square command.",
            ),
            DeclareLaunchArgument(
                "motion_action_name",
                default_value="/stm32/motion",
                description="STM32 gateway motion action name.",
            ),
            DeclareLaunchArgument(
                "action_wait_timeout_sec",
                default_value="1.0",
                description="Timeout while waiting for the STM32 motion action server.",
            ),
            DeclareLaunchArgument(
                "retry_on_failure",
                default_value="true",
                description="Retry the same side when the gateway motion action reports failure.",
            ),
            DeclareLaunchArgument(
                "max_cycles",
                default_value="0",
                description="Number of square cycles to run; 0 means loop forever.",
            ),
            DeclareLaunchArgument(
                "node_name",
                default_value="stm32_serial_gateway_node",
                description="ROS node name for the STM32 serial gateway.",
            ),
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyUSB0",
                description="Serial port connected to the STM32.",
            ),
            DeclareLaunchArgument(
                "baudrate",
                default_value="115200",
                description="Serial baudrate.",
            ),
            DeclareLaunchArgument(
                "tick_period_ms",
                default_value="10",
                description="Gateway timer period in milliseconds.",
            ),
            DeclareLaunchArgument(
                "resend_period_ms",
                default_value="20",
                description=(
                    "Deprecated compatibility parameter; gateway commands are sent once."
                ),
            ),
            DeclareLaunchArgument(
                "command_timeout_ms",
                default_value="50",
                description="Gateway timeout for one command in milliseconds.",
            ),
            DeclareLaunchArgument(
                "motion_timeout_ms",
                default_value="5000",
                description=(
                    "Gateway timeout for cmd_dis/cmd_turn completion ACK in milliseconds."
                ),
            ),
            DeclareLaunchArgument(
                "max_queue_size",
                default_value="32",
                description="Maximum queued commands behind the active command.",
            ),
            DeclareLaunchArgument(
                "enable_serial_log",
                default_value="true",
                description="Log parsed serial sends and ACK results.",
            ),
            DeclareLaunchArgument(
                "enable_raw_reply_log",
                default_value="false",
                description="Log raw serial chunks and reply lines from STM32.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for the gateway and square loop nodes.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gateway_launch_file)),
                condition=IfCondition(LaunchConfiguration("start_gateway")),
                launch_arguments={
                    "node_name": LaunchConfiguration("node_name"),
                    "port": LaunchConfiguration("port"),
                    "motion_action_name": LaunchConfiguration("motion_action_name"),
                    "baudrate": LaunchConfiguration("baudrate"),
                    "tick_period_ms": LaunchConfiguration("tick_period_ms"),
                    "resend_period_ms": LaunchConfiguration("resend_period_ms"),
                    "command_timeout_ms": LaunchConfiguration("command_timeout_ms"),
                    "motion_timeout_ms": LaunchConfiguration("motion_timeout_ms"),
                    "max_queue_size": LaunchConfiguration("max_queue_size"),
                    "enable_serial_log": LaunchConfiguration("enable_serial_log"),
                    "enable_raw_reply_log": LaunchConfiguration("enable_raw_reply_log"),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
            Node(
                package="rcj_localization",
                executable="stm32_square_loop.py",
                name="stm32_square_loop_node",
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                parameters=[
                    {
                        "motion_action_name": LaunchConfiguration("motion_action_name"),
                        "side_cm": ParameterValue(
                            LaunchConfiguration("side_cm"), value_type=float
                        ),
                        "pause_sec": ParameterValue(
                            LaunchConfiguration("pause_sec"), value_type=float
                        ),
                        "start_delay_sec": ParameterValue(
                            LaunchConfiguration("start_delay_sec"),
                            value_type=float,
                        ),
                        "action_wait_timeout_sec": ParameterValue(
                            LaunchConfiguration("action_wait_timeout_sec"),
                            value_type=float,
                        ),
                        "retry_on_failure": ParameterValue(
                            LaunchConfiguration("retry_on_failure"),
                            value_type=bool,
                        ),
                        "max_cycles": ParameterValue(
                            LaunchConfiguration("max_cycles"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
