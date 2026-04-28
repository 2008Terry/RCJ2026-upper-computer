from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription(
        [
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
                "motion_action_name",
                default_value="/stm32/motion",
                description="STM32 motion action name for cmd_dis/cmd_turn.",
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
                    "Deprecated compatibility parameter; commands are sent once "
                    "and this value is ignored."
                ),
            ),
            DeclareLaunchArgument(
                "command_timeout_ms",
                default_value="30",
                description="Total timeout for one active command in milliseconds.",
            ),
            DeclareLaunchArgument(
                "motion_timeout_ms",
                default_value="6000",
                description=(
                    "Timeout for cmd_dis/cmd_turn completion ACK after the motion command "
                    "has been sent, in milliseconds."
                ),
            ),
            DeclareLaunchArgument(
                "max_queue_size",
                default_value="32",
                description="Maximum number of queued commands waiting behind the active command.",
            ),
            DeclareLaunchArgument(
                "enable_serial_log",
                default_value="true",
                description="Log parsed serial sends and ACK results.",
            ),
            DeclareLaunchArgument(
                "enable_raw_reply_log",
                default_value="true",
                description="Log raw serial chunks and reply lines from STM32.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="ROS log level for the node.",
            ),
            Node(
                package="rcj_localization",
                executable="stm32_serial_gateway_node",
                name=LaunchConfiguration("node_name"),
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                parameters=[
                    {
                        "port": LaunchConfiguration("port"),
                        "motion_action_name": LaunchConfiguration("motion_action_name"),
                        "baudrate": ParameterValue(
                            LaunchConfiguration("baudrate"), value_type=int
                        ),
                        "tick_period_ms": ParameterValue(
                            LaunchConfiguration("tick_period_ms"), value_type=int
                        ),
                        "resend_period_ms": ParameterValue(
                            LaunchConfiguration("resend_period_ms"), value_type=int
                        ),
                        "command_timeout_ms": ParameterValue(
                            LaunchConfiguration("command_timeout_ms"), value_type=int
                        ),
                        "motion_timeout_ms": ParameterValue(
                            LaunchConfiguration("motion_timeout_ms"), value_type=int
                        ),
                        "max_queue_size": ParameterValue(
                            LaunchConfiguration("max_queue_size"), value_type=int
                        ),
                        "enable_serial_log": ParameterValue(
                            LaunchConfiguration("enable_serial_log"), value_type=bool
                        ),
                        "enable_raw_reply_log": ParameterValue(
                            LaunchConfiguration("enable_raw_reply_log"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
