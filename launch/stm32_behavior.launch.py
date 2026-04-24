import ast

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def parse_commands_argument(commands_text):
    commands_text = commands_text.strip()
    if not commands_text:
        raise RuntimeError("commands launch argument must not be empty")

    if commands_text.startswith("["):
        commands = ast.literal_eval(commands_text)
        if not isinstance(commands, list) or not all(
            isinstance(command, str) for command in commands
        ):
            raise RuntimeError("commands must be a list of strings")
        return commands

    return [
        command.strip()
        for command in commands_text.split(";")
        if command.strip()
    ]


def build_nodes(context):
    commands = parse_commands_argument(
        LaunchConfiguration("commands").perform(context)
    )

    return [
        Node(
            package="rcj_localization",
            executable="stm32_behavior_node",
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
                    "baudrate": ParameterValue(
                        LaunchConfiguration("baudrate"), value_type=int
                    ),
                    "timeout_sec": ParameterValue(
                        LaunchConfiguration("timeout_sec"), value_type=float
                    ),
                    "resend_period_ms": ParameterValue(
                        LaunchConfiguration("resend_period_ms"), value_type=int
                    ),
                    "tick_period_ms": ParameterValue(
                        LaunchConfiguration("tick_period_ms"), value_type=int
                    ),
                    "enable_serial_log": ParameterValue(
                        LaunchConfiguration("enable_serial_log"), value_type=bool
                    ),
                    "enable_raw_reply_log": ParameterValue(
                        LaunchConfiguration("enable_raw_reply_log"), value_type=bool
                    ),
                    "commands": commands,
                }
            ],
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "node_name",
                default_value="stm32_behavior_node",
                description="ROS node name for the STM32 behavior node.",
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
                "timeout_sec",
                default_value="0.05",
                description="Serial read timeout in seconds.",
            ),
            DeclareLaunchArgument(
                "resend_period_ms",
                default_value="200",
                description="How often to resend the current command while waiting for ACK.",
            ),
            DeclareLaunchArgument(
                "tick_period_ms",
                default_value="20",
                description="Main behavior timer period in milliseconds.",
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
                description="ROS log level for the node.",
            ),
            DeclareLaunchArgument(
                "commands",
                default_value='["cmd_dis 100 -100", "cmd_turn 90", "cmd_dis 50 0"]',
                description=(
                    "STM32 command sequence. Use a Python list of strings or "
                    "semicolon-separated commands, e.g. 'cmd_dis 100 -100; cmd_turn 90'."
                ),
            ),
            OpaqueFunction(function=build_nodes),
        ]
    )
