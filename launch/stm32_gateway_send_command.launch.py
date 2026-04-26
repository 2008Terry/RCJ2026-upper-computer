import json
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


SERVICE_TYPE = "rcj_localization/srv/Stm32Command"


def build_send_command_action(context):
    command = LaunchConfiguration("command").perform(context).strip()
    if not command:
        raise RuntimeError("command launch argument must not be empty")

    service_name = LaunchConfiguration("service_name").perform(context).strip()
    if not service_name:
        raise RuntimeError("service_name launch argument must not be empty")

    delay_text = LaunchConfiguration("send_delay_sec").perform(context).strip()
    try:
        send_delay_sec = float(delay_text)
    except ValueError as error:
        raise RuntimeError("send_delay_sec must be a number") from error

    service_call = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            service_name,
            SERVICE_TYPE,
            "{command: " + json.dumps(command) + "}",
        ],
        output="screen",
    )

    if send_delay_sec <= 0.0:
        return [service_call]

    return [TimerAction(period=send_delay_sec, actions=[service_call])]


def generate_launch_description():
    package_share = Path(get_package_share_directory("rcj_localization"))
    gateway_launch_file = package_share / "launch" / "stm32_serial_gateway.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_gateway",
                default_value="true",
                description=(
                    "Start stm32_serial_gateway_node before sending the command. "
                    "Set false if the gateway is already running."
                ),
            ),
            DeclareLaunchArgument(
                "command",
                default_value="cmd_request",
                description=(
                    "One STM32 command to send through /stm32/send_command, e.g. "
                    "'cmd_request', 'cmd_dis 100 -100', or 'cmd_turn 90'."
                ),
            ),
            DeclareLaunchArgument(
                "service_name",
                default_value="/stm32/send_command",
                description="STM32 gateway command service name.",
            ),
            DeclareLaunchArgument(
                "send_delay_sec",
                default_value="1.0",
                description="Delay before sending the service request after launch starts.",
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
                description="How often to resend the active command while waiting for ACK.",
            ),
            DeclareLaunchArgument(
                "command_timeout_ms",
                default_value="50",
                description="Total timeout for one active command in milliseconds.",
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
                description="ROS log level for the gateway node.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gateway_launch_file)),
                condition=IfCondition(LaunchConfiguration("start_gateway")),
                launch_arguments={
                    "node_name": LaunchConfiguration("node_name"),
                    "port": LaunchConfiguration("port"),
                    "baudrate": LaunchConfiguration("baudrate"),
                    "tick_period_ms": LaunchConfiguration("tick_period_ms"),
                    "resend_period_ms": LaunchConfiguration("resend_period_ms"),
                    "command_timeout_ms": LaunchConfiguration("command_timeout_ms"),
                    "max_queue_size": LaunchConfiguration("max_queue_size"),
                    "enable_serial_log": LaunchConfiguration("enable_serial_log"),
                    "enable_raw_reply_log": LaunchConfiguration("enable_raw_reply_log"),
                    "log_level": LaunchConfiguration("log_level"),
                }.items(),
            ),
            OpaqueFunction(function=build_send_command_action),
        ]
    )
