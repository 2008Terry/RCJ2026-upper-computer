import os
import select
import sys
import termios
import threading
import tty

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_KEYBOARD_STOP_KEYS = ("q", "Q", "\x1b")
_keyboard_terminal_fd = None
_keyboard_terminal_settings = None
_keyboard_listener_started = False


def _is_mode(mode: str) -> PythonExpression:
    return PythonExpression(["'", LaunchConfiguration("mode"), "' == '", mode, "'"])


def _start_keyboard_shutdown_listener(context):
    global _keyboard_listener_started
    global _keyboard_terminal_fd
    global _keyboard_terminal_settings

    if _keyboard_listener_started:
        return []
    _keyboard_listener_started = True

    if sys.stdin is None or not sys.stdin.isatty():
        print("[competition_one_key] Keyboard shutdown disabled: stdin is not a TTY.")
        return []

    _keyboard_terminal_fd = sys.stdin.fileno()
    _keyboard_terminal_settings = termios.tcgetattr(_keyboard_terminal_fd)
    tty.setcbreak(_keyboard_terminal_fd)
    print("[competition_one_key] Press q or Esc to shutdown this launch.")

    def _listen_for_stop_key():
        try:
            while True:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue

                key = os.read(_keyboard_terminal_fd, 1).decode(errors="ignore")
                if key in _KEYBOARD_STOP_KEYS:
                    print("[competition_one_key] Keyboard shutdown requested.")
                    context.emit_event_sync(
                        Shutdown(reason="competition keyboard stop")
                    )
                    return
        finally:
            _restore_keyboard_terminal()

    thread = threading.Thread(
        target=_listen_for_stop_key,
        name="competition_keyboard_shutdown",
        daemon=True,
    )
    thread.start()
    return []


def _restore_keyboard_terminal():
    global _keyboard_terminal_fd
    global _keyboard_terminal_settings

    if _keyboard_terminal_fd is None or _keyboard_terminal_settings is None:
        return
    termios.tcsetattr(
        _keyboard_terminal_fd,
        termios.TCSADRAIN,
        _keyboard_terminal_settings,
    )
    _keyboard_terminal_fd = None
    _keyboard_terminal_settings = None


def _on_launch_shutdown(event, context):
    _restore_keyboard_terminal()
    return []


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    start_delay_sec = LaunchConfiguration("start_delay_sec")
    yaw_zero_map_degrees = LaunchConfiguration("yaw_zero_map_degrees")
    log_level = LaunchConfiguration("log_level")

    runner_parameters = [
        {
            "stm32_command_service": "/stm32/send_command",
            "motion_action_name": "/stm32/motion",
            "ball_detection_topic": "/orange_ball_detector/detection",
            "pose_topic": "/amcl_pose",
            "yaw_zero_map_degrees": ParameterValue(
                yaw_zero_map_degrees,
                value_type=float,
            ),
        }
    ]

    return LaunchDescription(
        [
            OpaqueFunction(function=_start_keyboard_shutdown_listener),
            RegisterEventHandler(OnShutdown(on_shutdown=_on_launch_shutdown)),
            DeclareLaunchArgument(
                "mode",
                default_value="offence",
                choices=("offence", "defense"),
                description=(
                    "Which behavior runner to start. Start competition_base.launch.py "
                    "before this so STM32/camera/localization are already online."
                ),
            ),
            DeclareLaunchArgument(
                "start_delay_sec",
                default_value="0.0",
                description=(
                    "Optional countdown before starting the behavior runner. Use 0 for "
                    "instant match start after the base stack is already warm."
                ),
            ),
            DeclareLaunchArgument(
                "yaw_zero_map_degrees",
                default_value="0.0",
                description="Map yaw offset used by CompetitionRobot.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="warn",
                description="ROS log level for the behavior runner.",
            ),
            TimerAction(
                period=start_delay_sec,
                actions=[
                    Node(
                        package="rcj_localization",
                        executable="offence_runner.py",
                        name="competition_offence_runner",
                        output="screen",
                        condition=IfCondition(_is_mode("offence")),
                        arguments=["--ros-args", "--log-level", log_level],
                        parameters=runner_parameters,
                    ),
                    Node(
                        package="rcj_localization",
                        executable="defense_runner.py",
                        name="competition_defense_runner",
                        output="screen",
                        condition=IfCondition(_is_mode("defense")),
                        arguments=["--ros-args", "--log-level", log_level],
                        parameters=runner_parameters,
                    ),
                ],
            ),
        ]
    )
