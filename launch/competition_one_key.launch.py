import os
import select
import sys
import termios
import threading
import time
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
_KEYBOARD_START_KEYS = ("s", "S")
_keyboard_terminal_fd = None
_keyboard_terminal_settings = None
_keyboard_listener_started = False
_keyboard_start_published = False
_keyboard_rclpy = None
_keyboard_ros_node = None
_keyboard_start_publisher = None
_keyboard_ros_initialized = False


def _is_mode(mode: str) -> PythonExpression:
    return PythonExpression(["'", LaunchConfiguration("mode"), "' == '", mode, "'"])


def _start_keyboard_control_listener(context):
    global _keyboard_listener_started
    global _keyboard_terminal_fd
    global _keyboard_terminal_settings
    global _keyboard_rclpy
    global _keyboard_ros_node
    global _keyboard_start_publisher
    global _keyboard_ros_initialized

    if _keyboard_listener_started:
        return []
    _keyboard_listener_started = True

    if sys.stdin is None or not sys.stdin.isatty():
        print("[competition_one_key] Keyboard shutdown disabled: stdin is not a TTY.")
        return []

    start_topic = LaunchConfiguration("start_topic").perform(context)
    wait_for_start = LaunchConfiguration("wait_for_start").perform(context).lower()
    wait_for_start_enabled = wait_for_start in ("1", "true", "yes", "on")

    if wait_for_start_enabled:
        try:
            import rclpy
            from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
            from std_msgs.msg import Empty

            _keyboard_rclpy = rclpy
            try:
                rclpy.init(args=None)
                _keyboard_ros_initialized = True
            except RuntimeError as error:
                if "already initialized" not in str(error):
                    raise

            _keyboard_ros_node = rclpy.create_node("competition_launch_keyboard")
            _keyboard_start_publisher = _keyboard_ros_node.create_publisher(
                Empty,
                start_topic,
                QoSProfile(
                    depth=1,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    reliability=ReliabilityPolicy.RELIABLE,
                ),
            )
        except Exception as error:  # noqa: BLE001
            print(
                "[competition_one_key] Keyboard start disabled: "
                f"failed to create start publisher: {error}"
            )
            wait_for_start_enabled = False

    _keyboard_terminal_fd = sys.stdin.fileno()
    _keyboard_terminal_settings = termios.tcgetattr(_keyboard_terminal_fd)
    tty.setcbreak(_keyboard_terminal_fd)
    if wait_for_start_enabled:
        print(
            f"[competition_one_key] Press s to start ({start_topic}); "
            "press q or Esc to shutdown this launch."
        )
    else:
        print("[competition_one_key] Press q or Esc to shutdown this launch.")

    def _listen_for_keyboard():
        try:
            while True:
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not ready:
                    continue

                key = os.read(_keyboard_terminal_fd, 1).decode(errors="ignore")
                if wait_for_start_enabled and key in _KEYBOARD_START_KEYS:
                    _publish_start_signal(start_topic)
                    continue
                if key in _KEYBOARD_STOP_KEYS:
                    print("[competition_one_key] Keyboard shutdown requested.")
                    context.emit_event_sync(
                        Shutdown(reason="competition keyboard stop")
                    )
                    return
        finally:
            _restore_keyboard_terminal()

    thread = threading.Thread(
        target=_listen_for_keyboard,
        name="competition_keyboard_control",
        daemon=True,
    )
    thread.start()
    return []


def _publish_start_signal(start_topic: str):
    global _keyboard_start_published

    if _keyboard_start_published:
        print("[competition_one_key] Start signal was already published.")
        return
    if _keyboard_rclpy is None or _keyboard_ros_node is None:
        print("[competition_one_key] Start publisher is not available.")
        return

    from std_msgs.msg import Empty

    _keyboard_start_published = True
    print(f"[competition_one_key] Publishing start signal on {start_topic}.")
    msg = Empty()
    for _ in range(5):
        _keyboard_start_publisher.publish(msg)
        _keyboard_rclpy.spin_once(_keyboard_ros_node, timeout_sec=0.0)
        time.sleep(0.05)


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
    _shutdown_keyboard_ros()
    return []


def _shutdown_keyboard_ros():
    global _keyboard_ros_node
    global _keyboard_start_publisher

    if _keyboard_ros_node is not None:
        _keyboard_ros_node.destroy_node()
        _keyboard_ros_node = None
        _keyboard_start_publisher = None
    if _keyboard_rclpy is not None and _keyboard_ros_initialized:
        _keyboard_rclpy.shutdown()


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    start_delay_sec = LaunchConfiguration("start_delay_sec")
    start_topic = LaunchConfiguration("start_topic")
    wait_for_start = LaunchConfiguration("wait_for_start")
    yaw_zero_map_degrees = LaunchConfiguration("yaw_zero_map_degrees")
    log_level = LaunchConfiguration("log_level")

    runner_parameters = [
        {
            "stm32_command_service": "/stm32/send_command",
            "motion_action_name": "/stm32/motion",
            "ball_detection_topic": "/orange_ball_detector/detection",
            "pose_topic": "/amcl_pose",
            "wait_for_start": ParameterValue(wait_for_start, value_type=bool),
            "start_topic": start_topic,
            "yaw_zero_map_degrees": ParameterValue(
                yaw_zero_map_degrees,
                value_type=float,
            ),
        }
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mode",
                default_value="offence",
                choices=("offence", "defense"),
                description=(
                    "Which behavior runner to prestart. Use "
                    "competition_ready.launch.py to start the base stack and runner "
                    "from one launch."
                ),
            ),
            DeclareLaunchArgument(
                "start_delay_sec",
                default_value="0.0",
                description=(
                    "Optional delay before prestarting the behavior runner. With "
                    "wait_for_start=true, the task logic still waits for s."
                ),
            ),
            DeclareLaunchArgument(
                "wait_for_start",
                default_value="true",
                description=(
                    "Start the selected runner immediately, preflight it, then "
                    "wait for /competition/start before run_task()."
                ),
            ),
            DeclareLaunchArgument(
                "start_topic",
                default_value="/competition/start",
                description="Topic used to release the prestarted competition runner.",
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
            OpaqueFunction(function=_start_keyboard_control_listener),
            RegisterEventHandler(OnShutdown(on_shutdown=_on_launch_shutdown)),
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
