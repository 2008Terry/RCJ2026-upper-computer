from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _is_mode(mode: str) -> PythonExpression:
    return PythonExpression(["'", LaunchConfiguration("mode"), "' == '", mode, "'"])


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
