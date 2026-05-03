from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcj_shared_launch_params import declare_stm32_port_argument


def generate_launch_description():
    package_share = Path(get_package_share_directory("rcj_localization"))
    base_launch_file = package_share / "launch" / "competition_base.launch.py"
    runner_launch_file = package_share / "launch" / "competition_one_key.launch.py"

    return LaunchDescription(
        [
            declare_stm32_port_argument(),
            DeclareLaunchArgument(
                "stm32_baudrate",
                default_value="115200",
                description="STM32 serial baudrate.",
            ),
            DeclareLaunchArgument(
                "yaw_zero_map_degrees",
                default_value="0.0",
                description="Map yaw offset shared by AMCL and CompetitionRobot.",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="offence",
                choices=("offence", "defense"),
                description="Which behavior runner to prestart.",
            ),
            DeclareLaunchArgument(
                "wait_for_start",
                default_value="true",
                description="Wait for s before releasing run_task().",
            ),
            DeclareLaunchArgument(
                "start_topic",
                default_value="/competition/start",
                description="Topic used to release the prestarted competition runner.",
            ),
            DeclareLaunchArgument(
                "runner_start_delay_sec",
                default_value="0.0",
                description=(
                    "Optional delay before prestarting the behavior runner. "
                    "The runner still waits for s when wait_for_start=true."
                ),
            ),
            DeclareLaunchArgument(
                "runner_log_level",
                default_value="warn",
                description="ROS log level for the behavior runner.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(base_launch_file)),
                launch_arguments={
                    "stm32_port": LaunchConfiguration("stm32_port"),
                    "stm32_baudrate": LaunchConfiguration("stm32_baudrate"),
                    "yaw_zero_map_degrees": LaunchConfiguration(
                        "yaw_zero_map_degrees"
                    ),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(runner_launch_file)),
                launch_arguments={
                    "mode": LaunchConfiguration("mode"),
                    "start_delay_sec": LaunchConfiguration("runner_start_delay_sec"),
                    "wait_for_start": LaunchConfiguration("wait_for_start"),
                    "start_topic": LaunchConfiguration("start_topic"),
                    "yaw_zero_map_degrees": LaunchConfiguration(
                        "yaw_zero_map_degrees"
                    ),
                    "log_level": LaunchConfiguration("runner_log_level"),
                }.items(),
            ),
        ]
    )
