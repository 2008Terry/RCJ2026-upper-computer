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
    competition_launch_file = package_share / "launch" / "competition.launch.py"

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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(competition_launch_file)),
                launch_arguments={
                    "stm32_port": LaunchConfiguration("stm32_port"),
                    "stm32_baudrate": LaunchConfiguration("stm32_baudrate"),
                    "yaw_zero_map_degrees": LaunchConfiguration("yaw_zero_map_degrees"),
                    # Keep the base stack warm without flooding the terminal or disk.
                    "stm32_enable_odometry_log": "false",
                    "stm32_enable_serial_log": "false",
                    "stm32_enable_raw_reply_log": "false",
                    "enable_camera_compressed_debug": "false",
                    "ridge_publish_debug_images": "false",
                    "ridge_publish_debug_image": "false",
                    "publish_debug_pointcloud": "false",
                    "publish_particle_weight_markers": "false",
                    "orange_publish_debug_images": "false",
                    "publish_raw_mask": "false",
                    "publish_filtered_mask": "false",
                    "publish_overlay_image": "false",
                }.items(),
            ),
        ]
    )
