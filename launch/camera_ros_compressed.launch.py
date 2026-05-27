from pathlib import Path
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcj_shared_launch_params import declare_camera_ros_arguments, camera_ros_parameters


def generate_launch_description() -> LaunchDescription:
    input_topic = LaunchConfiguration("input_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    compressed_topic = LaunchConfiguration("compressed_topic")
    jpeg_quality = LaunchConfiguration("jpeg_quality")
    max_fps = LaunchConfiguration("max_fps")
    lazy = LaunchConfiguration("lazy")
    log_size_stats = LaunchConfiguration("log_size_stats")
    size_log_interval = LaunchConfiguration("size_log_interval")

    return LaunchDescription(
        [
            *declare_camera_ros_arguments(),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera_info",
                description="Remapped CameraInfo topic.",
            ),
            DeclareLaunchArgument(
                "input_topic",
                default_value="/camera/image_raw",
                description="Remapped raw camera image topic.",
            ),
            DeclareLaunchArgument(
                "compressed_topic",
                default_value="/foxglove/camera/image_compressed",
                description="Compressed image output topic.",
            ),
            DeclareLaunchArgument(
                "jpeg_quality",
                default_value="80",
                description="JPEG quality for compressed camera images.",
            ),
            DeclareLaunchArgument(
                "max_fps",
                default_value="0.0",
                description="Maximum compressed publish rate. Use 0.0 for no limit.",
            ),
            DeclareLaunchArgument(
                "lazy",
                default_value="false",
                description="Only subscribe to raw images when compressed topic has subscribers.",
            ),
            DeclareLaunchArgument(
                "log_size_stats",
                default_value="true",
                description="Print raw and compressed image sizes periodically.",
            ),
            DeclareLaunchArgument(
                "size_log_interval",
                default_value="30",
                description="Print one size comparison every N compressed frames.",
            ),
            Node(
                package="camera_ros",
                executable="camera_node",
                name="camera",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                remappings=[
                    ("~/image_raw", input_topic),
                    ("~/camera_info", camera_info_topic),
                ],
                parameters=[camera_ros_parameters()],
            ),
            Node(
                package="rcj_localization",
                executable="image_compression_relay.py",
                name="camera_image_compression_relay",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {
                        "relay_name": "camera",
                        "input_topic": input_topic,
                        "output_topic": compressed_topic,
                        "jpeg_quality": ParameterValue(jpeg_quality, value_type=int),
                        "max_fps": ParameterValue(max_fps, value_type=float),
                        "lazy": ParameterValue(lazy, value_type=bool),
                        "log_size_stats": ParameterValue(
                            log_size_stats, value_type=bool
                        ),
                        "size_log_interval": ParameterValue(
                            size_log_interval, value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
