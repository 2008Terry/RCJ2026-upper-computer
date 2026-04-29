from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcj_shared_launch_params import (
    camera_control_parameters,
    declare_camera_control_arguments,
    declare_remap_interpolation_argument,
)


def find_latest_fastmap_file():
    config_dir = Path(get_package_share_directory("rcj_localization")) / "config"
    candidates = sorted(config_dir.glob("undistort_map_*_fast.xml"))
    if not candidates:
        candidates = sorted(config_dir.glob("*.xml"))
    if not candidates:
        raise FileNotFoundError(f"No fastmap XML file found in {config_dir}")
    return candidates[-1]


def read_fastmap_source_size(fastmap_file):
    root = ET.parse(fastmap_file).getroot()
    source_width = root.findtext("source_width")
    source_height = root.findtext("source_height")
    if source_width is None or source_height is None:
        raise RuntimeError(
            f"Fastmap XML '{fastmap_file}' is missing source_width/source_height"
        )
    return int(source_width), int(source_height)


def resolve_fastmap_file(context):
    use_latest_fastmap = (
        LaunchConfiguration("use_latest_fastmap").perform(context).strip().lower()
        == "true"
    )
    fastmap_file_value = LaunchConfiguration("fastmap_file").perform(context).strip()

    if use_latest_fastmap or not fastmap_file_value:
        return find_latest_fastmap_file()

    fastmap_path = Path(fastmap_file_value).expanduser()
    if not fastmap_path.is_absolute():
        fastmap_path = fastmap_path.resolve()
    if not fastmap_path.exists():
        raise FileNotFoundError(f"Fastmap XML file does not exist: {fastmap_path}")
    return fastmap_path


def build_nodes(context):
    selected_fastmap_file = resolve_fastmap_file(context)
    default_width, default_height = read_fastmap_source_size(selected_fastmap_file)
    width_value = int(
        LaunchConfiguration("width").perform(context).strip() or str(default_width)
    )
    height_value = int(
        LaunchConfiguration("height").perform(context).strip() or str(default_height)
    )

    input_topic = LaunchConfiguration("input_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    output_topic = LaunchConfiguration("output_topic")

    return [
        Node(
            package="camera_ros",
            executable="camera_node",
            name="camera",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_camera")),
            arguments=["--ros-args", "--log-level", "info"],
            remappings=[
                ("~/image_raw", input_topic),
                ("~/camera_info", camera_info_topic),
            ],
            parameters=[
                {
                    "camera": ParameterValue(
                        LaunchConfiguration("camera_index"), value_type=int
                    ),
                    "role": LaunchConfiguration("role"),
                    "format": LaunchConfiguration("format"),
                    "width": ParameterValue(width_value, value_type=int),
                    "height": ParameterValue(height_value, value_type=int),
                    "orientation": ParameterValue(
                        LaunchConfiguration("orientation"), value_type=int
                    ),
                    "sensor_mode": LaunchConfiguration("sensor_mode"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "camera_info_url": LaunchConfiguration("camera_info_url"),
                    "use_node_time": ParameterValue(
                        LaunchConfiguration("use_node_time"), value_type=bool
                    ),
                    **camera_control_parameters(),
                }
            ],
        ),
        Node(
            package="rcj_localization",
            executable="fastmap_remap_node",
            name="black_feature_input_remap_node",
            output="screen",
            condition=IfCondition(LaunchConfiguration("enable_fastmap_remap")),
            parameters=[
                {
                    "fastmap_file": str(selected_fastmap_file),
                    "robot_mask_path": LaunchConfiguration("robot_mask_path"),
                    "input_topic": input_topic,
                    "output_topic": output_topic,
                    "input_transport": LaunchConfiguration("input_transport"),
                    "interpolation": LaunchConfiguration("interpolation"),
                    "enable_image_view": False,
                    "show_input_image": False,
                    "show_output_image": False,
                    "enable_timing_log": ParameterValue(
                        LaunchConfiguration("remap_enable_timing_log"), value_type=bool
                    ),
                    "timing_log_interval": ParameterValue(
                        LaunchConfiguration("remap_timing_log_interval"), value_type=int
                    ),
                }
            ],
        ),
        Node(
            package="rcj_localization",
            executable="remapped_photo_capture_ui.py",
            name="remapped_photo_capture_ui",
            output="screen",
            arguments=[
                "--input-topic",
                output_topic,
                "--output-dir",
                LaunchConfiguration("capture_output_dir"),
                "--host",
                LaunchConfiguration("capture_host"),
                "--port",
                LaunchConfiguration("capture_port"),
                "--prefix",
                LaunchConfiguration("capture_prefix"),
                "--format",
                LaunchConfiguration("capture_format"),
                "--target-count",
                LaunchConfiguration("capture_target_count"),
                "--jpeg-quality",
                LaunchConfiguration("capture_jpeg_quality"),
            ],
        ),
    ]


def generate_launch_description():
    package_share = Path(get_package_share_directory("rcj_localization"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_camera", default_value="true"),
            DeclareLaunchArgument("enable_fastmap_remap", default_value="true"),
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("role", default_value="viewfinder"),
            DeclareLaunchArgument("format", default_value="RGB888"),
            DeclareLaunchArgument("width", default_value="800"),
            DeclareLaunchArgument("height", default_value="600"),
            DeclareLaunchArgument("orientation", default_value="0"),
            DeclareLaunchArgument("sensor_mode", default_value="1332:990"),
            DeclareLaunchArgument("frame_id", default_value="camera"),
            DeclareLaunchArgument("camera_info_url", default_value=""),
            DeclareLaunchArgument("use_node_time", default_value="false"),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera_info"
            ),
            DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),
            *declare_camera_control_arguments(),
            DeclareLaunchArgument(
                "output_topic",
                default_value="/black_feature_input_remap_node/image_remapped",
            ),
            DeclareLaunchArgument("use_latest_fastmap", default_value="true"),
            DeclareLaunchArgument("fastmap_file", default_value=""),
            DeclareLaunchArgument(
                "robot_mask_path",
                default_value=str(package_share / "config" / "remapped_mask.png"),
            ),
            DeclareLaunchArgument("input_transport", default_value="raw"),
            declare_remap_interpolation_argument(),
            DeclareLaunchArgument("remap_enable_timing_log", default_value="true"),
            DeclareLaunchArgument("remap_timing_log_interval", default_value="30"),
            DeclareLaunchArgument(
                "capture_output_dir", default_value="~/rcj_remapped_captures"
            ),
            DeclareLaunchArgument("capture_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("capture_port", default_value="8080"),
            DeclareLaunchArgument("capture_prefix", default_value="remapped"),
            DeclareLaunchArgument("capture_format", default_value="png"),
            DeclareLaunchArgument("capture_target_count", default_value="300"),
            DeclareLaunchArgument("capture_jpeg_quality", default_value="85"),
            OpaqueFunction(function=build_nodes),
        ]
    )
