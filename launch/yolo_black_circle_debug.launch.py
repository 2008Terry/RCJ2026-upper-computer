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
from rcj_camera_config import resolve_fastmap_file


def read_fastmap_source_size(fastmap_file):
    root = ET.parse(fastmap_file).getroot()
    source_width = root.findtext("source_width")
    source_height = root.findtext("source_height")
    if source_width is None or source_height is None:
        raise RuntimeError(
            f"Fastmap XML '{fastmap_file}' is missing source_width/source_height"
        )
    return int(source_width), int(source_height)


def build_nodes(context):
    selected_fastmap_file = resolve_fastmap_file()
    default_width, default_height = read_fastmap_source_size(selected_fastmap_file)
    width_value = int(
        LaunchConfiguration("width").perform(context).strip() or str(default_width)
    )
    height_value = int(
        LaunchConfiguration("height").perform(context).strip() or str(default_height)
    )

    input_topic = LaunchConfiguration("input_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    remap_topic = LaunchConfiguration("remap_topic")

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
                    "output_topic": remap_topic,
                    "input_transport": LaunchConfiguration("input_transport"),
                    "interpolation": LaunchConfiguration("interpolation"),
                    "enable_image_view": ParameterValue(
                        LaunchConfiguration("remap_enable_image_view"),
                        value_type=bool,
                    ),
                    "show_input_image": False,
                    "show_output_image": False,
                    "enable_timing_log": ParameterValue(
                        LaunchConfiguration("remap_enable_timing_log"),
                        value_type=bool,
                    ),
                    "timing_log_interval": ParameterValue(
                        LaunchConfiguration("remap_timing_log_interval"),
                        value_type=int,
                    ),
                }
            ],
        ),
        Node(
            package="rcj_localization",
            executable="yolo_black_circle_debug.py",
            name="yolo_black_circle_debug",
            output="screen",
            parameters=[
                {
                    "input_topic": remap_topic,
                    "model_path": LaunchConfiguration("model_path"),
                    "confidence": ParameterValue(
                        LaunchConfiguration("confidence"), value_type=float
                    ),
                    "iou": ParameterValue(LaunchConfiguration("iou"), value_type=float),
                    "imgsz": ParameterValue(
                        LaunchConfiguration("imgsz"), value_type=int
                    ),
                    "device": LaunchConfiguration("device"),
                    "max_det": ParameterValue(
                        LaunchConfiguration("max_det"), value_type=int
                    ),
                    "max_processing_hz": ParameterValue(
                        LaunchConfiguration("max_processing_hz"), value_type=float
                    ),
                    "web_host": LaunchConfiguration("web_host"),
                    "web_port": ParameterValue(
                        LaunchConfiguration("web_port"), value_type=int
                    ),
                    "jpeg_quality": ParameterValue(
                        LaunchConfiguration("jpeg_quality"), value_type=int
                    ),
                    "publish_debug_image": ParameterValue(
                        LaunchConfiguration("publish_debug_image"), value_type=bool
                    ),
                    "debug_image_topic": LaunchConfiguration("debug_image_topic"),
                    "publish_roi_mask": ParameterValue(
                        LaunchConfiguration("publish_roi_mask"), value_type=bool
                    ),
                    "roi_mask_topic": LaunchConfiguration("roi_mask_topic"),
                    "log_interval": ParameterValue(
                        LaunchConfiguration("log_interval"), value_type=int
                    ),
                }
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
                "remap_topic",
                default_value="/black_feature_input_remap_node/image_remapped",
            ),
            DeclareLaunchArgument(
                "robot_mask_path",
                default_value=str(package_share / "config" / "remapped_mask.png"),
            ),
            DeclareLaunchArgument("input_transport", default_value="raw"),
            declare_remap_interpolation_argument(),
            DeclareLaunchArgument("remap_enable_image_view", default_value="false"),
            DeclareLaunchArgument("remap_enable_timing_log", default_value="true"),
            DeclareLaunchArgument("remap_timing_log_interval", default_value="30"),
            DeclareLaunchArgument(
                "model_path",
                default_value="~/Downloads/train-6/weights/best.pt",
            ),
            DeclareLaunchArgument("confidence", default_value="0.25"),
            DeclareLaunchArgument("iou", default_value="0.45"),
            DeclareLaunchArgument("imgsz", default_value="320"),
            DeclareLaunchArgument("device", default_value="cpu"),
            DeclareLaunchArgument("max_det", default_value="20"),
            DeclareLaunchArgument("max_processing_hz", default_value="30.0"),
            DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_port", default_value="8081"),
            DeclareLaunchArgument("jpeg_quality", default_value="85"),
            DeclareLaunchArgument("publish_debug_image", default_value="false"),
            DeclareLaunchArgument(
                "debug_image_topic",
                default_value="/yolo_black_circle_debug/debug_image",
            ),
            DeclareLaunchArgument("publish_roi_mask", default_value="true"),
            DeclareLaunchArgument(
                "roi_mask_topic",
                default_value="/yolo_black_circle_debug/roi_mask",
            ),
            DeclareLaunchArgument("log_interval", default_value="30"),
            OpaqueFunction(function=build_nodes),
        ]
    )
