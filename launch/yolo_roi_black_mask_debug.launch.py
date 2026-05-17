from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcj_shared_launch_params import (
    declare_camera_control_arguments,
    declare_remap_interpolation_argument,
)


def generate_launch_description():
    package_share = Path(get_package_share_directory("rcj_localization"))
    yolo_launch_file = package_share / "launch" / "yolo_black_circle_debug.launch.py"

    remap_topic = LaunchConfiguration("remap_topic")
    roi_mask_topic = LaunchConfiguration("roi_mask_topic")

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
            DeclareLaunchArgument("max_det", default_value="1"),
            DeclareLaunchArgument("max_processing_hz", default_value="30.0"),
            DeclareLaunchArgument("production_mode", default_value="false"),
            DeclareLaunchArgument("web_host", default_value="0.0.0.0"),
            DeclareLaunchArgument("web_port", default_value="8081"),
            DeclareLaunchArgument("enable_web_viewer", default_value=""),
            DeclareLaunchArgument("enable_yolo_overlay", default_value=""),
            DeclareLaunchArgument("enable_black_mask_preview", default_value=""),
            DeclareLaunchArgument("jpeg_quality", default_value="85"),
            DeclareLaunchArgument("publish_debug_image", default_value=""),
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
            DeclareLaunchArgument("black_h_min", default_value="69"),
            DeclareLaunchArgument("black_h_max", default_value="92"),
            DeclareLaunchArgument("black_s_min", default_value="75"),
            DeclareLaunchArgument("black_s_max", default_value="255"),
            DeclareLaunchArgument("black_v_min", default_value="72"),
            DeclareLaunchArgument("black_v_max", default_value="141"),
            DeclareLaunchArgument("black_mask_sync_queue_size", default_value="60"),
            DeclareLaunchArgument("black_mask_publish_debug_image", default_value="true"),
            DeclareLaunchArgument(
                "black_mask_debug_image_topic",
                default_value="/yolo_roi_black_mask/debug/overlay_image",
            ),
            DeclareLaunchArgument("black_mask_enable_image_view", default_value="false"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(yolo_launch_file)),
                launch_arguments={
                    "enable_camera": LaunchConfiguration("enable_camera"),
                    "enable_fastmap_remap": LaunchConfiguration("enable_fastmap_remap"),
                    "camera_index": LaunchConfiguration("camera_index"),
                    "role": LaunchConfiguration("role"),
                    "format": LaunchConfiguration("format"),
                    "width": LaunchConfiguration("width"),
                    "height": LaunchConfiguration("height"),
                    "orientation": LaunchConfiguration("orientation"),
                    "sensor_mode": LaunchConfiguration("sensor_mode"),
                    "frame_id": LaunchConfiguration("frame_id"),
                    "camera_info_url": LaunchConfiguration("camera_info_url"),
                    "use_node_time": LaunchConfiguration("use_node_time"),
                    "camera_info_topic": LaunchConfiguration("camera_info_topic"),
                    "input_topic": LaunchConfiguration("input_topic"),
                    "exposure_time": LaunchConfiguration("exposure_time"),
                    "exposure_time_mode": LaunchConfiguration("exposure_time_mode"),
                    "ae_enable": LaunchConfiguration("ae_enable"),
                    "remap_topic": remap_topic,
                    "robot_mask_path": LaunchConfiguration("robot_mask_path"),
                    "input_transport": LaunchConfiguration("input_transport"),
                    "interpolation": LaunchConfiguration("interpolation"),
                    "remap_enable_image_view": LaunchConfiguration("remap_enable_image_view"),
                    "remap_enable_timing_log": LaunchConfiguration("remap_enable_timing_log"),
                    "remap_timing_log_interval": LaunchConfiguration(
                        "remap_timing_log_interval"
                    ),
                    "model_path": LaunchConfiguration("model_path"),
                    "confidence": LaunchConfiguration("confidence"),
                    "iou": LaunchConfiguration("iou"),
                    "imgsz": LaunchConfiguration("imgsz"),
                    "device": LaunchConfiguration("device"),
                    "max_det": LaunchConfiguration("max_det"),
                    "max_processing_hz": LaunchConfiguration("max_processing_hz"),
                    "production_mode": LaunchConfiguration("production_mode"),
                    "web_host": LaunchConfiguration("web_host"),
                    "web_port": LaunchConfiguration("web_port"),
                    "enable_web_viewer": LaunchConfiguration("enable_web_viewer"),
                    "enable_yolo_overlay": LaunchConfiguration("enable_yolo_overlay"),
                    "enable_black_mask_preview": LaunchConfiguration(
                        "enable_black_mask_preview"
                    ),
                    "jpeg_quality": LaunchConfiguration("jpeg_quality"),
                    "publish_debug_image": LaunchConfiguration("publish_debug_image"),
                    "debug_image_topic": LaunchConfiguration("debug_image_topic"),
                    "publish_roi_mask": LaunchConfiguration("publish_roi_mask"),
                    "roi_mask_topic": roi_mask_topic,
                    "log_interval": LaunchConfiguration("log_interval"),
                }.items(),
            ),
            Node(
                package="rcj_localization",
                executable="yolo_roi_black_mask_node",
                name="yolo_roi_black_mask",
                output="screen",
                parameters=[
                    {
                        "input_topic": remap_topic,
                        "roi_mask_topic": roi_mask_topic,
                        "black_h_min": ParameterValue(
                            LaunchConfiguration("black_h_min"), value_type=int
                        ),
                        "black_h_max": ParameterValue(
                            LaunchConfiguration("black_h_max"), value_type=int
                        ),
                        "black_s_min": ParameterValue(
                            LaunchConfiguration("black_s_min"), value_type=int
                        ),
                        "black_s_max": ParameterValue(
                            LaunchConfiguration("black_s_max"), value_type=int
                        ),
                        "black_v_min": ParameterValue(
                            LaunchConfiguration("black_v_min"), value_type=int
                        ),
                        "black_v_max": ParameterValue(
                            LaunchConfiguration("black_v_max"), value_type=int
                        ),
                        "sync_queue_size": ParameterValue(
                            LaunchConfiguration("black_mask_sync_queue_size"),
                            value_type=int,
                        ),
                        "publish_debug_image": ParameterValue(
                            LaunchConfiguration("black_mask_publish_debug_image"),
                            value_type=bool,
                        ),
                        "debug_image_topic": LaunchConfiguration(
                            "black_mask_debug_image_topic"
                        ),
                        "enable_image_view": ParameterValue(
                            LaunchConfiguration("black_mask_enable_image_view"),
                            value_type=bool,
                        ),
                    }
                ],
            ),
        ]
    )
