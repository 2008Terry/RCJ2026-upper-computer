from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("rcj_localization"))
    default_lut = package_share / "config" / "raw_ball_top_lut.xml"

    camera_index = LaunchConfiguration("camera_index")
    role = LaunchConfiguration("role")
    image_format = LaunchConfiguration("format")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    orientation = LaunchConfiguration("orientation")
    sensor_mode = LaunchConfiguration("sensor_mode")
    frame_id = LaunchConfiguration("frame_id")
    camera_info_url = LaunchConfiguration("camera_info_url")
    use_node_time = LaunchConfiguration("use_node_time")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    input_topic = LaunchConfiguration("input_topic")
    lut_file = LaunchConfiguration("lut_file")

    return LaunchDescription(
        [
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
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),
            DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),
            DeclareLaunchArgument("lut_file", default_value=str(default_lut)),
            DeclareLaunchArgument("orange_h_min", default_value="5"),
            DeclareLaunchArgument("orange_h_max", default_value="30"),
            DeclareLaunchArgument("orange_s_min", default_value="100"),
            DeclareLaunchArgument("orange_v_min", default_value="60"),
            DeclareLaunchArgument("enable_morph_open", default_value="true"),
            DeclareLaunchArgument("morph_kernel_size", default_value="3"),
            DeclareLaunchArgument("search_downsample_scale", default_value="0.5"),
            DeclareLaunchArgument("min_blob_area_px", default_value="20"),
            DeclareLaunchArgument("max_blob_area_px", default_value="40000"),
            DeclareLaunchArgument("min_aspect_ratio", default_value="0.35"),
            DeclareLaunchArgument("max_aspect_ratio", default_value="2.8"),
            DeclareLaunchArgument("max_edge_touch_ratio", default_value="0.35"),
            DeclareLaunchArgument("top_band_px", default_value="4"),
            DeclareLaunchArgument("roi_scale", default_value="2.0"),
            DeclareLaunchArgument("roi_min_half_size_px", default_value="50"),
            DeclareLaunchArgument("roi_max_half_size_px", default_value="140"),
            DeclareLaunchArgument("lost_frame_tolerance", default_value="3"),
            DeclareLaunchArgument("ema_alpha", default_value="0.6"),
            DeclareLaunchArgument("enable_timing_log", default_value="true"),
            DeclareLaunchArgument("timing_log_interval", default_value="30"),
            DeclareLaunchArgument("publish_processing_time", default_value="true"),
            DeclareLaunchArgument(
                "processing_time_topic", default_value="/orange_ball_detector/processing_time_ms"
            ),
            DeclareLaunchArgument("enable_image_view", default_value="false"),
            DeclareLaunchArgument("display_max_width", default_value="960"),
            DeclareLaunchArgument("display_max_height", default_value="720"),
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
                parameters=[
                    {
                        "camera": ParameterValue(camera_index, value_type=int),
                        "role": role,
                        "format": image_format,
                        "width": ParameterValue(width, value_type=int),
                        "height": ParameterValue(height, value_type=int),
                        "orientation": ParameterValue(orientation, value_type=int),
                        "sensor_mode": sensor_mode,
                        "frame_id": frame_id,
                        "camera_info_url": camera_info_url,
                        "use_node_time": ParameterValue(use_node_time, value_type=bool),
                    }
                ],
            ),
            Node(
                package="rcj_localization",
                executable="orange_ball_detector_node",
                name="orange_ball_detector",
                output="screen",
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {
                        "input_topic": input_topic,
                        "lut_file": lut_file,
                        "orange_h_min": ParameterValue(
                            LaunchConfiguration("orange_h_min"), value_type=int
                        ),
                        "orange_h_max": ParameterValue(
                            LaunchConfiguration("orange_h_max"), value_type=int
                        ),
                        "orange_s_min": ParameterValue(
                            LaunchConfiguration("orange_s_min"), value_type=int
                        ),
                        "orange_v_min": ParameterValue(
                            LaunchConfiguration("orange_v_min"), value_type=int
                        ),
                        "enable_morph_open": ParameterValue(
                            LaunchConfiguration("enable_morph_open"), value_type=bool
                        ),
                        "morph_kernel_size": ParameterValue(
                            LaunchConfiguration("morph_kernel_size"), value_type=int
                        ),
                        "search_downsample_scale": ParameterValue(
                            LaunchConfiguration("search_downsample_scale"), value_type=float
                        ),
                        "min_blob_area_px": ParameterValue(
                            LaunchConfiguration("min_blob_area_px"), value_type=int
                        ),
                        "max_blob_area_px": ParameterValue(
                            LaunchConfiguration("max_blob_area_px"), value_type=int
                        ),
                        "min_aspect_ratio": ParameterValue(
                            LaunchConfiguration("min_aspect_ratio"), value_type=float
                        ),
                        "max_aspect_ratio": ParameterValue(
                            LaunchConfiguration("max_aspect_ratio"), value_type=float
                        ),
                        "max_edge_touch_ratio": ParameterValue(
                            LaunchConfiguration("max_edge_touch_ratio"), value_type=float
                        ),
                        "top_band_px": ParameterValue(
                            LaunchConfiguration("top_band_px"), value_type=int
                        ),
                        "roi_scale": ParameterValue(
                            LaunchConfiguration("roi_scale"), value_type=float
                        ),
                        "roi_min_half_size_px": ParameterValue(
                            LaunchConfiguration("roi_min_half_size_px"), value_type=int
                        ),
                        "roi_max_half_size_px": ParameterValue(
                            LaunchConfiguration("roi_max_half_size_px"), value_type=int
                        ),
                        "lost_frame_tolerance": ParameterValue(
                            LaunchConfiguration("lost_frame_tolerance"), value_type=int
                        ),
                        "ema_alpha": ParameterValue(
                            LaunchConfiguration("ema_alpha"), value_type=float
                        ),
                        "enable_timing_log": ParameterValue(
                            LaunchConfiguration("enable_timing_log"), value_type=bool
                        ),
                        "timing_log_interval": ParameterValue(
                            LaunchConfiguration("timing_log_interval"), value_type=int
                        ),
                        "publish_processing_time": ParameterValue(
                            LaunchConfiguration("publish_processing_time"), value_type=bool
                        ),
                        "processing_time_topic": LaunchConfiguration("processing_time_topic"),
                        "enable_image_view": ParameterValue(
                            LaunchConfiguration("enable_image_view"), value_type=bool
                        ),
                        "display_max_width": ParameterValue(
                            LaunchConfiguration("display_max_width"), value_type=int
                        ),
                        "display_max_height": ParameterValue(
                            LaunchConfiguration("display_max_height"), value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
