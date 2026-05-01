from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("rcj_localization"))
    imx_amcl_launch_file = (
        package_share / "launch" / "imx477_hsv_dt_fastmap_amcl.launch.py"
    )
    default_lut = package_share / "config" / "raw_ball_top_lut_20260422_104020.xml"
    default_robot_mask = package_share / "config" / "mask.png"

    input_topic = LaunchConfiguration("input_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    enable_amcl_stack = LaunchConfiguration("enable_amcl_stack")
    enable_orange_ball_detector = LaunchConfiguration("enable_orange_ball_detector")
    enable_camera_compressed_debug = LaunchConfiguration("enable_camera_compressed_debug")
    lut_file = LaunchConfiguration("lut_file")
    orange_robot_mask_path = LaunchConfiguration("orange_robot_mask_path")
    yaw_zero_map_degrees = LaunchConfiguration("yaw_zero_map_degrees")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_amcl_stack",
                default_value="true",
                description="Start the IMX477 white-line FastMap AMCL stack.",
            ),
            DeclareLaunchArgument(
                "enable_orange_ball_detector",
                default_value="true",
                description="Start orange_ball_detector_node using the shared camera image.",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/camera_info",
                description="Shared CameraInfo topic published by the IMX477 stack.",
            ),
            DeclareLaunchArgument(
                "input_topic",
                default_value="/camera/image_raw",
                description="Shared raw image topic published by the single camera node.",
            ),
            DeclareLaunchArgument(
                "debug_jpeg_quality",
                default_value="80",
                description="JPEG quality for compressed debug image topics.",
            ),
            DeclareLaunchArgument(
                "debug_image_max_fps",
                default_value="5.0",
                description="Maximum publish rate for lazy debug image topics.",
            ),
            DeclareLaunchArgument(
                "enable_camera_compressed_debug",
                default_value="true",
                description="Start the lazy /camera/image_raw/compressed_debug relay.",
            ),
            DeclareLaunchArgument(
                "lut_file",
                default_value=str(default_lut),
                description="Orange ball pixel-to-ground LUT XML file.",
            ),
            DeclareLaunchArgument(
                "orange_robot_mask_path",
                default_value=str(default_robot_mask),
                description="Raw-space robot allow-mask image for orange ball detection.",
            ),
            DeclareLaunchArgument(
                "yaw_zero_map_degrees",
                default_value="0.0",
                description=(
                    "Map yaw offset used by AMCL and CompetitionRobot when "
                    "converting between ROS map yaw and STM32 yaw."
                ),
            ),
            DeclareLaunchArgument("orange_h_min", default_value="5"),
            DeclareLaunchArgument("orange_h_max", default_value="30"),
            DeclareLaunchArgument("orange_s_min", default_value="100"),
            DeclareLaunchArgument("orange_v_min", default_value="60"),
            DeclareLaunchArgument("enable_morph_open", default_value="true"),
            DeclareLaunchArgument("morph_kernel_size", default_value="3"),
            DeclareLaunchArgument("search_downsample_scale", default_value="0.5"),
            DeclareLaunchArgument("min_blob_area_px", default_value="30"),
            DeclareLaunchArgument("max_blob_area_px", default_value="1500"),
            DeclareLaunchArgument(
                "enable_distance_aware_area_prior", default_value="true"
            ),
            DeclareLaunchArgument("area_prior_near_min_ratio", default_value="0.66"),
            DeclareLaunchArgument("area_prior_near_max_ratio", default_value="1.5"),
            DeclareLaunchArgument("area_prior_far_min_ratio", default_value="0.50"),
            DeclareLaunchArgument("area_prior_far_max_ratio", default_value="2.00"),
            DeclareLaunchArgument("min_aspect_ratio", default_value="0.3"),
            DeclareLaunchArgument("max_aspect_ratio", default_value="3"),
            DeclareLaunchArgument("max_edge_touch_ratio", default_value="0.35"),
            DeclareLaunchArgument("top_band_px", default_value="4"),
            DeclareLaunchArgument("roi_scale", default_value="2.0"),
            DeclareLaunchArgument("roi_min_half_size_px", default_value="50"),
            DeclareLaunchArgument("roi_max_half_size_px", default_value="140"),
            DeclareLaunchArgument("lost_frame_tolerance", default_value="3"),
            DeclareLaunchArgument("ema_alpha", default_value="0.5"),
            DeclareLaunchArgument("force_search_mode", default_value="false"),
            DeclareLaunchArgument("orange_enable_timing_log", default_value="true"),
            DeclareLaunchArgument("orange_timing_log_interval", default_value="30"),
            DeclareLaunchArgument("publish_processing_time", default_value="true"),
            DeclareLaunchArgument(
                "processing_time_topic",
                default_value="/orange_ball_detector/processing_time_ms",
            ),
            DeclareLaunchArgument("orange_publish_debug_images", default_value="false"),
            DeclareLaunchArgument("publish_raw_mask", default_value="true"),
            DeclareLaunchArgument("publish_filtered_mask", default_value="true"),
            DeclareLaunchArgument("publish_overlay_image", default_value="true"),
            DeclareLaunchArgument("orange_enable_image_view", default_value="false"),
            DeclareLaunchArgument("show_input_image", default_value="false"),
            DeclareLaunchArgument("show_threshold_mask", default_value="false"),
            DeclareLaunchArgument("show_morph_mask", default_value="false"),
            DeclareLaunchArgument("show_raw_mask", default_value="false"),
            DeclareLaunchArgument("show_area_filter", default_value="true"),
            DeclareLaunchArgument("show_area_prior_filter", default_value="true"),
            DeclareLaunchArgument("show_area_prior_debug", default_value="true"),
            DeclareLaunchArgument("show_aspect_filter", default_value="false"),
            DeclareLaunchArgument("show_edge_filter", default_value="false"),
            DeclareLaunchArgument("show_fill_filter", default_value="false"),
            DeclareLaunchArgument("show_lut_filter", default_value="false"),
            DeclareLaunchArgument("show_top_band_filter", default_value="false"),
            DeclareLaunchArgument("show_filtered_mask", default_value="false"),
            DeclareLaunchArgument("show_search_debug", default_value="false"),
            DeclareLaunchArgument("show_search_area_prior", default_value="true"),
            DeclareLaunchArgument("show_overlay_image", default_value="false"),
            DeclareLaunchArgument("show_roi_image", default_value="false"),
            DeclareLaunchArgument("show_roi_mask", default_value="false"),
            DeclareLaunchArgument("orange_display_max_width", default_value="960"),
            DeclareLaunchArgument("orange_display_max_height", default_value="720"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(imx_amcl_launch_file)),
                condition=IfCondition(enable_amcl_stack),
                launch_arguments={
                    "input_topic": input_topic,
                    "camera_info_topic": camera_info_topic,
                    "debug_jpeg_quality": LaunchConfiguration("debug_jpeg_quality"),
                    "debug_image_max_fps": LaunchConfiguration("debug_image_max_fps"),
                    "use_fake_yaw": "true",
                    "use_stm32_request_theta": "true",
                    "yaw_zero_map_degrees": yaw_zero_map_degrees,
                }.items(),
            ),
            Node(
                package="rcj_localization",
                executable="camera_compressed_debug_relay_node",
                name="camera_compressed_debug_relay",
                output="screen",
                condition=IfCondition(enable_camera_compressed_debug),
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {
                        "input_topic": input_topic,
                        "output_topic": "/camera/image_raw/compressed_debug",
                        "debug_jpeg_quality": ParameterValue(
                            LaunchConfiguration("debug_jpeg_quality"),
                            value_type=int,
                        ),
                        "debug_image_max_fps": ParameterValue(
                            LaunchConfiguration("debug_image_max_fps"),
                            value_type=float,
                        ),
                    }
                ],
            ),
            Node(
                package="rcj_localization",
                executable="orange_ball_detector_node",
                name="orange_ball_detector",
                output="screen",
                condition=IfCondition(enable_orange_ball_detector),
                arguments=["--ros-args", "--log-level", "info"],
                parameters=[
                    {
                        "input_topic": input_topic,
                        "lut_file": lut_file,
                        "robot_mask_path": orange_robot_mask_path,
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
                            LaunchConfiguration("enable_morph_open"),
                            value_type=bool,
                        ),
                        "morph_kernel_size": ParameterValue(
                            LaunchConfiguration("morph_kernel_size"), value_type=int
                        ),
                        "search_downsample_scale": ParameterValue(
                            LaunchConfiguration("search_downsample_scale"),
                            value_type=float,
                        ),
                        "min_blob_area_px": ParameterValue(
                            LaunchConfiguration("min_blob_area_px"), value_type=int
                        ),
                        "max_blob_area_px": ParameterValue(
                            LaunchConfiguration("max_blob_area_px"), value_type=int
                        ),
                        "enable_distance_aware_area_prior": ParameterValue(
                            LaunchConfiguration("enable_distance_aware_area_prior"),
                            value_type=bool,
                        ),
                        "area_prior_near_min_ratio": ParameterValue(
                            LaunchConfiguration("area_prior_near_min_ratio"),
                            value_type=float,
                        ),
                        "area_prior_near_max_ratio": ParameterValue(
                            LaunchConfiguration("area_prior_near_max_ratio"),
                            value_type=float,
                        ),
                        "area_prior_far_min_ratio": ParameterValue(
                            LaunchConfiguration("area_prior_far_min_ratio"),
                            value_type=float,
                        ),
                        "area_prior_far_max_ratio": ParameterValue(
                            LaunchConfiguration("area_prior_far_max_ratio"),
                            value_type=float,
                        ),
                        "min_aspect_ratio": ParameterValue(
                            LaunchConfiguration("min_aspect_ratio"), value_type=float
                        ),
                        "max_aspect_ratio": ParameterValue(
                            LaunchConfiguration("max_aspect_ratio"), value_type=float
                        ),
                        "max_edge_touch_ratio": ParameterValue(
                            LaunchConfiguration("max_edge_touch_ratio"),
                            value_type=float,
                        ),
                        "top_band_px": ParameterValue(
                            LaunchConfiguration("top_band_px"), value_type=int
                        ),
                        "roi_scale": ParameterValue(
                            LaunchConfiguration("roi_scale"), value_type=float
                        ),
                        "roi_min_half_size_px": ParameterValue(
                            LaunchConfiguration("roi_min_half_size_px"),
                            value_type=int,
                        ),
                        "roi_max_half_size_px": ParameterValue(
                            LaunchConfiguration("roi_max_half_size_px"),
                            value_type=int,
                        ),
                        "lost_frame_tolerance": ParameterValue(
                            LaunchConfiguration("lost_frame_tolerance"),
                            value_type=int,
                        ),
                        "ema_alpha": ParameterValue(
                            LaunchConfiguration("ema_alpha"), value_type=float
                        ),
                        "force_search_mode": ParameterValue(
                            LaunchConfiguration("force_search_mode"),
                            value_type=bool,
                        ),
                        "enable_timing_log": ParameterValue(
                            LaunchConfiguration("orange_enable_timing_log"),
                            value_type=bool,
                        ),
                        "timing_log_interval": ParameterValue(
                            LaunchConfiguration("orange_timing_log_interval"),
                            value_type=int,
                        ),
                        "publish_processing_time": ParameterValue(
                            LaunchConfiguration("publish_processing_time"),
                            value_type=bool,
                        ),
                        "processing_time_topic": LaunchConfiguration(
                            "processing_time_topic"
                        ),
                        "publish_debug_images": ParameterValue(
                            LaunchConfiguration("orange_publish_debug_images"),
                            value_type=bool,
                        ),
                        "debug_jpeg_quality": ParameterValue(
                            LaunchConfiguration("debug_jpeg_quality"),
                            value_type=int,
                        ),
                        "debug_image_max_fps": ParameterValue(
                            LaunchConfiguration("debug_image_max_fps"),
                            value_type=float,
                        ),
                        "publish_raw_mask": ParameterValue(
                            LaunchConfiguration("publish_raw_mask"), value_type=bool
                        ),
                        "publish_filtered_mask": ParameterValue(
                            LaunchConfiguration("publish_filtered_mask"),
                            value_type=bool,
                        ),
                        "publish_overlay_image": ParameterValue(
                            LaunchConfiguration("publish_overlay_image"),
                            value_type=bool,
                        ),
                        "enable_image_view": ParameterValue(
                            LaunchConfiguration("orange_enable_image_view"),
                            value_type=bool,
                        ),
                        "show_threshold_mask": ParameterValue(
                            LaunchConfiguration("show_threshold_mask"),
                            value_type=bool,
                        ),
                        "show_morph_mask": ParameterValue(
                            LaunchConfiguration("show_morph_mask"), value_type=bool
                        ),
                        "show_input_image": ParameterValue(
                            LaunchConfiguration("show_input_image"), value_type=bool
                        ),
                        "show_raw_mask": ParameterValue(
                            LaunchConfiguration("show_raw_mask"), value_type=bool
                        ),
                        "show_filtered_mask": ParameterValue(
                            LaunchConfiguration("show_filtered_mask"),
                            value_type=bool,
                        ),
                        "show_area_filter": ParameterValue(
                            LaunchConfiguration("show_area_filter"),
                            value_type=bool,
                        ),
                        "show_area_prior_filter": ParameterValue(
                            LaunchConfiguration("show_area_prior_filter"),
                            value_type=bool,
                        ),
                        "show_area_prior_debug": ParameterValue(
                            LaunchConfiguration("show_area_prior_debug"),
                            value_type=bool,
                        ),
                        "show_aspect_filter": ParameterValue(
                            LaunchConfiguration("show_aspect_filter"),
                            value_type=bool,
                        ),
                        "show_edge_filter": ParameterValue(
                            LaunchConfiguration("show_edge_filter"), value_type=bool
                        ),
                        "show_fill_filter": ParameterValue(
                            LaunchConfiguration("show_fill_filter"), value_type=bool
                        ),
                        "show_lut_filter": ParameterValue(
                            LaunchConfiguration("show_lut_filter"), value_type=bool
                        ),
                        "show_top_band_filter": ParameterValue(
                            LaunchConfiguration("show_top_band_filter"),
                            value_type=bool,
                        ),
                        "show_search_debug": ParameterValue(
                            LaunchConfiguration("show_search_debug"),
                            value_type=bool,
                        ),
                        "show_search_area_prior": ParameterValue(
                            LaunchConfiguration("show_search_area_prior"),
                            value_type=bool,
                        ),
                        "show_overlay_image": ParameterValue(
                            LaunchConfiguration("show_overlay_image"),
                            value_type=bool,
                        ),
                        "show_roi_image": ParameterValue(
                            LaunchConfiguration("show_roi_image"), value_type=bool
                        ),
                        "show_roi_mask": ParameterValue(
                            LaunchConfiguration("show_roi_mask"), value_type=bool
                        ),
                        "display_max_width": ParameterValue(
                            LaunchConfiguration("orange_display_max_width"),
                            value_type=int,
                        ),
                        "display_max_height": ParameterValue(
                            LaunchConfiguration("orange_display_max_height"),
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )
