from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("rcj_localization"))
    default_lut = package_share / "config" / "raw_ball_top_lut_20260422_104020.xml"

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
            DeclareLaunchArgument("camera_index", default_value="0"),  # Camera device index
            DeclareLaunchArgument("role", default_value="viewfinder"),  # camera_ros capture role
            DeclareLaunchArgument("format", default_value="RGB888"),  # Camera pixel format
            DeclareLaunchArgument("width", default_value="800"),  # Capture width in pixels
            DeclareLaunchArgument("height", default_value="600"),  # Capture height in pixels
            DeclareLaunchArgument("orientation", default_value="0"),  # Camera image rotation angle
            DeclareLaunchArgument("sensor_mode", default_value="1332:990"),  # Camera sensor mode
            DeclareLaunchArgument("frame_id", default_value="camera"),  # Frame ID for published images
            DeclareLaunchArgument("camera_info_url", default_value=""),  # Camera calibration URL
            DeclareLaunchArgument("use_node_time", default_value="false"),  # Whether camera_ros uses node clock
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),  # CameraInfo topic name
            DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),  # Raw image topic name
            DeclareLaunchArgument("lut_file", default_value=str(default_lut)),  # LUT XML file for pixel-to-ground projection
            DeclareLaunchArgument("orange_h_min", default_value="5"),  # Minimum orange hue threshold
            DeclareLaunchArgument("orange_h_max", default_value="30"),  # Maximum orange hue threshold
            DeclareLaunchArgument("orange_s_min", default_value="100"),  # Minimum orange saturation threshold
            DeclareLaunchArgument("orange_v_min", default_value="60"),  # Minimum orange value threshold
            DeclareLaunchArgument("enable_morph_open", default_value="true"),  # Whether to run morphological opening on the mask
            DeclareLaunchArgument("morph_kernel_size", default_value="3"),  # Morphological opening kernel size
            DeclareLaunchArgument("search_downsample_scale", default_value="0.5"),  # Downsample scale used during global search
            DeclareLaunchArgument("min_blob_area_px", default_value="30"),  # Minimum accepted blob area in pixels
            DeclareLaunchArgument("max_blob_area_px", default_value="1500"),  # Maximum accepted blob area in pixels
            DeclareLaunchArgument("enable_distance_aware_area_prior", default_value="true"),  # Whether to enable the distance-aware ball-area prior optimization
            DeclareLaunchArgument("area_prior_near_min_ratio", default_value="0.75"),  # Minimum allowed area/expected-area ratio at the nearest LUT distance
            DeclareLaunchArgument("area_prior_near_max_ratio", default_value="1.25"),  # Maximum allowed area/expected-area ratio at the nearest LUT distance
            DeclareLaunchArgument("area_prior_far_min_ratio", default_value="0.50"),  # Minimum allowed area/expected-area ratio at the farthest LUT distance
            DeclareLaunchArgument("area_prior_far_max_ratio", default_value="2.00"),  # Maximum allowed area/expected-area ratio at the farthest LUT distance
            DeclareLaunchArgument("min_aspect_ratio", default_value="0.3"),  # Minimum accepted blob width/height ratio
            DeclareLaunchArgument("max_aspect_ratio", default_value="3"),  # Maximum accepted blob width/height ratio
            DeclareLaunchArgument("max_edge_touch_ratio", default_value="0.35"),  # Maximum allowed image-border contact ratio
            DeclareLaunchArgument("top_band_px", default_value="4"),  # Thickness of the top-band used for top-point estimation
            DeclareLaunchArgument("roi_scale", default_value="2.0"),  # ROI expansion scale during tracking
            DeclareLaunchArgument("roi_min_half_size_px", default_value="50"),  # Minimum ROI half-size in pixels
            DeclareLaunchArgument("roi_max_half_size_px", default_value="140"),  # Maximum ROI half-size in pixels
            DeclareLaunchArgument("lost_frame_tolerance", default_value="3"),  # Number of missed frames allowed before reset
            DeclareLaunchArgument("ema_alpha", default_value="0.5"),  # EMA smoothing factor for tracked outputs
            DeclareLaunchArgument("force_search_mode", default_value="false"),  # Force the detector to stay in search mode for debugging
            DeclareLaunchArgument("enable_timing_log", default_value="true"),  # Whether to print timing summaries
            DeclareLaunchArgument("timing_log_interval", default_value="30"),  # Frame interval between timing log summaries
            DeclareLaunchArgument("publish_processing_time", default_value="true"),  # Whether to publish end-to-end processing time
            DeclareLaunchArgument(
                "processing_time_topic", default_value="/orange_ball_detector/processing_time_ms"
            ),  # Topic for published processing time in milliseconds
            DeclareLaunchArgument(
                "enable_image_view", default_value="true"
            ),  # Master switch for all OpenCV debug windows
            DeclareLaunchArgument(
                "show_input_image", default_value="false"
            ),  # Show the full raw camera image when enable_image_view is true
            DeclareLaunchArgument(
                "show_threshold_mask", default_value="false"
            ),  # Show the pre-morph HSV threshold mask when enable_image_view is true
            DeclareLaunchArgument(
                "show_morph_mask", default_value="false"
            ),  # Show the post-morph mask when enable_image_view is true
            DeclareLaunchArgument(
                "show_raw_mask", default_value="false"
            ),  # Show the compatibility full-frame detector mask when enable_image_view is true
            DeclareLaunchArgument(
                "show_area_filter", default_value="true"
            ),  # Show components that survive the area filter
            DeclareLaunchArgument(
                "show_area_prior_filter", default_value="true"
            ),  # Show components that survive the distance-aware area-prior filter
            DeclareLaunchArgument(
                "show_area_prior_debug", default_value="true"
            ),  # Show area-prior actual/expected/range/score annotations for ROI candidates
            DeclareLaunchArgument(
                "show_aspect_filter", default_value="false"
            ),  # Show components that survive the aspect-ratio filter
            DeclareLaunchArgument(
                "show_edge_filter", default_value="false"
            ),  # Show components that survive the edge-touch filter
            DeclareLaunchArgument(
                "show_fill_filter", default_value="false"
            ),  # Show components that survive the fill-ratio filter
            DeclareLaunchArgument(
                "show_lut_filter", default_value="false"
            ),  # Show components that survive the LUT-validity filter
            DeclareLaunchArgument(
                "show_top_band_filter", default_value="false"
            ),  # Show components that survive the top-band validity filter
            DeclareLaunchArgument(
                "show_filtered_mask", default_value="false"
            ),  # Show the accepted connected-component mask when enable_image_view is true
            DeclareLaunchArgument(
                "show_search_debug", default_value="false"
            ),  # Show the coarse-search debug image in search mode
            DeclareLaunchArgument(
                "show_search_area_prior", default_value="true"
            ),  # Show coarse-search area-prior pass/fail annotations in search mode
            DeclareLaunchArgument(
                "show_overlay_image", default_value="false"
            ),  # Show the annotated overlay image when enable_image_view is true
            DeclareLaunchArgument(
                "show_roi_image", default_value="false"
            ),  # Show the current search/track ROI crop when enable_image_view is true
            DeclareLaunchArgument(
                "show_roi_mask", default_value="false"
            ),  # Show the orange mask inside the current ROI when enable_image_view is true
            DeclareLaunchArgument("display_max_width", default_value="960"),  # Maximum debug window width
            DeclareLaunchArgument("display_max_height", default_value="720"),  # Maximum debug window height
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
                        "enable_distance_aware_area_prior": ParameterValue(
                            LaunchConfiguration("enable_distance_aware_area_prior"),
                            value_type=bool,
                        ),
                        "area_prior_near_min_ratio": ParameterValue(
                            LaunchConfiguration("area_prior_near_min_ratio"), value_type=float
                        ),
                        "area_prior_near_max_ratio": ParameterValue(
                            LaunchConfiguration("area_prior_near_max_ratio"), value_type=float
                        ),
                        "area_prior_far_min_ratio": ParameterValue(
                            LaunchConfiguration("area_prior_far_min_ratio"), value_type=float
                        ),
                        "area_prior_far_max_ratio": ParameterValue(
                            LaunchConfiguration("area_prior_far_max_ratio"), value_type=float
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
                        "force_search_mode": ParameterValue(
                            LaunchConfiguration("force_search_mode"), value_type=bool
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
                        "show_threshold_mask": ParameterValue(
                            LaunchConfiguration("show_threshold_mask"), value_type=bool
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
                            LaunchConfiguration("show_filtered_mask"), value_type=bool
                        ),
                        "show_area_filter": ParameterValue(
                            LaunchConfiguration("show_area_filter"), value_type=bool
                        ),
                        "show_area_prior_filter": ParameterValue(
                            LaunchConfiguration("show_area_prior_filter"), value_type=bool
                        ),
                        "show_area_prior_debug": ParameterValue(
                            LaunchConfiguration("show_area_prior_debug"), value_type=bool
                        ),
                        "show_aspect_filter": ParameterValue(
                            LaunchConfiguration("show_aspect_filter"), value_type=bool
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
                            LaunchConfiguration("show_top_band_filter"), value_type=bool
                        ),
                        "show_search_debug": ParameterValue(
                            LaunchConfiguration("show_search_debug"), value_type=bool
                        ),
                        "show_search_area_prior": ParameterValue(
                            LaunchConfiguration("show_search_area_prior"), value_type=bool
                        ),
                        "show_overlay_image": ParameterValue(
                            LaunchConfiguration("show_overlay_image"), value_type=bool
                        ),
                        "show_roi_image": ParameterValue(
                            LaunchConfiguration("show_roi_image"), value_type=bool
                        ),
                        "show_roi_mask": ParameterValue(
                            LaunchConfiguration("show_roi_mask"), value_type=bool
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
