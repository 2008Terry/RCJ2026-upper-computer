from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
        LaunchConfiguration("use_latest_fastmap").perform(context).strip().lower() == "true"
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

    camera_index = LaunchConfiguration("camera_index")
    role = LaunchConfiguration("role")
    image_format = LaunchConfiguration("format")
    orientation = LaunchConfiguration("orientation")
    sensor_mode = LaunchConfiguration("sensor_mode")
    frame_id = LaunchConfiguration("frame_id")
    camera_info_url = LaunchConfiguration("camera_info_url")
    use_node_time = LaunchConfiguration("use_node_time")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    input_topic = LaunchConfiguration("input_topic")
    remap_topic = LaunchConfiguration("remap_topic")
    white_mask_topic = LaunchConfiguration("white_mask_topic")
    input_transport = LaunchConfiguration("input_transport")
    interpolation = LaunchConfiguration("interpolation")

    hsv_node_name = "white_line_hsv_white_node"

    return [
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
                    "camera": ParameterValue(camera_index, value_type=int),  # Camera index
                    "role": role,  # camera_ros role
                    "format": image_format,  # Camera pixel format
                    "width": ParameterValue(width_value, value_type=int),  # Capture width
                    "height": ParameterValue(height_value, value_type=int),  # Capture height
                    "orientation": ParameterValue(orientation, value_type=int),  # Camera rotation angle
                    "sensor_mode": sensor_mode,  # Camera sensor mode
                    "frame_id": frame_id,  # Image frame id
                    "camera_info_url": camera_info_url,  # Camera calibration URL
                    "use_node_time": ParameterValue(use_node_time, value_type=bool),  # Whether to use node time
                }
            ],
        ),
        Node(
            package="rcj_localization",
            executable="fastmap_remap_node",
            name="white_line_hsv_input_remap_node",
            output="screen",
            arguments=["--ros-args", "--log-level", "info"],
            parameters=[
                {
                    "fastmap_file": str(selected_fastmap_file),  # Fastmap XML path
                    "input_topic": input_topic,  # Remap input image topic
                    "output_topic": remap_topic,  # Remap output image topic
                    "input_transport": input_transport,  # Remap input transport
                    "interpolation": interpolation,  # Remap interpolation mode
                    "enable_image_view": LaunchConfiguration("remap_enable_image_view"),  # Whether to show remap windows
                    "enable_timing_log": ParameterValue(
                        LaunchConfiguration("remap_enable_timing_log"),
                        value_type=bool,
                    ),  # Whether to log remap timing
                    "timing_log_interval": ParameterValue(
                        LaunchConfiguration("remap_timing_log_interval"), value_type=int
                    ),  # Remap timing log frame interval
                }
            ],
        ),
        Node(
            package="rcj_localization",
            executable="white_line_hsv_white_node",
            name=hsv_node_name,
            output="screen",
            arguments=["--ros-args", "--log-level", "info"],
            remappings=[("~/white_mask", white_mask_topic)],
            parameters=[
                {
                    "input_topic": remap_topic,  # HSV input image topic
                    "white_h_min": ParameterValue(
                        LaunchConfiguration("white_h_min"), value_type=int
                    ),  # White HSV minimum H
                    "white_h_max": ParameterValue(
                        LaunchConfiguration("white_h_max"), value_type=int
                    ),  # White HSV maximum H
                    "white_s_max": ParameterValue(
                        LaunchConfiguration("white_s_max"), value_type=int
                    ),  # White HSV maximum S
                    "white_v_min": ParameterValue(
                        LaunchConfiguration("white_v_min"), value_type=int
                    ),  # White HSV minimum V
                    "black_v_max": ParameterValue(
                        LaunchConfiguration("black_v_max"), value_type=int
                    ),  # Black HSV maximum V
                    "green_h_min": ParameterValue(
                        LaunchConfiguration("green_h_min"), value_type=int
                    ),  # Green HSV minimum H
                    "green_h_max": ParameterValue(
                        LaunchConfiguration("green_h_max"), value_type=int
                    ),  # Green HSV maximum H
                    "green_s_min": ParameterValue(
                        LaunchConfiguration("green_s_min"), value_type=int
                    ),  # Green HSV minimum S
                    "green_v_min": ParameterValue(
                        LaunchConfiguration("green_v_min"), value_type=int
                    ),  # Green HSV minimum V
                    "enable_timing_log": ParameterValue(
                        LaunchConfiguration("hsv_enable_timing_log"),
                        value_type=bool,
                    ),  # Whether to log HSV timing
                    "timing_log_interval": ParameterValue(
                        LaunchConfiguration("hsv_timing_log_interval"), value_type=int
                    ),  # HSV timing log frame interval
                    "enable_image_view": ParameterValue(
                        LaunchConfiguration("hsv_enable_image_view"), value_type=bool
                    ),  # Whether to show HSV debug windows
                    "show_input_image": ParameterValue(
                        LaunchConfiguration("hsv_show_input_image"), value_type=bool
                    ),  # Whether to show HSV input image
                    "show_white_mask": ParameterValue(
                        LaunchConfiguration("hsv_show_white_mask"), value_type=bool
                    ),  # Whether to show white mask image
                    "show_green_mask": ParameterValue(
                        LaunchConfiguration("hsv_show_green_mask"), value_type=bool
                    ),  # Whether to show green priority mask image
                    "show_black_mask": ParameterValue(
                        LaunchConfiguration("hsv_show_black_mask"), value_type=bool
                    ),  # Whether to show black priority mask image
                    "show_noise_mask": ParameterValue(
                        LaunchConfiguration("hsv_show_noise_mask"), value_type=bool
                    ),  # Whether to show remaining noise mask image
                    "show_overlay_image": ParameterValue(
                        LaunchConfiguration("hsv_show_overlay_image"), value_type=bool
                    ),  # Whether to show overlay image
                    "display_max_width": ParameterValue(
                        LaunchConfiguration("hsv_display_max_width"), value_type=int
                    ),  # HSV window max width
                    "display_max_height": ParameterValue(
                        LaunchConfiguration("hsv_display_max_height"), value_type=int
                    ),  # HSV window max height
                }
            ],
        ),
        Node(
            package="rcj_localization",
            executable="white_line_dt_ridge_filter_node",
            name="white_line_dt_ridge_filter_node",
            output="screen",
            arguments=["--ros-args", "--log-level", "info"],
            parameters=[
                {
                    "morph_mask_topic": white_mask_topic,  # Ridge input white mask topic
                    "green_mask_topic": f"/{hsv_node_name}/green_mask",  # Ridge input green mask topic
                    "black_mask_topic": f"/{hsv_node_name}/black_mask",  # Ridge input black mask topic
                    "noise_mask_topic": f"/{hsv_node_name}/noise_mask",  # Ridge input noise mask topic
                    "orientation_window_radius_px": ParameterValue(
                        LaunchConfiguration("ridge_orientation_window_radius_px"),
                        value_type=int,
                    ),  # Neighborhood radius for orientation estimation
                    "min_orientation_neighbors": ParameterValue(
                        LaunchConfiguration("ridge_min_orientation_neighbors"),
                        value_type=int,
                    ),  # Minimum ridge neighbors for valid orientation
                    "enable_orientation_estimate": ParameterValue(
                        LaunchConfiguration("ridge_enable_orientation_estimate"),
                        value_type=bool,
                    ),  # Whether to run orientation estimation before side support
                    "side_margin_px": ParameterValue(
                        LaunchConfiguration("ridge_side_margin_px"), value_type=int
                    ),  # Offset from centerline before side sampling
                    "side_band_depth_px": ParameterValue(
                        LaunchConfiguration("ridge_side_band_depth_px"), value_type=int
                    ),  # Side sampling band depth
                    "min_green_ratio": ParameterValue(
                        LaunchConfiguration("ridge_min_green_ratio"), value_type=float
                    ),  # Minimum green support ratio
                    "min_boundary_ratio": ParameterValue(
                        LaunchConfiguration("ridge_min_boundary_ratio"), value_type=float
                    ),  # Minimum boundary support ratio
                    "enable_boundary_mode": ParameterValue(
                        LaunchConfiguration("ridge_enable_boundary_mode"),
                        value_type=bool,
                    ),  # Whether to allow green-boundary support
                    "width_floor_px": ParameterValue(
                        LaunchConfiguration("ridge_width_floor_px"), value_type=float
                    ),  # Minimum accepted local width
                    "width_ceil_px": ParameterValue(
                        LaunchConfiguration("ridge_width_ceil_px"), value_type=float
                    ),  # Maximum accepted local width
                    "width_mad_scale": ParameterValue(
                        LaunchConfiguration("ridge_width_mad_scale"), value_type=float
                    ),  # MAD scale for adaptive width range
                    "min_width_samples": ParameterValue(
                        LaunchConfiguration("ridge_min_width_samples"), value_type=int
                    ),  # Minimum samples before adaptive width estimation
                    "enable_candidate_prefilter": ParameterValue(
                        LaunchConfiguration("ridge_enable_candidate_prefilter"),
                        value_type=bool,
                    ),  # Whether to run candidate prefilter before orientation
                    "candidate_min_component_px": ParameterValue(
                        LaunchConfiguration("ridge_candidate_min_component_px"),
                        value_type=int,
                    ),  # Minimum candidate ridge component area before pruning
                    "candidate_prune_rounds": ParameterValue(
                        LaunchConfiguration("ridge_candidate_prune_rounds"),
                        value_type=int,
                    ),  # Endpoint pruning rounds for candidate ridge mask
                    "side_scan_stride": ParameterValue(
                        LaunchConfiguration("ridge_side_scan_stride"), value_type=int
                    ),  # Seed sampling stride before side support scan
                    "side_template_direction_bins": ParameterValue(
                        LaunchConfiguration("ridge_side_template_direction_bins"),
                        value_type=int,
                    ),  # Number of direction bins for side scan templates
                    "enable_parallel_side_scan": ParameterValue(
                        LaunchConfiguration("ridge_enable_parallel_side_scan"),
                        value_type=bool,
                    ),  # Whether to parallelize side support seed scanning
                    "enable_parallel_orientation_estimate": ParameterValue(
                        LaunchConfiguration("ridge_enable_parallel_orientation_estimate"),
                        value_type=bool,
                    ),  # Whether to parallelize seed-only orientation estimation
                    "enable_length_filter": ParameterValue(
                        LaunchConfiguration("ridge_enable_length_filter"),
                        value_type=bool,
                    ),  # Whether to run connected-component length filtering
                    "min_skeleton_length_px": ParameterValue(
                        LaunchConfiguration("ridge_min_skeleton_length_px"),
                        value_type=int,
                    ),  # Minimum ridge component length
                    "reconstruction_margin_px": ParameterValue(
                        LaunchConfiguration("ridge_reconstruction_margin_px"),
                        value_type=float,
                    ),  # Extra radius added during reconstruction
                    "enable_image_view": ParameterValue(
                        LaunchConfiguration("ridge_enable_image_view"), value_type=bool
                    ),  # Whether to show ridge debug windows
                    "show_morph_mask": ParameterValue(
                        LaunchConfiguration("ridge_show_morph_mask"), value_type=bool
                    ),  # Whether to show input white mask
                    "show_distance_transform": ParameterValue(
                        LaunchConfiguration("ridge_show_distance_transform"), value_type=bool
                    ),  # Whether to show the DT image before orientation filtering
                    "show_green_mask": ParameterValue(
                        LaunchConfiguration("ridge_show_green_mask"), value_type=bool
                    ),  # Whether to show input green mask
                    "show_black_mask": ParameterValue(
                        LaunchConfiguration("ridge_show_black_mask"), value_type=bool
                    ),  # Whether to show input black mask
                    "show_noise_mask": ParameterValue(
                        LaunchConfiguration("ridge_show_noise_mask"), value_type=bool
                    ),  # Whether to show input noise mask
                    "show_ridge_mask": ParameterValue(
                        LaunchConfiguration("ridge_show_ridge_mask"), value_type=bool
                    ),  # Whether to show extracted ridge mask
                    "show_candidate_prefilter_mask": ParameterValue(
                        LaunchConfiguration("ridge_show_candidate_prefilter_mask"),
                        value_type=bool,
                    ),  # Whether to show candidate-prefilter ridge mask
                    "show_orientation_valid_mask": ParameterValue(
                        LaunchConfiguration("ridge_show_orientation_valid_mask"),
                        value_type=bool,
                    ),  # Whether to show orientation-valid ridge mask
                    "show_side_support_seed_mask": ParameterValue(
                        LaunchConfiguration("ridge_show_side_support_seed_mask"),
                        value_type=bool,
                    ),  # Whether to show supported side-scan seed mask
                    "show_side_support_mask": ParameterValue(
                        LaunchConfiguration("ridge_show_side_support_mask"),
                        value_type=bool,
                    ),  # Whether to show side-support mask
                    "show_width_supported_ridge_mask": ParameterValue(
                        LaunchConfiguration("ridge_show_width_supported_ridge_mask"),
                        value_type=bool,
                    ),  # Whether to show width-filtered ridge mask
                    "show_length_filtered_ridge_mask": ParameterValue(
                        LaunchConfiguration("ridge_show_length_filtered_ridge_mask"),
                        value_type=bool,
                    ),  # Whether to show length-filtered ridge mask
                    "show_reconstructed_mask": ParameterValue(
                        LaunchConfiguration("ridge_show_reconstructed_mask"),
                        value_type=bool,
                    ),  # Whether to show reconstructed mask
                    "show_white_final_mask": ParameterValue(
                        LaunchConfiguration("ridge_show_white_final_mask"),
                        value_type=bool,
                    ),  # Whether to show final white mask
                    "show_debug_image": ParameterValue(
                        LaunchConfiguration("ridge_show_debug_image"), value_type=bool
                    ),  # Whether to show composite debug image
                    "enable_timing_debug": ParameterValue(
                        LaunchConfiguration("ridge_enable_timing_debug"),
                        value_type=bool,
                    ),  # Whether to log ridge timing summary
                    "timing_summary_interval": ParameterValue(
                        LaunchConfiguration("ridge_timing_summary_interval"),
                        value_type=int,
                    ),  # Ridge timing summary frame interval
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_index", default_value="0"),  # Camera index
            DeclareLaunchArgument("role", default_value="viewfinder"),  # camera_ros role
            DeclareLaunchArgument("format", default_value="RGB888"),  # Camera pixel format
            DeclareLaunchArgument(
                "width",
                default_value="800",
            ),  # Capture width
            DeclareLaunchArgument(
                "height",
                default_value="600",
            ),  # Capture height
            DeclareLaunchArgument("orientation", default_value="0"),  # Camera rotation angle
            DeclareLaunchArgument("sensor_mode", default_value="1332:990"),  # Camera sensor mode
            DeclareLaunchArgument("frame_id", default_value="camera"),  # Image frame id
            DeclareLaunchArgument("camera_info_url", default_value=""),  # Camera calibration URL
            DeclareLaunchArgument("use_node_time", default_value="false"),  # Whether to use node time
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera_info"
            ),  # Camera info topic
            DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),  # Raw image topic
            DeclareLaunchArgument(
                "remap_topic", default_value="/camera/image_remapped"
            ),  # Remapped image topic
            DeclareLaunchArgument(
                "white_mask_topic", default_value="/camera/white_mask"
            ),  # White mask topic
            DeclareLaunchArgument("use_latest_fastmap", default_value="false"),  # Whether to auto-select the latest Fastmap XML
            DeclareLaunchArgument(
                "fastmap_file", default_value=""
            ),  # Specific Fastmap XML path when auto-select is disabled
            DeclareLaunchArgument("input_transport", default_value="raw"),  # Remap input transport
            DeclareLaunchArgument("interpolation", default_value="linear"),  # Remap interpolation mode
            DeclareLaunchArgument(
                "remap_enable_image_view", default_value="false"
            ),  # Whether to show remap windows
            DeclareLaunchArgument(
                "remap_enable_timing_log", default_value="true"
            ),  # Whether to log remap timing
            DeclareLaunchArgument(
                "remap_timing_log_interval", default_value="30"
            ),  # Remap timing log frame interval
            DeclareLaunchArgument("white_h_min", default_value="0"),  # White HSV minimum H
            DeclareLaunchArgument("white_h_max", default_value="179"),  # White HSV maximum H
            DeclareLaunchArgument("white_s_max", default_value="153"),  # White HSV maximum S
            DeclareLaunchArgument("white_v_min", default_value="170"),  # White HSV minimum V
            DeclareLaunchArgument("black_v_max", default_value="140"),  # Black HSV maximum V
            DeclareLaunchArgument("green_h_min", default_value="35"),  # Green HSV minimum H
            DeclareLaunchArgument("green_h_max", default_value="100"),  # Green HSV maximum H
            DeclareLaunchArgument("green_s_min", default_value="150"),  # Green HSV minimum S
            DeclareLaunchArgument("green_v_min", default_value="100"),  # Green HSV minimum V
            DeclareLaunchArgument(
                "hsv_enable_timing_log", default_value="true"
            ),  # Whether to log HSV timing
            DeclareLaunchArgument(
                "hsv_timing_log_interval", default_value="15"
            ),  # HSV timing log frame interval
            DeclareLaunchArgument(
                "hsv_enable_image_view", default_value="false"
            ),  # Master switch for HSV debug windows; false means no window creation or GUI processing
            DeclareLaunchArgument("hsv_show_input_image", default_value="true"),  # Show the HSV input image window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_white_mask", default_value="true"),  # Show the white-priority mask window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_green_mask", default_value="false"),  # Show the green-priority mask window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_black_mask", default_value="false"),  # Show the black-priority mask window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_noise_mask", default_value="false"),  # Show the remaining noise mask window when hsv_enable_image_view is true
            DeclareLaunchArgument(
                "hsv_show_overlay_image", default_value="true"
            ),  # Show the HSV overlay window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_display_max_width", default_value="960"),  # HSV window max width
            DeclareLaunchArgument("hsv_display_max_height", default_value="720"),  # HSV window max height
            DeclareLaunchArgument(
                "ridge_orientation_window_radius_px", default_value="5"
            ),  # Neighborhood radius for orientation estimation
            DeclareLaunchArgument(
                "ridge_min_orientation_neighbors", default_value="6"
            ),  # Minimum ridge neighbors for valid orientation
            DeclareLaunchArgument(
                "ridge_enable_orientation_estimate", default_value="false"
            ),  # Whether to run orientation estimation before side support
            DeclareLaunchArgument("ridge_side_margin_px", default_value="1"),  # Offset from centerline before side sampling
            DeclareLaunchArgument(
                "ridge_side_band_depth_px", default_value="4"
            ),  # Side sampling band depth
            DeclareLaunchArgument("ridge_min_green_ratio", default_value="0.35"),  # Minimum green support ratio
            DeclareLaunchArgument(
                "ridge_min_boundary_ratio", default_value="0.35"
            ),  # Minimum boundary support ratio
            DeclareLaunchArgument(
                "ridge_enable_boundary_mode", default_value="true"
            ),  # Whether to allow green-boundary support
            DeclareLaunchArgument("ridge_width_floor_px", default_value="2.0"),  # Minimum accepted local width
            DeclareLaunchArgument("ridge_width_ceil_px", default_value="40.0"),  # Maximum accepted local width
            DeclareLaunchArgument("ridge_width_mad_scale", default_value="2.5"),  # MAD scale for adaptive width range
            DeclareLaunchArgument("ridge_min_width_samples", default_value="25"),  # Minimum samples before adaptive width estimation
            DeclareLaunchArgument(
                "ridge_enable_candidate_prefilter", default_value="false"
            ),  # Whether to run candidate prefilter before orientation
            DeclareLaunchArgument(
                "ridge_candidate_min_component_px", default_value="3"
            ),  # Minimum candidate ridge component area before pruning
            DeclareLaunchArgument(
                "ridge_candidate_prune_rounds", default_value="1"
            ),  # Endpoint pruning rounds for candidate ridge mask
            DeclareLaunchArgument(
                "ridge_side_scan_stride", default_value="3"
            ),  # Seed sampling stride before side support scan
            DeclareLaunchArgument(
                "ridge_side_template_direction_bins", default_value="16"
            ),  # Number of direction bins for side scan templates
            DeclareLaunchArgument(
                "ridge_enable_parallel_side_scan", default_value="true"
            ),  # Whether to parallelize side support seed scanning
            DeclareLaunchArgument(
                "ridge_enable_parallel_orientation_estimate", default_value="true"
            ),  # Whether to parallelize seed-only orientation estimation
            DeclareLaunchArgument(
                "ridge_enable_length_filter", default_value="false"
            ),  # Whether to run connected-component length filtering
            DeclareLaunchArgument(
                "ridge_min_skeleton_length_px", default_value="12"
            ),  # Minimum ridge component length
            DeclareLaunchArgument(
                "ridge_reconstruction_margin_px", default_value="1.0"
            ),  # Extra radius added during reconstruction
            DeclareLaunchArgument(
                "ridge_enable_image_view", default_value="true"
            ),  # Whether to show ridge debug windows
            DeclareLaunchArgument("ridge_show_morph_mask", default_value="true"),  # Whether to show input white mask
            DeclareLaunchArgument("ridge_show_distance_transform", default_value="true"),  # Whether to show the DT image before orientation filtering
            DeclareLaunchArgument("ridge_show_green_mask", default_value="false"),  # Whether to show input green mask
            DeclareLaunchArgument("ridge_show_black_mask", default_value="false"),  # Whether to show input black mask
            DeclareLaunchArgument("ridge_show_noise_mask", default_value="false"),  # Whether to show input noise mask
            DeclareLaunchArgument("ridge_show_ridge_mask", default_value="false"),  # Whether to show extracted ridge mask
            DeclareLaunchArgument(
                "ridge_show_candidate_prefilter_mask", default_value="false"
            ),  # Whether to show candidate-prefilter ridge mask
            DeclareLaunchArgument(
                "ridge_show_orientation_valid_mask", default_value="true"
            ),  # Whether to show orientation-valid ridge mask
            DeclareLaunchArgument(
                "ridge_show_side_support_seed_mask", default_value="false"
            ),  # Whether to show supported side-scan seed mask
            DeclareLaunchArgument(
                "ridge_show_side_support_mask", default_value="true"
            ),  # Whether to show side-support mask
            DeclareLaunchArgument(
                "ridge_show_width_supported_ridge_mask", default_value="true"
            ),  # Whether to show width-filtered ridge mask
            DeclareLaunchArgument(
                "ridge_show_length_filtered_ridge_mask", default_value="true"
            ),  # Whether to show length-filtered ridge mask
            DeclareLaunchArgument(
                "ridge_show_reconstructed_mask", default_value="true"
            ),  # Whether to show reconstructed mask
            DeclareLaunchArgument(
                "ridge_show_white_final_mask", default_value="true"
            ),  # Whether to show final white mask
            DeclareLaunchArgument("ridge_show_debug_image", default_value="false"),  # Whether to show composite debug image
            DeclareLaunchArgument(
                "ridge_enable_timing_debug", default_value="true"
            ),  # Whether to log ridge timing summary
            DeclareLaunchArgument(
                "ridge_timing_summary_interval", default_value="10"
            ),  # Ridge timing summary frame interval
            OpaqueFunction(function=build_nodes),
        ]
    )
