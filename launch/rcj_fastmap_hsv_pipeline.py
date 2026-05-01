from pathlib import Path
import xml.etree.ElementTree as ET

from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from rcj_shared_launch_params import (
    camera_ros_parameters,
    hsv_green_white_black_parameters,
)


def optional_launch_config(name, default_value):
    return LaunchConfiguration(name, default=default_value)


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
    fastmap_file_value = LaunchConfiguration("fastmap_file").perform(context).strip()

    if not fastmap_file_value:
        raise RuntimeError("Launch argument 'fastmap_file' must be set.")

    fastmap_path = Path(fastmap_file_value).expanduser()
    if not fastmap_path.is_absolute():
        fastmap_path = fastmap_path.resolve()
    if not fastmap_path.exists():
        raise FileNotFoundError(f"Fastmap XML file does not exist: {fastmap_path}")
    return fastmap_path


def resolve_camera_size(context, fastmap_file):
    default_width, default_height = read_fastmap_source_size(fastmap_file)
    width_value = int(
        LaunchConfiguration("width").perform(context).strip() or str(default_width)
    )
    height_value = int(
        LaunchConfiguration("height").perform(context).strip() or str(default_height)
    )
    return width_value, height_value


def build_camera_hsv_fastmap_nodes(context, *, use_apply_mask_argument=False):
    selected_fastmap_file = resolve_fastmap_file(context)
    width_value, height_value = resolve_camera_size(context, selected_fastmap_file)

    input_topic = LaunchConfiguration("input_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    remap_topic = LaunchConfiguration("remap_topic")
    white_mask_topic = LaunchConfiguration("white_mask_topic")
    robot_mask_path = LaunchConfiguration("robot_mask_path")
    input_transport = LaunchConfiguration("input_transport")
    interpolation = LaunchConfiguration("interpolation")

    apply_mask = True
    if use_apply_mask_argument:
        apply_mask = (
            LaunchConfiguration("apply_mask").perform(context).strip().lower()
            == "true"
        )
    remap_robot_mask_path = robot_mask_path if apply_mask else ""
    hsv_robot_mask_topic = (
        "/white_line_hsv_input_remap_node/robot_mask" if apply_mask else ""
    )

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
            parameters=[camera_ros_parameters(width_value, height_value)],
        ),
        Node(
            package="rcj_localization",
            executable="fastmap_remap_node",
            name="white_line_hsv_input_remap_node",
            output="screen",
            arguments=["--ros-args", "--log-level", "info"],
            parameters=[
                {
                    "fastmap_file": str(selected_fastmap_file),
                    "robot_mask_path": remap_robot_mask_path,
                    "input_topic": input_topic,
                    "output_topic": remap_topic,
                    "input_transport": input_transport,
                    "interpolation": interpolation,
                    "enable_image_view": LaunchConfiguration(
                        "remap_enable_image_view"
                    ),
                    "show_input_image": ParameterValue(
                        optional_launch_config("remap_show_input_image", "true"),
                        value_type=bool,
                    ),
                    "show_output_image": ParameterValue(
                        optional_launch_config("remap_show_output_image", "true"),
                        value_type=bool,
                    ),
                    "publish_debug_images": ParameterValue(
                        optional_launch_config("remap_publish_debug_images", "false"),
                        value_type=bool,
                    ),
                    "publish_input_image": ParameterValue(
                        optional_launch_config("remap_publish_input_image", "true"),
                        value_type=bool,
                    ),
                    "publish_output_image": ParameterValue(
                        optional_launch_config("remap_publish_output_image", "true"),
                        value_type=bool,
                    ),
                    "debug_jpeg_quality": ParameterValue(
                        optional_launch_config("debug_jpeg_quality", "80"),
                        value_type=int,
                    ),
                    "debug_image_max_fps": ParameterValue(
                        optional_launch_config("debug_image_max_fps", "5.0"),
                        value_type=float,
                    ),
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
            executable="white_line_hsv_white_node",
            name="white_line_hsv_white_node",
            output="screen",
            arguments=["--ros-args", "--log-level", "info"],
            remappings=[("~/white_mask", white_mask_topic)],
            parameters=[
                {
                    "input_topic": remap_topic,
                    "robot_mask_topic": hsv_robot_mask_topic,
                    **hsv_green_white_black_parameters(),
                    "enable_timing_log": ParameterValue(
                        LaunchConfiguration("hsv_enable_timing_log"),
                        value_type=bool,
                    ),
                    "timing_log_interval": ParameterValue(
                        LaunchConfiguration("hsv_timing_log_interval"),
                        value_type=int,
                    ),
                    "enable_image_view": ParameterValue(
                        LaunchConfiguration("hsv_enable_image_view"),
                        value_type=bool,
                    ),
                    "enable_controls_window": ParameterValue(
                        LaunchConfiguration("hsv_enable_controls_window"),
                        value_type=bool,
                    ),
                    "show_input_image": ParameterValue(
                        LaunchConfiguration("hsv_show_input_image"),
                        value_type=bool,
                    ),
                    "show_white_mask": ParameterValue(
                        LaunchConfiguration("hsv_show_white_mask"),
                        value_type=bool,
                    ),
                    "show_green_mask": ParameterValue(
                        LaunchConfiguration("hsv_show_green_mask"),
                        value_type=bool,
                    ),
                    "show_black_mask": ParameterValue(
                        LaunchConfiguration("hsv_show_black_mask"),
                        value_type=bool,
                    ),
                    "show_noise_mask": ParameterValue(
                        LaunchConfiguration("hsv_show_noise_mask"),
                        value_type=bool,
                    ),
                    "show_overlay_image": ParameterValue(
                        LaunchConfiguration("hsv_show_overlay_image"),
                        value_type=bool,
                    ),
                    "publish_debug_images": ParameterValue(
                        optional_launch_config("hsv_publish_debug_images", "false"),
                        value_type=bool,
                    ),
                    "publish_input_image": ParameterValue(
                        optional_launch_config("hsv_publish_input_image", "true"),
                        value_type=bool,
                    ),
                    "publish_white_mask": ParameterValue(
                        optional_launch_config("hsv_publish_white_mask", "true"),
                        value_type=bool,
                    ),
                    "publish_green_mask": ParameterValue(
                        optional_launch_config("hsv_publish_green_mask", "true"),
                        value_type=bool,
                    ),
                    "publish_black_mask": ParameterValue(
                        optional_launch_config("hsv_publish_black_mask", "true"),
                        value_type=bool,
                    ),
                    "publish_noise_mask": ParameterValue(
                        optional_launch_config("hsv_publish_noise_mask", "true"),
                        value_type=bool,
                    ),
                    "publish_overlay_image": ParameterValue(
                        optional_launch_config("hsv_publish_overlay_image", "true"),
                        value_type=bool,
                    ),
                    "debug_jpeg_quality": ParameterValue(
                        optional_launch_config("debug_jpeg_quality", "80"),
                        value_type=int,
                    ),
                    "debug_image_max_fps": ParameterValue(
                        optional_launch_config("debug_image_max_fps", "5.0"),
                        value_type=float,
                    ),
                    "display_max_width": ParameterValue(
                        LaunchConfiguration("hsv_display_max_width"),
                        value_type=int,
                    ),
                    "display_max_height": ParameterValue(
                        LaunchConfiguration("hsv_display_max_height"),
                        value_type=int,
                    ),
                }
            ],
        ),
    ]


def build_dt_ridge_node():
    hsv_node_name = "white_line_hsv_white_node"
    return Node(
        package="rcj_localization",
        executable="white_line_dt_ridge_filter_node",
        name="white_line_dt_ridge_filter_node",
        output="screen",
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[
            {
                "morph_mask_topic": LaunchConfiguration("white_mask_topic"),
                "green_mask_topic": f"/{hsv_node_name}/green_mask",
                "black_mask_topic": f"/{hsv_node_name}/black_mask",
                "noise_mask_topic": f"/{hsv_node_name}/noise_mask",
                "orientation_window_radius_px": ParameterValue(
                    LaunchConfiguration("ridge_orientation_window_radius_px"),
                    value_type=int,
                ),
                "min_orientation_neighbors": ParameterValue(
                    LaunchConfiguration("ridge_min_orientation_neighbors"),
                    value_type=int,
                ),
                "enable_orientation_estimate": ParameterValue(
                    LaunchConfiguration("ridge_enable_orientation_estimate"),
                    value_type=bool,
                ),
                "side_margin_px": ParameterValue(
                    LaunchConfiguration("ridge_side_margin_px"), value_type=int
                ),
                "side_band_depth_px": ParameterValue(
                    LaunchConfiguration("ridge_side_band_depth_px"), value_type=int
                ),
                "min_green_ratio": ParameterValue(
                    LaunchConfiguration("ridge_min_green_ratio"), value_type=float
                ),
                "min_boundary_ratio": ParameterValue(
                    LaunchConfiguration("ridge_min_boundary_ratio"), value_type=float
                ),
                "enable_boundary_mode": ParameterValue(
                    LaunchConfiguration("ridge_enable_boundary_mode"),
                    value_type=bool,
                ),
                "width_floor_px": ParameterValue(
                    LaunchConfiguration("ridge_width_floor_px"), value_type=float
                ),
                "width_ceil_px": ParameterValue(
                    LaunchConfiguration("ridge_width_ceil_px"), value_type=float
                ),
                "width_mad_scale": ParameterValue(
                    LaunchConfiguration("ridge_width_mad_scale"), value_type=float
                ),
                "min_width_samples": ParameterValue(
                    LaunchConfiguration("ridge_min_width_samples"), value_type=int
                ),
                "enable_candidate_prefilter": ParameterValue(
                    LaunchConfiguration("ridge_enable_candidate_prefilter"),
                    value_type=bool,
                ),
                "candidate_min_component_px": ParameterValue(
                    LaunchConfiguration("ridge_candidate_min_component_px"),
                    value_type=int,
                ),
                "candidate_prune_rounds": ParameterValue(
                    LaunchConfiguration("ridge_candidate_prune_rounds"),
                    value_type=int,
                ),
                "side_scan_stride": ParameterValue(
                    LaunchConfiguration("ridge_side_scan_stride"), value_type=int
                ),
                "side_template_direction_bins": ParameterValue(
                    LaunchConfiguration("ridge_side_template_direction_bins"),
                    value_type=int,
                ),
                "enable_parallel_side_scan": ParameterValue(
                    LaunchConfiguration("ridge_enable_parallel_side_scan"),
                    value_type=bool,
                ),
                "enable_parallel_orientation_estimate": ParameterValue(
                    LaunchConfiguration("ridge_enable_parallel_orientation_estimate"),
                    value_type=bool,
                ),
                "enable_length_filter": ParameterValue(
                    LaunchConfiguration("ridge_enable_length_filter"),
                    value_type=bool,
                ),
                "min_skeleton_length_px": ParameterValue(
                    LaunchConfiguration("ridge_min_skeleton_length_px"),
                    value_type=int,
                ),
                "reconstruction_margin_px": ParameterValue(
                    LaunchConfiguration("ridge_reconstruction_margin_px"),
                    value_type=float,
                ),
                "enable_image_view": ParameterValue(
                    LaunchConfiguration("ridge_enable_image_view"), value_type=bool
                ),
                "show_morph_mask": ParameterValue(
                    LaunchConfiguration("ridge_show_morph_mask"), value_type=bool
                ),
                "show_distance_transform": ParameterValue(
                    LaunchConfiguration("ridge_show_distance_transform"),
                    value_type=bool,
                ),
                "show_green_mask": ParameterValue(
                    LaunchConfiguration("ridge_show_green_mask"), value_type=bool
                ),
                "show_black_mask": ParameterValue(
                    LaunchConfiguration("ridge_show_black_mask"), value_type=bool
                ),
                "show_noise_mask": ParameterValue(
                    LaunchConfiguration("ridge_show_noise_mask"), value_type=bool
                ),
                "show_ridge_mask": ParameterValue(
                    LaunchConfiguration("ridge_show_ridge_mask"), value_type=bool
                ),
                "show_candidate_prefilter_mask": ParameterValue(
                    LaunchConfiguration("ridge_show_candidate_prefilter_mask"),
                    value_type=bool,
                ),
                "show_orientation_valid_mask": ParameterValue(
                    LaunchConfiguration("ridge_show_orientation_valid_mask"),
                    value_type=bool,
                ),
                "show_side_support_seed_mask": ParameterValue(
                    optional_launch_config("ridge_show_side_support_seed_mask", "false"),
                    value_type=bool,
                ),
                "show_side_support_mask": ParameterValue(
                    LaunchConfiguration("ridge_show_side_support_mask"),
                    value_type=bool,
                ),
                "show_width_supported_ridge_mask": ParameterValue(
                    LaunchConfiguration("ridge_show_width_supported_ridge_mask"),
                    value_type=bool,
                ),
                "show_length_filtered_ridge_mask": ParameterValue(
                    LaunchConfiguration("ridge_show_length_filtered_ridge_mask"),
                    value_type=bool,
                ),
                "show_reconstructed_mask": ParameterValue(
                    LaunchConfiguration("ridge_show_reconstructed_mask"),
                    value_type=bool,
                ),
                "show_white_final_mask": ParameterValue(
                    LaunchConfiguration("ridge_show_white_final_mask"),
                    value_type=bool,
                ),
                "show_debug_image": ParameterValue(
                    LaunchConfiguration("ridge_show_debug_image"), value_type=bool
                ),
                "publish_debug_images": ParameterValue(
                    optional_launch_config("ridge_publish_debug_images", "false"),
                    value_type=bool,
                ),
                "publish_morph_mask": ParameterValue(
                    optional_launch_config("ridge_publish_morph_mask", "true"), value_type=bool
                ),
                "publish_distance_transform": ParameterValue(
                    optional_launch_config("ridge_publish_distance_transform", "true"),
                    value_type=bool,
                ),
                "publish_green_mask": ParameterValue(
                    optional_launch_config("ridge_publish_green_mask", "true"), value_type=bool
                ),
                "publish_black_mask": ParameterValue(
                    optional_launch_config("ridge_publish_black_mask", "true"), value_type=bool
                ),
                "publish_noise_mask": ParameterValue(
                    optional_launch_config("ridge_publish_noise_mask", "true"), value_type=bool
                ),
                "publish_ridge_mask": ParameterValue(
                    optional_launch_config("ridge_publish_ridge_mask", "true"), value_type=bool
                ),
                "publish_candidate_prefilter_mask": ParameterValue(
                    optional_launch_config("ridge_publish_candidate_prefilter_mask", "true"),
                    value_type=bool,
                ),
                "publish_orientation_valid_mask": ParameterValue(
                    optional_launch_config("ridge_publish_orientation_valid_mask", "true"),
                    value_type=bool,
                ),
                "publish_side_support_seed_mask": ParameterValue(
                    optional_launch_config("ridge_publish_side_support_seed_mask", "true"),
                    value_type=bool,
                ),
                "publish_side_support_mask": ParameterValue(
                    optional_launch_config("ridge_publish_side_support_mask", "true"),
                    value_type=bool,
                ),
                "publish_width_supported_ridge_mask": ParameterValue(
                    optional_launch_config("ridge_publish_width_supported_ridge_mask", "true"),
                    value_type=bool,
                ),
                "publish_length_filtered_ridge_mask": ParameterValue(
                    optional_launch_config("ridge_publish_length_filtered_ridge_mask", "true"),
                    value_type=bool,
                ),
                "publish_reconstructed_mask": ParameterValue(
                    optional_launch_config("ridge_publish_reconstructed_mask", "true"),
                    value_type=bool,
                ),
                "publish_white_final_mask": ParameterValue(
                    optional_launch_config("ridge_publish_white_final_mask", "true"),
                    value_type=bool,
                ),
                "publish_debug_image": ParameterValue(
                    optional_launch_config("ridge_publish_debug_image", "true"), value_type=bool
                ),
                "debug_jpeg_quality": ParameterValue(
                    optional_launch_config("debug_jpeg_quality", "80"),
                    value_type=int,
                ),
                "debug_image_max_fps": ParameterValue(
                    optional_launch_config("debug_image_max_fps", "5.0"),
                    value_type=float,
                ),
                "enable_timing_debug": ParameterValue(
                    LaunchConfiguration("ridge_enable_timing_log"),
                    value_type=bool,
                ),
                "timing_summary_interval": ParameterValue(
                    LaunchConfiguration("ridge_timing_summary_interval"),
                    value_type=int,
                ),
            }
        ],
    )


def build_camera_hsv_dt_ridge_fastmap_nodes(context):
    return [
        *build_camera_hsv_fastmap_nodes(context, use_apply_mask_argument=True),
        build_dt_ridge_node(),
    ]
