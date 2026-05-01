from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcj_fastmap_hsv_pipeline import build_camera_hsv_dt_ridge_fastmap_nodes
from rcj_shared_launch_params import (
    declare_remap_interpolation_argument,
    declare_camera_ros_arguments,
    declare_hsv_green_white_black_arguments,
)


def generate_launch_description():
    return LaunchDescription(
        [
            *declare_camera_ros_arguments(),
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera_info"
            ),  # Camera info topic
            DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),  # Raw image topic
            DeclareLaunchArgument(
                "remap_topic",
                default_value="/white_line_hsv_input_remap_node/image_remapped",
            ),  # Remapped image topic
            DeclareLaunchArgument(
                "white_mask_topic",
                default_value="/white_line_hsv_white_node/white_mask",
            ),  # White mask topic
            DeclareLaunchArgument(
                "apply_mask", default_value="true"
            ),  # Whether to enable remap-stage robot masking
            DeclareLaunchArgument(
                "robot_mask_path", default_value=str(Path(get_package_share_directory("rcj_localization")) / "config" / "remapped_mask.png")
            ),  # Optional remapped-space robot mask image path
            DeclareLaunchArgument("input_transport", default_value="raw"),  # Remap input transport
            declare_remap_interpolation_argument(),
            DeclareLaunchArgument(
                "remap_enable_image_view", default_value="false"
            ),  # Whether to show remap windows
            DeclareLaunchArgument("remap_show_input_image", default_value="true"),  # Show remap input window when remap image_view is true
            DeclareLaunchArgument("remap_show_output_image", default_value="true"),  # Show remap output window when remap image_view is true
            DeclareLaunchArgument(
                "remap_publish_debug_images", default_value="true"
            ),  # Master switch for remap debug image topics
            DeclareLaunchArgument("remap_publish_input_image", default_value="true"),  # Publish remap input debug topic when subscribed
            DeclareLaunchArgument("remap_publish_output_image", default_value="true"),  # Publish remap output debug topic when subscribed
            DeclareLaunchArgument(
                "remap_enable_timing_log", default_value="true"
            ),  # Whether to log remap timing
            DeclareLaunchArgument(
                "remap_timing_log_interval", default_value="30"
            ),  # Remap timing log frame interval
            *declare_hsv_green_white_black_arguments(),
            DeclareLaunchArgument(
                "hsv_enable_timing_log", default_value="true"
            ),  # Whether to log HSV timing
            DeclareLaunchArgument(
                "hsv_timing_log_interval", default_value="15"
            ),  # HSV timing log frame interval
            DeclareLaunchArgument(
                "hsv_enable_image_view", default_value="true"
            ),  # Master switch for HSV debug windows; false means no window creation or GUI processing
            DeclareLaunchArgument(
                "hsv_enable_controls_window", default_value="false"
            ),  # Whether to show HSV slider controls window
            DeclareLaunchArgument("hsv_show_input_image", default_value="true"),  # Show the HSV input image window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_white_mask", default_value="true"),  # Show the white-priority mask window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_green_mask", default_value="false"),  # Show the green-priority mask window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_black_mask", default_value="false"),  # Show the black-priority mask window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_noise_mask", default_value="false"),  # Show the remaining noise mask window when hsv_enable_image_view is true
            DeclareLaunchArgument(
                "hsv_show_overlay_image", default_value="true"
            ),  # Show the HSV overlay window when hsv_enable_image_view is true
            DeclareLaunchArgument(
                "hsv_publish_debug_images", default_value="false"
            ),  # Master switch for HSV debug image topics
            DeclareLaunchArgument("hsv_publish_input_image", default_value="true"),  # Publish HSV input debug topic when subscribed
            DeclareLaunchArgument("hsv_publish_white_mask", default_value="true"),  # Publish HSV white debug mask when subscribed
            DeclareLaunchArgument("hsv_publish_green_mask", default_value="true"),  # Publish HSV green debug mask when subscribed
            DeclareLaunchArgument("hsv_publish_black_mask", default_value="true"),  # Publish HSV black debug mask when subscribed
            DeclareLaunchArgument("hsv_publish_noise_mask", default_value="true"),  # Publish HSV noise debug mask when subscribed
            DeclareLaunchArgument("hsv_publish_overlay_image", default_value="true"),  # Publish HSV overlay debug topic when subscribed
            DeclareLaunchArgument("hsv_display_max_width", default_value="960"),  # HSV window max width
            DeclareLaunchArgument("hsv_display_max_height", default_value="720"),  # HSV window max height
            DeclareLaunchArgument(
                "ridge_orientation_window_radius_px", default_value="5"
            ),  # Neighborhood radius for orientation estimation
            DeclareLaunchArgument(
                "ridge_min_orientation_neighbors", default_value="6"
            ),  # Minimum ridge neighbors for valid orientation
            DeclareLaunchArgument(
                "ridge_enable_orientation_estimate", default_value="true"
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
                "ridge_enable_boundary_mode", default_value="false"
            ),  # Whether to allow green-boundary support
            DeclareLaunchArgument("ridge_width_floor_px", default_value="5.0"),  # Minimum accepted local width
            DeclareLaunchArgument("ridge_width_ceil_px", default_value="18.0"),  # Maximum accepted local width
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
                "ridge_enable_image_view", default_value="false"
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
            DeclareLaunchArgument("ridge_show_debug_image", default_value="true"),  # Whether to show composite debug image
            DeclareLaunchArgument(
                "ridge_publish_debug_images", default_value="true"
            ),  # Master switch for ridge debug image topics
            DeclareLaunchArgument("ridge_publish_morph_mask", default_value="true"),  # Publish ridge input morph mask when subscribed
            DeclareLaunchArgument("ridge_publish_distance_transform", default_value="true"),  # Publish ridge distance-transform debug image when subscribed
            DeclareLaunchArgument("ridge_publish_green_mask", default_value="true"),  # Publish ridge input green mask when subscribed
            DeclareLaunchArgument("ridge_publish_black_mask", default_value="true"),  # Publish ridge input black mask when subscribed
            DeclareLaunchArgument("ridge_publish_noise_mask", default_value="true"),  # Publish ridge input noise mask when subscribed
            DeclareLaunchArgument("ridge_publish_ridge_mask", default_value="true"),  # Publish extracted ridge mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_candidate_prefilter_mask", default_value="true"
            ),  # Publish candidate-prefilter ridge mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_orientation_valid_mask", default_value="true"
            ),  # Publish orientation-valid seed mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_side_support_seed_mask", default_value="true"
            ),  # Publish supported side-scan seed mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_side_support_mask", default_value="true"
            ),  # Publish side-support mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_width_supported_ridge_mask", default_value="true"
            ),  # Publish width-filtered ridge mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_length_filtered_ridge_mask", default_value="true"
            ),  # Publish length-filtered ridge mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_reconstructed_mask", default_value="true"
            ),  # Publish reconstructed mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_white_final_mask", default_value="true"
            ),  # Keep final white mask topic available when subscribed
            DeclareLaunchArgument("ridge_publish_debug_image", default_value="true"),  # Publish composite debug image when subscribed
            DeclareLaunchArgument(
                "ridge_enable_timing_log", default_value="true"
            ),  # Whether to log ridge timing summary
            DeclareLaunchArgument(
                "ridge_timing_summary_interval", default_value="10"
            ),  # Ridge timing summary frame interval
            OpaqueFunction(function=build_camera_hsv_dt_ridge_fastmap_nodes),
        ]
    )
