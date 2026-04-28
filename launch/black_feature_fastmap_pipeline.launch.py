from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcj_shared_launch_params import (
    declare_remap_interpolation_argument,
    camera_control_launch_arguments,
    camera_control_parameters,
    declare_camera_control_arguments,
    declare_hsv_green_white_black_arguments,
    hsv_green_white_black_launch_arguments,
    hsv_green_white_black_parameters,
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

    input_topic = LaunchConfiguration("input_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    output_topic = LaunchConfiguration("output_topic")
    hsv_node_name = "black_feature_hsv_node"

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
            parameters=[
                {
                    "fastmap_file": str(selected_fastmap_file),
                    "robot_mask_path": LaunchConfiguration("robot_mask_path"),
                    "input_topic": input_topic,
                    "output_topic": output_topic,
                    "input_transport": LaunchConfiguration("input_transport"),
                    "interpolation": LaunchConfiguration("interpolation"),
                    "enable_image_view": ParameterValue(
                        LaunchConfiguration("remap_enable_image_view"), value_type=bool
                    ),
                    "show_input_image": ParameterValue(
                        LaunchConfiguration("remap_show_input_image"), value_type=bool
                    ),
                    "show_output_image": ParameterValue(
                        LaunchConfiguration("remap_show_output_image"), value_type=bool
                    ),
                    "display_max_width": ParameterValue(
                        LaunchConfiguration("remap_display_max_width"), value_type=int
                    ),
                    "display_max_height": ParameterValue(
                        LaunchConfiguration("remap_display_max_height"), value_type=int
                    ),
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
            executable="white_line_hsv_white_node",
            name=hsv_node_name,
            output="screen",
            parameters=[
                {
                    "input_topic": output_topic,
                    "robot_mask_topic": "/black_feature_input_remap_node/robot_mask",
                    "white_h_min": ParameterValue(
                        LaunchConfiguration("white_h_min"), value_type=int
                    ),
                    "white_h_max": ParameterValue(
                        LaunchConfiguration("white_h_max"), value_type=int
                    ),
                    "white_s_max": ParameterValue(
                        LaunchConfiguration("white_s_max"), value_type=int
                    ),
                    "white_v_min": ParameterValue(
                        LaunchConfiguration("white_v_min"), value_type=int
                    ),
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
                    "green_h_min": ParameterValue(
                        LaunchConfiguration("green_h_min"), value_type=int
                    ),
                    "green_h_max": ParameterValue(
                        LaunchConfiguration("green_h_max"), value_type=int
                    ),
                    "green_s_min": ParameterValue(
                        LaunchConfiguration("green_s_min"), value_type=int
                    ),
                    "green_s_max": ParameterValue(
                        LaunchConfiguration("green_s_max"), value_type=int
                    ),
                    "green_v_min": ParameterValue(
                        LaunchConfiguration("green_v_min"), value_type=int
                    ),
                    "green_v_max": ParameterValue(
                        LaunchConfiguration("green_v_max"), value_type=int
                    ),
                    "enable_timing_log": ParameterValue(
                        LaunchConfiguration("hsv_enable_timing_log"), value_type=bool
                    ),
                    "timing_log_interval": ParameterValue(
                        LaunchConfiguration("hsv_timing_log_interval"), value_type=int
                    ),
                    "enable_image_view": ParameterValue(
                        LaunchConfiguration("hsv_enable_image_view"), value_type=bool
                    ),
                    "enable_controls_window": ParameterValue(
                        LaunchConfiguration("hsv_enable_controls_window"), value_type=bool
                    ),
                    "show_input_image": ParameterValue(
                        LaunchConfiguration("hsv_show_input_image"), value_type=bool
                    ),
                    "show_white_mask": ParameterValue(
                        LaunchConfiguration("hsv_show_white_mask"), value_type=bool
                    ),
                    "show_green_mask": ParameterValue(
                        LaunchConfiguration("hsv_show_green_mask"), value_type=bool
                    ),
                    "show_black_mask": ParameterValue(
                        LaunchConfiguration("hsv_show_black_mask"), value_type=bool
                    ),
                    "show_noise_mask": ParameterValue(
                        LaunchConfiguration("hsv_show_noise_mask"), value_type=bool
                    ),
                    "show_overlay_image": ParameterValue(
                        LaunchConfiguration("hsv_show_overlay_image"), value_type=bool
                    ),
                    "show_green_overlay": ParameterValue(
                        LaunchConfiguration("hsv_show_green_overlay"), value_type=bool
                    ),
                    "show_black_overlay": ParameterValue(
                        LaunchConfiguration("hsv_show_black_overlay"), value_type=bool
                    ),
                    "display_max_width": ParameterValue(
                        LaunchConfiguration("hsv_display_max_width"), value_type=int
                    ),
                    "display_max_height": ParameterValue(
                        LaunchConfiguration("hsv_display_max_height"), value_type=int
                    ),
                }
            ],
        ),
        Node(
            package="rcj_localization",
            executable="black_feature_detector_node",
            name="black_feature_detector_node",
            output="screen",
            parameters=[
                {
                    "input_topic": output_topic,
                    "black_mask_topic": f"/{hsv_node_name}/black_mask",
                    "enable_image_view": ParameterValue(
                        LaunchConfiguration("detector_enable_image_view"), value_type=bool
                    ),
                    "publish_debug_image": ParameterValue(
                        LaunchConfiguration("detector_publish_debug_image"),
                        value_type=bool,
                    ),
                    "show_input_image": ParameterValue(
                        LaunchConfiguration("detector_show_input_image"), value_type=bool
                    ),
                    "show_input_black_mask": ParameterValue(
                        LaunchConfiguration("detector_show_input_black_mask"),
                        value_type=bool,
                    ),
                    "show_binary_normalized_mask": ParameterValue(
                        LaunchConfiguration("detector_show_binary_normalized_mask"),
                        value_type=bool,
                    ),
                    "show_morph_open_mask": ParameterValue(
                        LaunchConfiguration("detector_show_morph_open_mask"),
                        value_type=bool,
                    ),
                    "show_morph_close_mask": ParameterValue(
                        LaunchConfiguration("detector_show_morph_close_mask"),
                        value_type=bool,
                    ),
                    "show_border_filtered_mask": ParameterValue(
                        LaunchConfiguration("detector_show_border_filtered_mask"),
                        value_type=bool,
                    ),
                    "show_clean_mask": ParameterValue(
                        LaunchConfiguration("detector_show_clean_mask"), value_type=bool
                    ),
                    "show_dot_mask": ParameterValue(
                        LaunchConfiguration("detector_show_dot_mask"), value_type=bool
                    ),
                    "show_circle_mask": ParameterValue(
                        LaunchConfiguration("detector_show_circle_mask"), value_type=bool
                    ),
                    "show_circle_candidate_mask": ParameterValue(
                        LaunchConfiguration("detector_show_circle_candidate_mask"),
                        value_type=bool,
                    ),
                    "show_circle_rejected_component_mask": ParameterValue(
                        LaunchConfiguration(
                            "detector_show_circle_rejected_component_mask"
                        ),
                        value_type=bool,
                    ),
                    "show_circle_rejected_arc_mask": ParameterValue(
                        LaunchConfiguration("detector_show_circle_rejected_arc_mask"),
                        value_type=bool,
                    ),
                    "show_circle_radius_rejected_mask": ParameterValue(
                        LaunchConfiguration("detector_show_circle_radius_rejected_mask"),
                        value_type=bool,
                    ),
                    "show_circle_too_few_pixels_rejected_mask": ParameterValue(
                        LaunchConfiguration(
                            "detector_show_circle_too_few_pixels_rejected_mask"
                        ),
                        value_type=bool,
                    ),
                    "show_circle_radial_fit_rejected_mask": ParameterValue(
                        LaunchConfiguration(
                            "detector_show_circle_radial_fit_rejected_mask"
                        ),
                        value_type=bool,
                    ),
                    "show_circle_no_segments_rejected_mask": ParameterValue(
                        LaunchConfiguration(
                            "detector_show_circle_no_segments_rejected_mask"
                        ),
                        value_type=bool,
                    ),
                    "show_black_final_mask": ParameterValue(
                        LaunchConfiguration("detector_show_black_final_mask"),
                        value_type=bool,
                    ),
                    "show_debug_image": ParameterValue(
                        LaunchConfiguration("detector_show_debug_image"), value_type=bool
                    ),
                    "display_max_width": ParameterValue(
                        LaunchConfiguration("detector_display_max_width"), value_type=int
                    ),
                    "display_max_height": ParameterValue(
                        LaunchConfiguration("detector_display_max_height"), value_type=int
                    ),
                    "open_kernel": ParameterValue(
                        LaunchConfiguration("detector_open_kernel"), value_type=int
                    ),
                    "close_kernel": ParameterValue(
                        LaunchConfiguration("detector_close_kernel"), value_type=int
                    ),
                    "dot_min_area_px": ParameterValue(
                        LaunchConfiguration("detector_dot_min_area_px"), value_type=int
                    ),
                    "dot_max_area_px": ParameterValue(
                        LaunchConfiguration("detector_dot_max_area_px"), value_type=int
                    ),
                    "dot_min_circularity": ParameterValue(
                        LaunchConfiguration("detector_dot_min_circularity"),
                        value_type=float,
                    ),
                    "dot_min_solidity": ParameterValue(
                        LaunchConfiguration("detector_dot_min_solidity"), value_type=float
                    ),
                    "circle_min_radius_px": ParameterValue(
                        LaunchConfiguration("detector_circle_min_radius_px"),
                        value_type=float,
                    ),
                    "circle_max_radius_px": ParameterValue(
                        LaunchConfiguration("detector_circle_max_radius_px"),
                        value_type=float,
                    ),
                    "circle_ring_width_px": ParameterValue(
                        LaunchConfiguration("detector_circle_ring_width_px"),
                        value_type=float,
                    ),
                    "circle_inner_guard_px": ParameterValue(
                        LaunchConfiguration("detector_circle_inner_guard_px"),
                        value_type=float,
                    ),
                    "circle_outer_guard_px": ParameterValue(
                        LaunchConfiguration("detector_circle_outer_guard_px"),
                        value_type=float,
                    ),
                    "circle_min_ring_black_ratio": ParameterValue(
                        LaunchConfiguration("detector_circle_min_ring_black_ratio"),
                        value_type=float,
                    ),
                    "circle_max_inner_black_ratio": ParameterValue(
                        LaunchConfiguration("detector_circle_max_inner_black_ratio"),
                        value_type=float,
                    ),
                    "circle_max_outer_black_ratio": ParameterValue(
                        LaunchConfiguration("detector_circle_max_outer_black_ratio"),
                        value_type=float,
                    ),
                    "circle_max_radial_residual_px": ParameterValue(
                        LaunchConfiguration("detector_circle_max_radial_residual_px"),
                        value_type=float,
                    ),
                    "circle_min_arc_span_deg": ParameterValue(
                        LaunchConfiguration("detector_circle_min_arc_span_deg"),
                        value_type=float,
                    ),
                    "circle_angle_bin_deg": ParameterValue(
                        LaunchConfiguration("detector_circle_angle_bin_deg"),
                        value_type=float,
                    ),
                    "circle_max_gap_bins": ParameterValue(
                        LaunchConfiguration("detector_circle_max_gap_bins"),
                        value_type=int,
                    ),
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            # Camera device index passed to camera_node.
            DeclareLaunchArgument("camera_index", default_value="0"),
            # Camera stream role requested by camera_node.
            DeclareLaunchArgument("role", default_value="viewfinder"),
            # Pixel format requested from the camera driver.
            DeclareLaunchArgument("format", default_value="RGB888"),
            # Requested camera image width in pixels.
            DeclareLaunchArgument(
                "width",
                default_value="800",
            ),
            # Requested camera image height in pixels.
            DeclareLaunchArgument(
                "height",
                default_value="600",
            ),
            # Camera image orientation value passed to camera_node.
            DeclareLaunchArgument("orientation", default_value="0"),
            # Camera sensor mode string passed to camera_node.
            DeclareLaunchArgument("sensor_mode", default_value="1332:990"),
            # Frame ID used in the published camera messages.
            DeclareLaunchArgument("frame_id", default_value="camera"),
            # Optional camera calibration URL for camera_info.
            DeclareLaunchArgument("camera_info_url", default_value=""),
            # Use the node clock instead of the sensor timestamp when true.
            DeclareLaunchArgument("use_node_time", default_value="false"),
            # Output topic for camera_info published by camera_node.
            DeclareLaunchArgument(
                "camera_info_topic", default_value="/camera/camera_info"
            ),
            # Raw image topic published by camera_node and consumed by remap.
            DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),
            *declare_camera_control_arguments(),
            # Remapped top-down image topic produced by fastmap_remap_node.
            DeclareLaunchArgument(
                "output_topic",
                default_value="/black_feature_input_remap_node/image_remapped",
            ),
            # Automatically use the newest fastmap XML from the config folder.
            DeclareLaunchArgument("use_latest_fastmap", default_value="true"),
            # Explicit fastmap XML path; used when use_latest_fastmap is false.
            DeclareLaunchArgument("fastmap_file", default_value=""),
            # Path to the remapped robot mask image used by fastmap_remap_node.
            DeclareLaunchArgument(
                "robot_mask_path",
                default_value=str(
                    Path(get_package_share_directory("rcj_localization"))
                    / "config"
                    / "remapped_mask.png"
                ),
            ),
            # Image transport used by fastmap_remap_node for the input topic.
            DeclareLaunchArgument("input_transport", default_value="raw"),
            # Interpolation mode used by fastmap_remap_node during remapping.
            declare_remap_interpolation_argument(),
            # Enable all remap debug windows when true.
            DeclareLaunchArgument("remap_enable_image_view", default_value="false"),
            # Show the raw input image in the remap node debug view.
            DeclareLaunchArgument("remap_show_input_image", default_value="false"),
            # Show the remapped output image in the remap node debug view.
            DeclareLaunchArgument("remap_show_output_image", default_value="false"),
            # Maximum width of remap debug windows in pixels.
            DeclareLaunchArgument("remap_display_max_width", default_value="960"),
            # Maximum height of remap debug windows in pixels.
            DeclareLaunchArgument("remap_display_max_height", default_value="720"),
            # Print periodic processing timing logs from fastmap_remap_node.
            DeclareLaunchArgument("remap_enable_timing_log", default_value="true"),
            # Number of frames between remap timing log messages.
            DeclareLaunchArgument("remap_timing_log_interval", default_value="30"),
            # Minimum HSV hue allowed for white classification.
            DeclareLaunchArgument("white_h_min", default_value="0"),
            # Maximum HSV hue allowed for white classification.
            DeclareLaunchArgument("white_h_max", default_value="179"),
            # Maximum HSV saturation allowed for white pixels.
            DeclareLaunchArgument("white_s_max", default_value="118"),
            # Minimum HSV value required for white pixels.
            DeclareLaunchArgument("white_v_min", default_value="197"),
            # Minimum HSV hue allowed for black classification.
            DeclareLaunchArgument("black_h_min", default_value="0"),
            # Maximum HSV hue allowed for black classification.
            DeclareLaunchArgument("black_h_max", default_value="179"),
            # Minimum HSV saturation allowed for black classification.
            DeclareLaunchArgument("black_s_min", default_value="0"),
            # Maximum HSV saturation allowed for black classification.
            DeclareLaunchArgument("black_s_max", default_value="255"),
            # Minimum HSV value allowed for black classification.
            DeclareLaunchArgument("black_v_min", default_value="0"),
            # Maximum HSV value allowed for black classification.
            DeclareLaunchArgument("black_v_max", default_value="124"),
            # Minimum HSV hue allowed for green classification.
            DeclareLaunchArgument("green_h_min", default_value="35"),
            # Maximum HSV hue allowed for green classification.
            DeclareLaunchArgument("green_h_max", default_value="100"),
            # Minimum HSV saturation required for green pixels.
            DeclareLaunchArgument("green_s_min", default_value="140"),
            # Maximum HSV saturation allowed for green pixels.
            DeclareLaunchArgument("green_s_max", default_value="255"),
            # Minimum HSV value required for green pixels.
            DeclareLaunchArgument("green_v_min", default_value="80"),
            # Maximum HSV value allowed for green pixels.
            DeclareLaunchArgument("green_v_max", default_value="255"),
            # Print periodic processing timing logs from the HSV node.
            DeclareLaunchArgument("hsv_enable_timing_log", default_value="true"),
            # Number of frames between HSV timing log messages.
            DeclareLaunchArgument("hsv_timing_log_interval", default_value="15"),
            # Enable all HSV debug windows when true.
            DeclareLaunchArgument("hsv_enable_image_view", default_value="true"),
            # Show the HSV parameter controls window when image view is enabled.
            DeclareLaunchArgument("hsv_enable_controls_window", default_value="true"),
            # Show the remapped input image in the HSV node debug view.
            DeclareLaunchArgument("hsv_show_input_image", default_value="false"),
            # Show the white mask produced by the HSV priority node.
            DeclareLaunchArgument("hsv_show_white_mask", default_value="false"),
            # Show the green mask produced by the HSV priority node.
            DeclareLaunchArgument("hsv_show_green_mask", default_value="false"),
            # Show the black mask produced by the HSV priority node.
            DeclareLaunchArgument("hsv_show_black_mask", default_value="true"),
            # Show the leftover noise mask produced by the HSV priority node.
            DeclareLaunchArgument("hsv_show_noise_mask", default_value="false"),
            # Show the HSV classification overlay image.
            DeclareLaunchArgument("hsv_show_overlay_image", default_value="true"),
            # Show the green mask overlaid in bright red on the remapped input image.
            DeclareLaunchArgument("hsv_show_green_overlay", default_value="false"),
            # Show the black mask overlaid in red on the remapped input image.
            DeclareLaunchArgument("hsv_show_black_overlay", default_value="true"),
            # Maximum width of HSV debug windows in pixels.
            DeclareLaunchArgument("hsv_display_max_width", default_value="960"),
            # Maximum height of HSV debug windows in pixels.
            DeclareLaunchArgument("hsv_display_max_height", default_value="720"),
            # Enable all black detector debug windows when true.
            DeclareLaunchArgument("detector_enable_image_view", default_value="true"),
            # Publish the final overlay debug image topic from the detector.
            DeclareLaunchArgument("detector_publish_debug_image", default_value="false"),
            # Show the remapped input image in the detector debug view.
            DeclareLaunchArgument("detector_show_input_image", default_value="false"),
            # Show the incoming HSV black mask in the detector debug view.
            DeclareLaunchArgument(
                "detector_show_input_black_mask", default_value="false"
            ),
            # Show the binary-normalized mask after forcing all nonzero values to 255.
            DeclareLaunchArgument(
                "detector_show_binary_normalized_mask", default_value="false"
            ),
            # Show the intermediate mask after morphological opening.
            DeclareLaunchArgument(
                "detector_show_morph_open_mask", default_value="false"
            ),
            # Show the intermediate mask after morphological closing.
            DeclareLaunchArgument(
                "detector_show_morph_close_mask", default_value="false"
            ),
            # Show the intermediate mask after removing border-touching components.
            DeclareLaunchArgument(
                "detector_show_border_filtered_mask", default_value="false"
            ),
            # Show the cleaned binary mask after morphology and border filtering.
            DeclareLaunchArgument("detector_show_clean_mask", default_value="true"),
            # Show the detector mask containing accepted dot features.
            DeclareLaunchArgument("detector_show_dot_mask", default_value="false"),
            # Show the detector mask containing accepted hollow circle features.
            DeclareLaunchArgument("detector_show_circle_mask", default_value="false"),
            # Show every non-dot component sent into circle evaluation.
            DeclareLaunchArgument(
                "detector_show_circle_candidate_mask", default_value="true"
            ),
            # Show whole circle candidates that produced no accepted arcs.
            DeclareLaunchArgument(
                "detector_show_circle_rejected_component_mask",
                default_value="false",
            ),
            # Show arc fragments evaluated by the circle stage but rejected.
            DeclareLaunchArgument(
                "detector_show_circle_rejected_arc_mask", default_value="false"
            ),
            # Show whole candidates rejected because the fitted radius is out of range.
            DeclareLaunchArgument(
                "detector_show_circle_radius_rejected_mask", default_value="false"
            ),
            # Show whole candidates rejected because they contain too few pixels.
            DeclareLaunchArgument(
                "detector_show_circle_too_few_pixels_rejected_mask",
                default_value="false",
            ),
            # Show whole candidates rejected by the radial residual fit check.
            DeclareLaunchArgument(
                "detector_show_circle_radial_fit_rejected_mask",
                default_value="false",
            ),
            # Show whole candidates rejected because no occupied angle segments were found.
            DeclareLaunchArgument(
                "detector_show_circle_no_segments_rejected_mask",
                default_value="false",
            ),
            # Show the final filtered black feature mask.
            DeclareLaunchArgument(
                "detector_show_black_final_mask", default_value="true"
            ),
            # Show the final overlay image with black_final_mask on the remapped image.
            DeclareLaunchArgument("detector_show_debug_image", default_value="false"),
            # Maximum width of detector debug windows in pixels.
            DeclareLaunchArgument("detector_display_max_width", default_value="960"),
            # Maximum height of detector debug windows in pixels.
            DeclareLaunchArgument("detector_display_max_height", default_value="720"),
            # Kernel size for the detector opening operation used to remove small noise.
            DeclareLaunchArgument("detector_open_kernel", default_value="3"),
            # Kernel size for the detector closing operation used to fill small gaps.
            DeclareLaunchArgument("detector_close_kernel", default_value="5"),
            # Minimum contour area in pixels for a feature to be accepted as a dot.
            DeclareLaunchArgument("detector_dot_min_area_px", default_value="8"),
            # Maximum contour area in pixels for a feature to be accepted as a dot.
            DeclareLaunchArgument("detector_dot_max_area_px", default_value="220"),
            # Minimum contour circularity required for dot detection.
            DeclareLaunchArgument(
                "detector_dot_min_circularity", default_value="0.45"
            ),
            # Minimum contour solidity required for dot detection.
            DeclareLaunchArgument("detector_dot_min_solidity", default_value="0.75"),
            # Minimum fitted radius in pixels for hollow circle detection.
            DeclareLaunchArgument(
                "detector_circle_min_radius_px", default_value="0.0"
            ),
            # Maximum fitted radius in pixels for hollow circle detection.
            DeclareLaunchArgument(
                "detector_circle_max_radius_px", default_value="500.0"
            ),
            # Width of the sampled ring band used to evaluate hollow circles.
            DeclareLaunchArgument(
                "detector_circle_ring_width_px", default_value="8.0"
            ),
            # Inner guard distance in pixels used to check that the circle center stays mostly empty.
            DeclareLaunchArgument(
                "detector_circle_inner_guard_px", default_value="8.0"
            ),
            # Outer guard distance in pixels used to reject circles with too much nearby black outside the ring.
            DeclareLaunchArgument(
                "detector_circle_outer_guard_px", default_value="8.0"
            ),
            # Minimum fraction of black pixels required along the fitted circle ring.
            DeclareLaunchArgument(
                "detector_circle_min_ring_black_ratio", default_value="0.18"
            ),
            # Maximum fraction of black pixels allowed inside the fitted hollow circle.
            DeclareLaunchArgument(
                "detector_circle_max_inner_black_ratio", default_value="0.08"
            ),
            # Maximum fraction of black pixels allowed just outside the fitted circle.
            DeclareLaunchArgument(
                "detector_circle_max_outer_black_ratio", default_value="0.12"
            ),
            # Maximum average radial fitting error in pixels for a hollow circle candidate.
            DeclareLaunchArgument(
                "detector_circle_max_radial_residual_px", default_value="28.0"
            ),
            # Minimum angular coverage in degrees required for a hollow circle candidate.
            DeclareLaunchArgument(
                "detector_circle_min_arc_span_deg", default_value="45.0"
            ),
            # Angular bin size in degrees used when measuring circle coverage.
            DeclareLaunchArgument(
                "detector_circle_angle_bin_deg", default_value="6.0"
            ),
            # Maximum number of empty angular bins allowed inside one arc segment.
            DeclareLaunchArgument(
                "detector_circle_max_gap_bins", default_value="10"
            ),
            OpaqueFunction(function=build_nodes),
        ]
    )
