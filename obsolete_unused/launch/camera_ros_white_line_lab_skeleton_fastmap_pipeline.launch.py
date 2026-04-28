from pathlib import Path
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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


def resolve_legacy_bool(current_name, legacy_name):
    return ParameterValue(
        PythonExpression(
            [
                "'",
                LaunchConfiguration(legacy_name),
                "' != '__unset__' and '",
                LaunchConfiguration(legacy_name),
                "' or '",
                LaunchConfiguration(current_name),
                "'",
            ]
        ),
        value_type=bool,
    )


def legacy_arg_used_condition(name):
    return IfCondition(
        PythonExpression(["'", LaunchConfiguration(name), "' != '__unset__'"])
    )


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
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    orientation = LaunchConfiguration("orientation")
    sensor_mode = LaunchConfiguration("sensor_mode")
    frame_id = LaunchConfiguration("frame_id")
    camera_info_url = LaunchConfiguration("camera_info_url")
    use_node_time = LaunchConfiguration("use_node_time")
    exposure_time = LaunchConfiguration("exposure_time")
    exposure_time_mode = LaunchConfiguration("exposure_time_mode")
    ae_enable = LaunchConfiguration("ae_enable")
    analogue_gain = LaunchConfiguration("analogue_gain")
    awb_enable = LaunchConfiguration("awb_enable")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    robot_mask_path = LaunchConfiguration("robot_mask_path")
    input_transport = LaunchConfiguration("input_transport")
    interpolation = LaunchConfiguration("interpolation")

    morph_node_name = "white_line_lab_morph_node"
    skeleton_show_white_final_mask = resolve_legacy_bool(
        "skeleton_show_white_final_mask",
        "skeleton_show_white_mask",
    )

    return [
            LogInfo(
                condition=legacy_arg_used_condition("skeleton_show_white_mask"),
                msg=(
                    "Launch argument 'skeleton_show_white_mask' is deprecated. "
                    "Use 'skeleton_show_white_final_mask' instead."
                ),
            ),
            Node(
                package="camera_ros",
                executable="camera_node",
                name="camera",
                output="screen",
                remappings=[
                    ("~/image_raw", input_topic),
                    ("~/camera_info", camera_info_topic),
                ],
                parameters=[
                    {
                        "camera": ParameterValue(camera_index, value_type=int),
                        "role": role,
                        "format": image_format,
                        "width": ParameterValue(width_value, value_type=int),
                        "height": ParameterValue(height_value, value_type=int),
                        "orientation": ParameterValue(orientation, value_type=int),
                        "sensor_mode": sensor_mode,
                        "frame_id": frame_id,
                        "camera_info_url": camera_info_url,
                        "use_node_time": ParameterValue(use_node_time, value_type=bool),
                        **camera_control_parameters(),
                    }
                ],
            ),
            Node(
                package="rcj_localization",
                executable="fastmap_remap_node",
                name="white_line_lab_input_remap_node",
                output="screen",
                parameters=[
                    {
                        "fastmap_file": str(selected_fastmap_file),
                        "robot_mask_path": robot_mask_path,
                        "input_topic": input_topic,
                        "output_topic": output_topic,
                        "input_transport": input_transport,
                        "interpolation": interpolation,
                        "enable_image_view": LaunchConfiguration("remap_enable_image_view"),
                        "show_input_image": LaunchConfiguration("remap_show_input_image"),
                        "show_output_image": LaunchConfiguration("remap_show_output_image"),
                    }
                ],
            ),
            Node(
                package="rcj_localization",
                executable="white_line_lab_morph_node",
                name=morph_node_name,
                output="screen",
                parameters=[
                    {
                        "input_topic": output_topic,
                        "robot_mask_topic": "/white_line_lab_input_remap_node/robot_mask",
                        "green_a_max": 117,
                        "enable_image_view": LaunchConfiguration("morph_enable_image_view"),
                        "show_input_image": LaunchConfiguration("morph_show_input_image"),
                        "show_white_candidate_mask": LaunchConfiguration(
                            "morph_show_white_candidate_mask"
                        ),
                        "show_white_morph_mask": LaunchConfiguration("morph_show_white_morph_mask"),
                        "show_green_mask": LaunchConfiguration("morph_show_green_mask"),
                        "show_black_mask": LaunchConfiguration("morph_show_black_mask"),
                        "show_noise_mask": LaunchConfiguration("morph_show_noise_mask"),
                        "show_debug_image": LaunchConfiguration("morph_show_debug_image"),
                        "enable_timing_debug": LaunchConfiguration("morph_enable_timing_debug"),
                        "timing_summary_interval": LaunchConfiguration(
                            "morph_timing_summary_interval"
                        ),
                    }
                ],
            ),
            Node(
                package="rcj_localization",
                executable="white_line_skeleton_filter_node",
                name="white_line_skeleton_filter_node",
                output="screen",
                parameters=[
                    {
                        "morph_mask_topic": f"/{morph_node_name}/white_morph_mask",
                        "green_mask_topic": f"/{morph_node_name}/green_mask",
                        "black_mask_topic": f"/{morph_node_name}/black_mask",
                        "noise_mask_topic": f"/{morph_node_name}/noise_mask",
                        "orientation_window_radius_px": 5,
                        "min_orientation_neighbors": 6,
                        "enable_image_view": LaunchConfiguration("skeleton_enable_image_view"),
                        "show_morph_mask": LaunchConfiguration("skeleton_show_morph_mask"),
                        "show_green_mask": LaunchConfiguration("skeleton_show_green_mask"),
                        "show_black_mask": LaunchConfiguration("skeleton_show_black_mask"),
                        "show_noise_mask": LaunchConfiguration("skeleton_show_noise_mask"),
                        "show_skeleton_mask": LaunchConfiguration("skeleton_show_skeleton_mask"),
                        "show_orientation_valid_mask": LaunchConfiguration(
                            "skeleton_show_orientation_valid_mask"
                        ),
                        "show_side_support_mask": LaunchConfiguration(
                            "skeleton_show_side_support_mask"
                        ),
                        "show_width_supported_skeleton_mask": LaunchConfiguration(
                            "skeleton_show_width_supported_skeleton_mask"
                        ),
                        "show_length_filtered_skeleton_mask": LaunchConfiguration(
                            "skeleton_show_length_filtered_skeleton_mask"
                        ),
                        "show_supported_skeleton_mask": LaunchConfiguration(
                            "skeleton_show_length_filtered_skeleton_mask"
                        ),
                        "show_reconstructed_mask": LaunchConfiguration(
                            "skeleton_show_reconstructed_mask"
                        ),
                        "show_white_final_mask": skeleton_show_white_final_mask,
                        "show_debug_image": LaunchConfiguration("skeleton_show_debug_image"),
                        "enable_timing_debug": LaunchConfiguration(
                            "skeleton_enable_timing_debug"
                        ),
                        "timing_summary_interval": LaunchConfiguration(
                            "skeleton_timing_summary_interval"
                        ),
                    }
                ],
            ),
    ]


def generate_launch_description():
    launch_arguments = [
        DeclareLaunchArgument("camera_index", default_value="0"),  # Camera index
        DeclareLaunchArgument("role", default_value="viewfinder"),  # camera_ros role
        DeclareLaunchArgument("format", default_value="RGB888"),  # Camera pixel format
        DeclareLaunchArgument("width", default_value="800"),  # Capture width, empty means use fastmap source width
        DeclareLaunchArgument("height", default_value="600"),  # Capture height, empty means use fastmap source height
        DeclareLaunchArgument("orientation", default_value="0"),  # Camera rotation angle
        DeclareLaunchArgument("sensor_mode", default_value="1332:990"),  # Camera sensor mode
        DeclareLaunchArgument("frame_id", default_value="camera"),  # Image frame id
        DeclareLaunchArgument("camera_info_url", default_value=""),  # Camera calibration URL
        DeclareLaunchArgument("use_node_time", default_value="false"),  # Use node time instead of sensor timestamps
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),  # Camera info topic
        DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),  # Raw image topic
            *declare_camera_control_arguments(),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/white_line_lab_input_remap_node/image_remapped",
        ),  # Remapped image topic
        DeclareLaunchArgument("robot_mask_path", default_value=str(Path(get_package_share_directory("rcj_localization")) / "config" / "remapped_mask.png")),  # Optional remapped-space robot mask image path
        DeclareLaunchArgument("use_latest_fastmap", default_value="false"),  # Auto-select the latest fastmap XML
        DeclareLaunchArgument("fastmap_file", default_value=""),  # Specific fastmap XML path
        DeclareLaunchArgument("input_transport", default_value="raw"),  # Remap input transport
        declare_remap_interpolation_argument(),
        DeclareLaunchArgument("camera_enable_image_view", default_value="false"),  # Show camera debug window
        DeclareLaunchArgument("camera_show_published_image", default_value="true"),  # Show the camera output image
        DeclareLaunchArgument("remap_enable_image_view", default_value="false"),  # Show remap debug windows
        DeclareLaunchArgument("remap_show_input_image", default_value="false"),  # Show the remap input image
        DeclareLaunchArgument("remap_show_output_image", default_value="true"),  # Show the remap output image
        DeclareLaunchArgument("morph_enable_image_view", default_value="false"),  # Show morph debug windows
        DeclareLaunchArgument("morph_show_input_image", default_value="false"),  # Show the morph input image
        DeclareLaunchArgument("morph_show_white_candidate_mask", default_value="true"),  # Show the white candidate mask
        DeclareLaunchArgument("morph_show_white_morph_mask", default_value="true"),  # Show the morph output mask
        DeclareLaunchArgument("morph_show_green_mask", default_value="false"),  # Show the green mask
        DeclareLaunchArgument("morph_show_black_mask", default_value="false"),  # Show the black mask
        DeclareLaunchArgument("morph_show_noise_mask", default_value="false"),  # Show the noise mask
        DeclareLaunchArgument("morph_show_debug_image", default_value="false"),  # Show the morph debug image
        DeclareLaunchArgument("morph_enable_timing_debug", default_value="true"),  # Enable morph timing logs
        DeclareLaunchArgument("morph_timing_summary_interval", default_value="10"),  # Morph timing summary frame interval
        DeclareLaunchArgument("skeleton_enable_image_view", default_value="false"),  # Show skeleton debug windows
        DeclareLaunchArgument("skeleton_show_morph_mask", default_value="true"),  # Show the skeleton input morph mask
        DeclareLaunchArgument("skeleton_show_green_mask", default_value="false"),  # Show the skeleton green mask
        DeclareLaunchArgument("skeleton_show_black_mask", default_value="false"),  # Show the skeleton black mask
        DeclareLaunchArgument("skeleton_show_noise_mask", default_value="false"),  # Show the skeleton noise mask
        DeclareLaunchArgument("skeleton_show_skeleton_mask", default_value="false"),  # Show the raw skeleton mask
        DeclareLaunchArgument("skeleton_show_orientation_valid_mask", default_value="false"),  # Show the orientation-valid mask
        DeclareLaunchArgument("skeleton_show_side_support_mask", default_value="true"),  # Show the side-support mask
        DeclareLaunchArgument("skeleton_show_width_supported_skeleton_mask", default_value="true"),  # Show the width-filtered skeleton
        DeclareLaunchArgument("skeleton_show_supported_skeleton_mask", default_value="true"),  # Deprecated alias for the length-filtered skeleton view
        DeclareLaunchArgument(
            "skeleton_show_length_filtered_skeleton_mask",
            default_value=LaunchConfiguration("skeleton_show_supported_skeleton_mask"),
        ),  # Show the length-filtered skeleton
        DeclareLaunchArgument("skeleton_show_reconstructed_mask", default_value="true"),  # Show the reconstructed white mask
        DeclareLaunchArgument("skeleton_show_white_final_mask", default_value="true"),  # Show the final white mask
        DeclareLaunchArgument("skeleton_show_white_mask", default_value="__unset__"),  # Deprecated alias for the final white mask view
        DeclareLaunchArgument("skeleton_show_debug_image", default_value="false"),  # Show the skeleton debug mosaic
        DeclareLaunchArgument("skeleton_enable_timing_debug", default_value="true"),  # Enable skeleton timing logs
        DeclareLaunchArgument("skeleton_timing_summary_interval", default_value="10"),  # Skeleton timing summary frame interval
    ]

    return LaunchDescription(
        launch_arguments
        + [
            OpaqueFunction(function=build_nodes),
        ]
    )
