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
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")
    orientation = LaunchConfiguration("orientation")
    sensor_mode = LaunchConfiguration("sensor_mode")
    frame_id = LaunchConfiguration("frame_id")
    camera_info_url = LaunchConfiguration("camera_info_url")
    use_node_time = LaunchConfiguration("use_node_time")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    input_topic = LaunchConfiguration("input_topic")
    remap_topic = LaunchConfiguration("remap_topic")
    white_mask_topic = LaunchConfiguration("white_mask_topic")
    fastmap_file = LaunchConfiguration("fastmap_file")
    input_transport = LaunchConfiguration("input_transport")
    interpolation = LaunchConfiguration("interpolation")

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
                name="white_line_hsv_white_node",
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
                        "enable_controls_window": ParameterValue(
                            LaunchConfiguration("hsv_enable_controls_window"), value_type=bool
                        ),  # Whether to show HSV slider controls window
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
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("camera_index", default_value="0"),  # Camera index
            DeclareLaunchArgument("role", default_value="viewfinder"),  # camera_ros role
            DeclareLaunchArgument("format", default_value="RGB888"),  # Camera pixel format
            DeclareLaunchArgument("width", default_value="800"),  # Capture width, empty means use selected Fastmap source width
            DeclareLaunchArgument("height", default_value="600"),  # Capture height, empty means use selected Fastmap source height
            DeclareLaunchArgument("orientation", default_value="0"),  # Camera rotation angle
            DeclareLaunchArgument("sensor_mode", default_value="1332:990"),  # Camera sensor mode
            DeclareLaunchArgument("frame_id", default_value="camera"),  # Image frame id
            DeclareLaunchArgument("camera_info_url", default_value=""),  # Camera calibration URL
            DeclareLaunchArgument("use_node_time", default_value="false"),  # Whether to use node time
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),  # Camera info topic
            DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),  # Raw image topic
            DeclareLaunchArgument("remap_topic", default_value="/camera/image_remapped"),  # Remapped image topic
            DeclareLaunchArgument("white_mask_topic", default_value="/camera/white_mask"),  # White mask topic
            DeclareLaunchArgument("use_latest_fastmap", default_value="false"),  # Whether to auto-select the latest Fastmap XML
            DeclareLaunchArgument("fastmap_file", default_value=""),  # Specific Fastmap XML path when auto-select is disabled
            DeclareLaunchArgument("input_transport", default_value="raw"),  # Remap input transport
            DeclareLaunchArgument("interpolation", default_value="linear"),  # Remap interpolation mode
            DeclareLaunchArgument("remap_enable_image_view", default_value="false"),  # Whether to show remap windows
            DeclareLaunchArgument("remap_enable_timing_log", default_value="true"),  # Whether to log remap timing
            DeclareLaunchArgument("remap_timing_log_interval", default_value="30"),  # Remap timing log frame interval
            DeclareLaunchArgument("white_h_min", default_value="0"),  # White HSV minimum H
            DeclareLaunchArgument("white_h_max", default_value="179"),  # White HSV maximum H
            DeclareLaunchArgument("white_s_max", default_value="118"),  # White HSV maximum S
            DeclareLaunchArgument("white_v_min", default_value="197"),  # White HSV minimum V
            DeclareLaunchArgument("black_v_max", default_value="124"),  # Black HSV maximum V
            DeclareLaunchArgument("green_h_min", default_value="35"),  # Green HSV minimum H
            DeclareLaunchArgument("green_h_max", default_value="100"),  # Green HSV maximum H
            DeclareLaunchArgument("green_s_min", default_value="140"),  # Green HSV minimum S
            DeclareLaunchArgument("green_v_min", default_value="80"),  # Green HSV minimum V
            DeclareLaunchArgument("hsv_enable_timing_log", default_value="true"),  # Whether to log HSV timing
            DeclareLaunchArgument("hsv_timing_log_interval", default_value="15"),  # HSV timing log frame interval
            DeclareLaunchArgument("hsv_enable_image_view", default_value="true"),  # Master switch for HSV debug windows; false means no window creation or GUI processing
            DeclareLaunchArgument("hsv_enable_controls_window", default_value="true"),  # Whether to show HSV slider controls window
            DeclareLaunchArgument("hsv_show_input_image", default_value="true"),  # Show the HSV input image window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_white_mask", default_value="true"),  # Show the white-priority mask window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_green_mask", default_value="true"),  # Show the green-priority mask window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_black_mask", default_value="true"),  # Show the black-priority mask window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_noise_mask", default_value="true"),  # Show the remaining noise mask window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_overlay_image", default_value="true"),  # Show the HSV overlay window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_display_max_width", default_value="960"),  # HSV window max width
            DeclareLaunchArgument("hsv_display_max_height", default_value="720"),  # HSV window max height
            OpaqueFunction(function=build_nodes),
        ]
    )
