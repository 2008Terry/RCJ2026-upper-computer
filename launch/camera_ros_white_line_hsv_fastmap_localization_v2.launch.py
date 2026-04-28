import xml.etree.ElementTree as ET
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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
    declare_camera_ros_arguments,
    declare_hsv_green_white_black_arguments,
    hsv_green_white_black_launch_arguments,
    hsv_green_white_black_parameters,
)
from rcj_fastmap_hsv_pipeline import build_camera_hsv_fastmap_nodes


def find_latest_fastmap_file():
    config_dir = Path(get_package_share_directory("rcj_localization")) / "config"
    candidates = sorted(config_dir.glob("undistort_map_*_fast.xml"))
    if not candidates:
        candidates = sorted(config_dir.glob("*.xml"))
    if not candidates:
        raise FileNotFoundError(f"No fastmap XML file found in {config_dir}")
    return candidates[-1]


def generate_launch_description():
    package_share = Path(get_package_share_directory("rcj_localization"))
    map_yaml_default = package_share / "maps" / "rcj_map.yaml"

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
    remap_topic = LaunchConfiguration("remap_topic")
    white_mask_topic = LaunchConfiguration("white_mask_topic")
    robot_mask_path = LaunchConfiguration("robot_mask_path")
    fastmap_file = LaunchConfiguration("fastmap_file")
    input_transport = LaunchConfiguration("input_transport")
    interpolation = LaunchConfiguration("interpolation")

    map_yaml_file = LaunchConfiguration("map_yaml_file")
    use_fake_yaw = LaunchConfiguration("use_fake_yaw")
    yaw_topic = LaunchConfiguration("yaw_topic")
    map_topic = LaunchConfiguration("map_topic")
    enable_localization = LaunchConfiguration("enable_localization")
    enable_map_server = LaunchConfiguration("enable_map_server")
    enable_lifecycle_manager = LaunchConfiguration("enable_lifecycle_manager")
    enable_yaw_publisher = LaunchConfiguration("enable_yaw_publisher")
    enable_topdown_pf_localization_node_v2 = LaunchConfiguration(
        "enable_topdown_pf_localization_node_v2"
    )
    meters_per_pixel = LaunchConfiguration("meters_per_pixel")
    forward_axis = LaunchConfiguration("forward_axis")
    left_axis = LaunchConfiguration("left_axis")
    max_points = LaunchConfiguration("max_points")
    publish_debug_pointcloud = LaunchConfiguration("publish_debug_pointcloud")
    debug_pointcloud_topic = LaunchConfiguration("debug_pointcloud_topic")
    num_particles = LaunchConfiguration("num_particles")
    sigma_hit = LaunchConfiguration("sigma_hit")
    noise_xy = LaunchConfiguration("noise_xy")
    noise_theta = LaunchConfiguration("noise_theta")
    alpha_fast_rate = LaunchConfiguration("alpha_fast_rate")
    alpha_slow_rate = LaunchConfiguration("alpha_slow_rate")
    random_injection_max_ratio = LaunchConfiguration("random_injection_max_ratio")
    off_map_penalty = LaunchConfiguration("off_map_penalty")
    occupancy_threshold = LaunchConfiguration("occupancy_threshold")
    distance_transform_mask_size = LaunchConfiguration("distance_transform_mask_size")
    init_field_width = LaunchConfiguration("init_field_width")
    init_field_height = LaunchConfiguration("init_field_height")
    filter_period_ms = LaunchConfiguration("filter_period_ms")
    topdown_pf_publish_processing_time = LaunchConfiguration(
        "topdown_pf_publish_processing_time"
    )
    topdown_pf_processing_time_topic = LaunchConfiguration(
        "topdown_pf_processing_time_topic"
    )
    topdown_pf_enable_timing_log = LaunchConfiguration("topdown_pf_enable_timing_log")
    topdown_pf_timing_log_interval = LaunchConfiguration(
        "topdown_pf_timing_log_interval"
    )

    fake_yaw_condition = IfCondition(
        PythonExpression(
            [
                "'",
                enable_yaw_publisher,
                "' == 'true' and '",
                use_fake_yaw,
                "' == 'true'",
            ]
        )
    )

    return LaunchDescription(
        [
            *declare_camera_ros_arguments(),
            DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),  # Camera info topic
            DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),  # Raw image topic
            DeclareLaunchArgument(
                "remap_topic",
                default_value="/white_line_hsv_input_remap_node/image_remapped",
            ),  # Remapped image topic
            DeclareLaunchArgument(
                "white_mask_topic",
                default_value="/white_line_hsv_white_node/white_mask",
            ),  # White mask topic
            DeclareLaunchArgument("robot_mask_path", default_value=str(Path(get_package_share_directory("rcj_localization")) / "config" / "remapped_mask.png")),  # Optional remapped-space robot mask image path
            DeclareLaunchArgument("use_latest_fastmap", default_value="false"),  # Whether to auto-select the latest Fastmap XML
            DeclareLaunchArgument("fastmap_file", default_value=""),  # Specific Fastmap XML path when auto-select is disabled
            DeclareLaunchArgument("input_transport", default_value="raw"),  # Remap input transport
            declare_remap_interpolation_argument(),
            DeclareLaunchArgument("remap_enable_image_view", default_value="false"),  # Whether to show remap windows
            DeclareLaunchArgument("remap_enable_timing_log", default_value="true"),  # Whether to log remap timing
            DeclareLaunchArgument("remap_timing_log_interval", default_value="30"),  # Remap timing log frame interval
            *declare_hsv_green_white_black_arguments(),
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
            DeclareLaunchArgument("map_yaml_file", default_value=str(map_yaml_default)),  # Map YAML file path
            DeclareLaunchArgument("use_fake_yaw", default_value="true"),  # Whether to publish fake yaw
            DeclareLaunchArgument("yaw_topic", default_value="/robot/yaw"),  # Yaw topic
            DeclareLaunchArgument("yaw_enable_publish_log", default_value="false"),  # Whether to log each fake yaw publish
            DeclareLaunchArgument("map_topic", default_value="/map"),  # Occupancy map topic
            DeclareLaunchArgument("enable_localization", default_value="true"),  # Whether PF localization is enabled
            DeclareLaunchArgument(
                "enable_map_server",
                default_value=enable_localization,
            ),  # Whether to start the map server
            DeclareLaunchArgument(
                "enable_lifecycle_manager",
                default_value=enable_map_server,
            ),  # Whether to autostart map server lifecycle
            DeclareLaunchArgument(
                "enable_yaw_publisher",
                default_value=enable_localization,
            ),  # Whether to start the fake yaw publisher
            DeclareLaunchArgument(
                "enable_topdown_pf_localization_node_v2",
                default_value="true",
            ),  # Whether to start PF localization v2
            DeclareLaunchArgument("meters_per_pixel", default_value="0.0019"),  # Meters per mask pixel
            DeclareLaunchArgument("forward_axis", default_value="v+"),  # Forward axis mapping
            DeclareLaunchArgument("left_axis", default_value="u-"),  # Left axis mapping
            DeclareLaunchArgument("max_points", default_value="3000"),  # Max sampled mask points
            DeclareLaunchArgument("publish_debug_pointcloud", default_value="true"),  # Whether to publish debug point cloud
            DeclareLaunchArgument(
                "debug_pointcloud_topic",
                default_value="/field_line_observations_debug",
            ),  # Debug point cloud topic
            DeclareLaunchArgument("num_particles", default_value="1000"),  # Particle count
            DeclareLaunchArgument("sigma_hit", default_value="0.10"),  # Measurement sigma in meters
            DeclareLaunchArgument("noise_xy", default_value="0.05"),  # Position noise std in meters
            DeclareLaunchArgument("noise_theta", default_value="0.10"),  # Heading noise std in radians
            DeclareLaunchArgument("alpha_fast_rate", default_value="0.1"),  # Fast average update rate
            DeclareLaunchArgument("alpha_slow_rate", default_value="0.001"),  # Slow average update rate
            DeclareLaunchArgument(
                "random_injection_max_ratio",
                default_value="0.25",
            ),  # Max random injection ratio
            DeclareLaunchArgument("off_map_penalty", default_value="1.0"),  # Off-map penalty in meters
            DeclareLaunchArgument("occupancy_threshold", default_value="50"),  # Map occupancy threshold
            DeclareLaunchArgument(
                "distance_transform_mask_size",
                default_value="5",
            ),  # Distance transform mask size
            DeclareLaunchArgument("init_field_width", default_value="2.0"),  # Initial particle field width in meters
            DeclareLaunchArgument("init_field_height", default_value="3.0"),  # Initial particle field height in meters
            DeclareLaunchArgument("filter_period_ms", default_value="80"),  # PF timer period in milliseconds
            DeclareLaunchArgument(
                "topdown_pf_publish_processing_time", default_value="true"
            ),  # Whether to publish PF processing time
            DeclareLaunchArgument(
                "topdown_pf_processing_time_topic",
                default_value="~/processing_time_ms",
            ),  # PF processing time topic
            DeclareLaunchArgument(
                "topdown_pf_enable_timing_log", default_value="true"
            ),  # Whether to log PF timing
            DeclareLaunchArgument(
                "topdown_pf_timing_log_interval", default_value="10"
            ),  # PF timing log iteration interval
            OpaqueFunction(function=build_camera_hsv_fastmap_nodes),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                condition=IfCondition(enable_map_server),
                parameters=[{"yaml_filename": map_yaml_file}],  # Map YAML file path
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                condition=IfCondition(enable_lifecycle_manager),
                parameters=[
                    {
                        "autostart": True,  # Whether to autostart lifecycle nodes
                        "node_names": ["map_server"],  # Lifecycle-managed node list
                    }
                ],
            ),
            Node(
                package="rcj_localization",
                executable="yaw_publisher.py",
                name="yaw_publisher",
                output="screen",
                condition=fake_yaw_condition,
                parameters=[
                    {
                        "enable_publish_log": ParameterValue(
                            LaunchConfiguration("yaw_enable_publish_log"),
                            value_type=bool,
                        ),  # Whether to log each fake yaw publish
                    }
                ],
                remappings=[("/robot/yaw", yaw_topic)],  # Yaw output topic
            ),
            Node(
                package="rcj_localization",
                executable="topdown_pf_localization_node_v2",
                name="topdown_pf_localization_node_v2",
                output="screen",
                condition=IfCondition(enable_topdown_pf_localization_node_v2),
                parameters=[
                    {
                        "mask_topic": white_mask_topic,  # White mask input topic for PF
                        "meters_per_pixel": ParameterValue(
                            meters_per_pixel, value_type=float
                        ),  # Meters per mask pixel
                        "forward_axis": forward_axis,  # Forward axis mapping
                        "left_axis": left_axis,  # Left axis mapping
                        "max_points": ParameterValue(
                            max_points, value_type=int
                        ),  # Max sampled mask points
                        "enable_localization": ParameterValue(
                            enable_localization, value_type=bool
                        ),  # Whether PF localization is enabled
                        "publish_debug_pointcloud": ParameterValue(
                            publish_debug_pointcloud, value_type=bool
                        ),  # Whether to publish debug point cloud
                        "debug_pointcloud_topic": debug_pointcloud_topic,  # Debug point cloud topic
                        "num_particles": ParameterValue(
                            num_particles, value_type=int
                        ),  # Particle count
                        "map_topic": map_topic,  # Occupancy map topic
                        "yaw_topic": yaw_topic,  # Yaw topic
                        "sigma_hit": ParameterValue(
                            sigma_hit, value_type=float
                        ),  # Measurement sigma in meters
                        "noise_xy": ParameterValue(
                            noise_xy, value_type=float
                        ),  # Position noise std in meters
                        "noise_theta": ParameterValue(
                            noise_theta, value_type=float
                        ),  # Heading noise std in radians
                        "alpha_fast_rate": ParameterValue(
                            alpha_fast_rate, value_type=float
                        ),  # Fast average update rate
                        "alpha_slow_rate": ParameterValue(
                            alpha_slow_rate, value_type=float
                        ),  # Slow average update rate
                        "random_injection_max_ratio": ParameterValue(
                            random_injection_max_ratio, value_type=float
                        ),  # Max random injection ratio
                        "off_map_penalty": ParameterValue(
                            off_map_penalty, value_type=float
                        ),  # Off-map penalty in meters
                        "occupancy_threshold": ParameterValue(
                            occupancy_threshold, value_type=int
                        ),  # Map occupancy threshold
                        "distance_transform_mask_size": ParameterValue(
                            distance_transform_mask_size, value_type=int
                        ),  # Distance transform mask size
                        "init_field_width": ParameterValue(
                            init_field_width, value_type=float
                        ),  # Initial particle field width in meters
                        "init_field_height": ParameterValue(
                            init_field_height, value_type=float
                        ),  # Initial particle field height in meters
                        "filter_period_ms": ParameterValue(
                            filter_period_ms, value_type=int
                        ),  # PF timer period in milliseconds
                        "publish_processing_time": ParameterValue(
                            topdown_pf_publish_processing_time, value_type=bool
                        ),  # Whether to publish PF processing time
                        "processing_time_topic": topdown_pf_processing_time_topic,  # PF processing time topic
                        "enable_timing_log": ParameterValue(
                            topdown_pf_enable_timing_log, value_type=bool
                        ),  # Whether to log PF timing
                        "timing_log_interval": ParameterValue(
                            topdown_pf_timing_log_interval, value_type=int
                        ),  # PF timing log iteration interval
                    }
                ],
            ),
        ]
    )
