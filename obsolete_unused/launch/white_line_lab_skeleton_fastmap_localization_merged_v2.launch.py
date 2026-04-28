from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcj_shared_launch_params import declare_remap_interpolation_argument

def find_latest_fastmap_file():
    config_dir = Path(get_package_share_directory("rcj_localization")) / "config"
    candidates = sorted(config_dir.glob("undistort_map_*_fast.xml"))
    if not candidates:
        candidates = sorted(config_dir.glob("*.xml"))
    if not candidates:
        raise FileNotFoundError(f"No fastmap XML file found in {config_dir}")
    return candidates[-1]


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


def enabled_condition(name):
    return IfCondition(LaunchConfiguration(name))


def combined_true_condition(*values):
    expression = []
    for idx, value in enumerate(values):
        if idx > 0:
            expression.append(" and ")
        expression.extend(["'", value, "' == 'true'"])
    return IfCondition(PythonExpression(expression))


def generate_launch_description():
    package_share = Path(get_package_share_directory("rcj_localization"))
    fastmap_default = str(find_latest_fastmap_file())
    selected_fastmap_file = PythonExpression(
        [
            "'",
            LaunchConfiguration("use_latest_fastmap"),
            "' == 'true' and '",
            fastmap_default,
            "' or '",
            LaunchConfiguration("fastmap_file"),
            "'",
        ]
    )
    map_yaml_default = str(package_share / "maps" / "rcj_map.yaml")

    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    timeout_sec = LaunchConfiguration("timeout_sec")
    frame_id = LaunchConfiguration("frame_id")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    input_topic = LaunchConfiguration("input_topic")
    output_topic = LaunchConfiguration("output_topic")
    robot_mask_path = LaunchConfiguration("robot_mask_path")
    fastmap_file = LaunchConfiguration("fastmap_file")
    input_transport = LaunchConfiguration("input_transport")
    interpolation = LaunchConfiguration("interpolation")
    map_yaml_file = LaunchConfiguration("map_yaml_file")
    use_fake_yaw = LaunchConfiguration("use_fake_yaw")
    yaw_topic = LaunchConfiguration("yaw_topic")
    map_topic = LaunchConfiguration("map_topic")
    mask_topic = LaunchConfiguration("mask_topic")
    enable_localization = LaunchConfiguration("enable_localization")

    morph_node_name = "white_line_lab_morph_node"
    skeleton_show_white_final_mask = resolve_legacy_bool(
        "skeleton_show_white_final_mask",
        "skeleton_show_white_mask",
    )

    launch_arguments = [
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),  # Serial port for the camera publisher
        DeclareLaunchArgument("baudrate", default_value="115200"),  # Serial baudrate
        DeclareLaunchArgument("timeout_sec", default_value="1.0"),  # Serial read timeout in seconds
        DeclareLaunchArgument("frame_id", default_value="camera"),  # Frame id for published images
        DeclareLaunchArgument("camera_info_topic", default_value="/camera/camera_info"),  # Camera info topic
        DeclareLaunchArgument("input_topic", default_value="/camera/image_raw"),  # Raw input image topic
        DeclareLaunchArgument(
            "output_topic",
            default_value="/white_line_lab_input_remap_node/image_remapped",
        ),  # Remapped output image topic
        DeclareLaunchArgument("robot_mask_path", default_value=str(Path(get_package_share_directory("rcj_localization")) / "config" / "remapped_mask.png")),  # Optional remapped-space robot mask image path
        DeclareLaunchArgument("use_latest_fastmap", default_value="true"),  # Auto-select the newest fastmap file
        DeclareLaunchArgument("fastmap_file", default_value=""),  # Fastmap XML path when auto-select is off
        DeclareLaunchArgument("input_transport", default_value="raw"),  # Input transport for the remap node
        declare_remap_interpolation_argument(),
        DeclareLaunchArgument("map_yaml_file", default_value=map_yaml_default),  # Map YAML file path
        DeclareLaunchArgument("use_fake_yaw", default_value="true"),  # Publish fake yaw instead of subscribing
        DeclareLaunchArgument("yaw_topic", default_value="/robot/yaw"),  # Yaw topic
        DeclareLaunchArgument("map_topic", default_value="/map"),  # Occupancy map topic
        DeclareLaunchArgument(
            "mask_topic",
            default_value="/white_line_skeleton_filter_node/white_final_mask",
        ),  # Topdown white mask topic for PF input
        DeclareLaunchArgument("enable_localization", default_value="true"),  # Enable the localization stack
        DeclareLaunchArgument("publish_debug_pointcloud", default_value="true"),  # Publish localization debug pointcloud
        DeclareLaunchArgument(
            "debug_pointcloud_topic",
            default_value="/field_line_observations_debug",
        ),  # Debug pointcloud topic name
        DeclareLaunchArgument("enable_openmv_camera", default_value="true"),  # Start the OpenMV image publisher
        DeclareLaunchArgument("enable_fastmap_remap", default_value="true"),  # Start the fastmap remap node
        DeclareLaunchArgument("enable_morph_node", default_value="true"),  # Start the LAB morph node
        DeclareLaunchArgument("enable_skeleton_filter", default_value="true"),  # Start the skeleton filter node
        DeclareLaunchArgument(
            "enable_map_server",
            default_value=LaunchConfiguration("enable_localization"),
        ),  # Start the map server when localization is enabled
        DeclareLaunchArgument(
            "enable_lifecycle_manager",
            default_value=LaunchConfiguration("enable_map_server"),
        ),  # Start the lifecycle manager for the map server
        DeclareLaunchArgument(
            "enable_yaw_publisher",
            default_value=LaunchConfiguration("enable_localization"),
        ),  # Start the fake yaw publisher when localization is enabled
        DeclareLaunchArgument(
            "enable_topdown_pf_localization_node_v2",
            default_value="true",
        ),  # Start the V2 topdown particle filter node
        DeclareLaunchArgument("meters_per_pixel", default_value="0.0025"),  # Meters per topdown mask pixel
        DeclareLaunchArgument("forward_axis", default_value="v+"),  # Forward axis mapping
        DeclareLaunchArgument("left_axis", default_value="u-"),  # Left axis mapping
        DeclareLaunchArgument("max_points", default_value="5000"),  # Max sampled mask points
        DeclareLaunchArgument("num_particles", default_value="1000"),  # Particle filter particle count
        DeclareLaunchArgument("sigma_hit", default_value="0.10"),  # Measurement sigma in meters
        DeclareLaunchArgument("noise_xy", default_value="0.05"),  # Position noise std in meters
        DeclareLaunchArgument("noise_theta", default_value="0.10"),  # Heading noise std in radians
        DeclareLaunchArgument("alpha_fast_rate", default_value="0.1"),  # Fast average update rate
        DeclareLaunchArgument("alpha_slow_rate", default_value="0.001"),  # Slow average update rate
        DeclareLaunchArgument(
            "random_injection_max_ratio",
            default_value="0.25",
        ),  # Upper bound for random particle reinjection ratio
        DeclareLaunchArgument("off_map_penalty", default_value="1.0"),  # Off-map penalty in meters
        DeclareLaunchArgument("occupancy_threshold", default_value="50"),  # Map occupancy threshold
        DeclareLaunchArgument(
            "distance_transform_mask_size",
            default_value="5",
        ),  # Distance transform mask size
        DeclareLaunchArgument("init_field_width", default_value="2.0"),  # Initial particle field width in meters
        DeclareLaunchArgument("init_field_height", default_value="3.0"),  # Initial particle field height in meters
        DeclareLaunchArgument("filter_period_ms", default_value="100"),  # PF timer period in milliseconds
        DeclareLaunchArgument(
            "topdown_pf_publish_processing_time", default_value="true"
        ),  # Publish PF processing time
        DeclareLaunchArgument(
            "topdown_pf_processing_time_topic",
            default_value="~/processing_time_ms",
        ),  # PF processing time topic name
        DeclareLaunchArgument(
            "topdown_pf_enable_timing_log", default_value="true"
        ),  # Enable PF timing logs
        DeclareLaunchArgument(
            "topdown_pf_timing_log_interval", default_value="30"
        ),  # PF timing log frame interval
        DeclareLaunchArgument("camera_enable_image_view", default_value="false"),  # Show camera debug window
        DeclareLaunchArgument("camera_show_published_image", default_value="true"),  # Show the camera output image
        DeclareLaunchArgument("remap_enable_image_view", default_value="false"),  # Show remap debug windows
        DeclareLaunchArgument("remap_show_input_image", default_value="false"),  # Show the remap input image
        DeclareLaunchArgument("remap_show_output_image", default_value="true"),  # Show the remap output image
        DeclareLaunchArgument("morph_enable_image_view", default_value="false"),  # Show morph debug windows
        DeclareLaunchArgument("morph_show_input_image", default_value="false"),  # Show the morph input image
        DeclareLaunchArgument("morph_show_white_candidate_mask", default_value="false"),  # Show the white candidate mask
        DeclareLaunchArgument("morph_show_white_morph_mask", default_value="true"),  # Show the morph output mask
        DeclareLaunchArgument("morph_show_green_mask", default_value="false"),  # Show the green mask
        DeclareLaunchArgument("morph_show_black_mask", default_value="false"),  # Show the black mask
        DeclareLaunchArgument("morph_show_noise_mask", default_value="false"),  # Show the noise mask
        DeclareLaunchArgument("morph_show_debug_image", default_value="false"),  # Show the morph debug image
        DeclareLaunchArgument("morph_enable_timing_debug", default_value="false"),  # Enable morph timing logs
        DeclareLaunchArgument("morph_timing_summary_interval", default_value="10"),  # Morph timing summary frame interval
        DeclareLaunchArgument("skeleton_enable_image_view", default_value="true"),  # Show skeleton debug windows
        DeclareLaunchArgument("skeleton_show_morph_mask", default_value="true"),  # Show the skeleton input morph mask
        DeclareLaunchArgument("skeleton_show_green_mask", default_value="false"),  # Show the skeleton green mask
        DeclareLaunchArgument("skeleton_show_black_mask", default_value="false"),  # Show the skeleton black mask
        DeclareLaunchArgument("skeleton_show_noise_mask", default_value="false"),  # Show the skeleton noise mask
        DeclareLaunchArgument("skeleton_show_skeleton_mask", default_value="true"),  # Show the raw skeleton mask
        DeclareLaunchArgument("skeleton_show_orientation_valid_mask", default_value="false"),  # Show the orientation-valid mask
        DeclareLaunchArgument("skeleton_show_side_support_mask", default_value="true"),  # Show the side-support mask
        DeclareLaunchArgument("skeleton_show_width_supported_skeleton_mask", default_value="true"),  # Show the width-filtered skeleton
        DeclareLaunchArgument("skeleton_show_supported_skeleton_mask", default_value="false"),  # Deprecated alias for the length-filtered skeleton view
        DeclareLaunchArgument(
            "skeleton_show_length_filtered_skeleton_mask",
            default_value=LaunchConfiguration("skeleton_show_supported_skeleton_mask"),
        ),  # Show the length-filtered skeleton
        DeclareLaunchArgument("skeleton_show_reconstructed_mask", default_value="false"),  # Show the reconstructed white mask
        DeclareLaunchArgument("skeleton_show_white_final_mask", default_value="true"),  # Show the final white mask
        DeclareLaunchArgument("skeleton_show_white_mask", default_value="__unset__"),  # Deprecated alias for the final white mask view
        DeclareLaunchArgument("skeleton_show_debug_image", default_value="false"),  # Show the skeleton debug mosaic
        DeclareLaunchArgument("skeleton_enable_timing_debug", default_value="false"),  # Enable skeleton timing logs
        DeclareLaunchArgument("skeleton_timing_summary_interval", default_value="10"),  # Skeleton timing summary frame interval
    ]

    return LaunchDescription(
        launch_arguments
        + [
            LogInfo(
                condition=legacy_arg_used_condition("skeleton_show_white_mask"),
                msg=(
                    "Launch argument 'skeleton_show_white_mask' is deprecated. "
                    "Use 'skeleton_show_white_final_mask' instead."
                ),
            ),
            Node(
                package="rcj_localization",
                executable="image_publisher.py",
                name="openmv_camera",
                output="screen",
                condition=enabled_condition("enable_openmv_camera"),
                parameters=[
                    {
                        "port": port,
                        "baudrate": baudrate,
                        "timeout_sec": timeout_sec,
                        "frame_id": frame_id,
                        "image_topic": input_topic,
                        "camera_info_topic": camera_info_topic,
                        "enable_image_view": LaunchConfiguration("camera_enable_image_view"),
                        "show_published_image": LaunchConfiguration("camera_show_published_image"),
                    }
                ],
            ),
            Node(
                package="rcj_localization",
                executable="fastmap_remap_node",
                name="white_line_lab_input_remap_node",
                output="screen",
                condition=enabled_condition("enable_fastmap_remap"),
                parameters=[
                    {
                        "fastmap_file": selected_fastmap_file,
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
                condition=enabled_condition("enable_morph_node"),
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
                condition=enabled_condition("enable_skeleton_filter"),
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
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                condition=enabled_condition("enable_map_server"),
                parameters=[{"yaml_filename": map_yaml_file}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                condition=enabled_condition("enable_lifecycle_manager"),
                parameters=[{"autostart": True, "node_names": ["map_server"]}],
            ),
            Node(
                package="rcj_localization",
                executable="yaw_publisher.py",
                name="yaw_publisher",
                output="screen",
                condition=combined_true_condition(
                    LaunchConfiguration("enable_yaw_publisher"),
                    use_fake_yaw,
                ),
                remappings=[("/robot/yaw", yaw_topic)],
            ),
            Node(
                package="rcj_localization",
                executable="topdown_pf_localization_node_v2",
                name="topdown_pf_localization_node_v2",
                output="screen",
                condition=enabled_condition("enable_topdown_pf_localization_node_v2"),
                parameters=[
                    {
                        "mask_topic": mask_topic,
                        "meters_per_pixel": ParameterValue(
                            LaunchConfiguration("meters_per_pixel"),
                            value_type=float,
                        ),
                        "forward_axis": LaunchConfiguration("forward_axis"),
                        "left_axis": LaunchConfiguration("left_axis"),
                        "max_points": ParameterValue(
                            LaunchConfiguration("max_points"),
                            value_type=int,
                        ),
                        "enable_localization": ParameterValue(
                            enable_localization,
                            value_type=bool,
                        ),
                        "publish_debug_pointcloud": ParameterValue(
                            LaunchConfiguration("publish_debug_pointcloud"),
                            value_type=bool,
                        ),
                        "debug_pointcloud_topic": LaunchConfiguration(
                            "debug_pointcloud_topic"
                        ),
                        "num_particles": ParameterValue(
                            LaunchConfiguration("num_particles"),
                            value_type=int,
                        ),
                        "map_topic": map_topic,
                        "yaw_topic": yaw_topic,
                        "sigma_hit": ParameterValue(
                            LaunchConfiguration("sigma_hit"),
                            value_type=float,
                        ),
                        "noise_xy": ParameterValue(
                            LaunchConfiguration("noise_xy"),
                            value_type=float,
                        ),
                        "noise_theta": ParameterValue(
                            LaunchConfiguration("noise_theta"),
                            value_type=float,
                        ),
                        "alpha_fast_rate": ParameterValue(
                            LaunchConfiguration("alpha_fast_rate"),
                            value_type=float,
                        ),
                        "alpha_slow_rate": ParameterValue(
                            LaunchConfiguration("alpha_slow_rate"),
                            value_type=float,
                        ),
                        "random_injection_max_ratio": ParameterValue(
                            LaunchConfiguration("random_injection_max_ratio"),
                            value_type=float,
                        ),
                        "off_map_penalty": ParameterValue(
                            LaunchConfiguration("off_map_penalty"),
                            value_type=float,
                        ),
                        "occupancy_threshold": ParameterValue(
                            LaunchConfiguration("occupancy_threshold"),
                            value_type=int,
                        ),
                        "distance_transform_mask_size": ParameterValue(
                            LaunchConfiguration("distance_transform_mask_size"),
                            value_type=int,
                        ),
                        "init_field_width": ParameterValue(
                            LaunchConfiguration("init_field_width"),
                            value_type=float,
                        ),
                        "init_field_height": ParameterValue(
                            LaunchConfiguration("init_field_height"),
                            value_type=float,
                        ),
                        "filter_period_ms": ParameterValue(
                            LaunchConfiguration("filter_period_ms"),
                            value_type=int,
                        ),
                        "publish_processing_time": ParameterValue(
                            LaunchConfiguration("topdown_pf_publish_processing_time"),
                            value_type=bool,
                        ),
                        "processing_time_topic": LaunchConfiguration(
                            "topdown_pf_processing_time_topic"
                        ),
                        "enable_timing_log": ParameterValue(
                            LaunchConfiguration("topdown_pf_enable_timing_log"),
                            value_type=bool,
                        ),
                        "timing_log_interval": ParameterValue(
                            LaunchConfiguration("topdown_pf_timing_log_interval"),
                            value_type=int,
                        ),
                    }
                ],
            ),
        ]
    )
