from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = Path(get_package_share_directory("rcj_localization"))
    base_launch_file = (
        package_share / "launch" / "camera_ros_white_line_hsv_dt_ridge_fastmap.launch.py"
    )
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
            DeclareLaunchArgument("camera_index", default_value="0"),  # Camera index
            DeclareLaunchArgument("role", default_value="viewfinder"),  # camera_ros role
            DeclareLaunchArgument("format", default_value="RGB888"),  # Camera pixel format
            DeclareLaunchArgument("width", default_value="800"),  # Capture width
            DeclareLaunchArgument("height", default_value="600"),  # Capture height
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
            DeclareLaunchArgument(
                "robot_mask_path", default_value="/home/rcj/Documents/calibration_images/remapped_mask.png"
            ),  # Optional remapped-space robot mask image path
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
            DeclareLaunchArgument("white_s_max", default_value="118"),  # White HSV maximum S
            DeclareLaunchArgument("white_v_min", default_value="197"),  # White HSV minimum V
            DeclareLaunchArgument("black_v_max", default_value="124"),  # Black HSV maximum V
            DeclareLaunchArgument("green_h_min", default_value="35"),  # Green HSV minimum H
            DeclareLaunchArgument("green_h_max", default_value="100"),  # Green HSV maximum H
            DeclareLaunchArgument("green_s_min", default_value="140"),  # Green HSV minimum S
            DeclareLaunchArgument("green_v_min", default_value="80"),  # Green HSV minimum V
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
                "hsv_enable_controls_window", default_value="true"
            ),  # Whether to show HSV slider controls window
            DeclareLaunchArgument("hsv_show_input_image", default_value="true"),  # Show the HSV input image window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_white_mask", default_value="true"),  # Show the white-priority mask window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_green_mask", default_value="true"),  # Show the green-priority mask window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_black_mask", default_value="true"),  # Show the black-priority mask window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_noise_mask", default_value="true"),  # Show the remaining noise mask window when hsv_enable_image_view is true
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
                "ridge_enable_orientation_estimate", default_value="true"
            ),  # Whether to run orientation estimation before side support
            DeclareLaunchArgument(
                "ridge_enable_parallel_orientation_estimate", default_value="true"
            ),  # Whether to parallelize seed-only orientation estimation
            DeclareLaunchArgument(
                "ridge_enable_length_filter", default_value="false"
            ),  # Whether to run connected-component length filtering
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
            ),  # Whether to show orientation-valid seed mask
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
                "ridge_enable_timing_debug", default_value="true"
            ),  # Whether to log ridge timing summary
            DeclareLaunchArgument(
                "ridge_timing_summary_interval", default_value="10"
            ),  # Ridge timing summary frame interval
            DeclareLaunchArgument(
                "map_yaml_file", default_value=str(map_yaml_default)
            ),  # Nav2 map YAML path
            DeclareLaunchArgument("use_fake_yaw", default_value="false"),  # Whether to use synthetic yaw
            DeclareLaunchArgument("yaw_topic", default_value="/robot/yaw"),  # Robot yaw topic
            DeclareLaunchArgument(
                "yaw_enable_publish_log", default_value="false"
            ),  # Whether to log yaw publisher output
            DeclareLaunchArgument("map_topic", default_value="/map"),  # Occupancy grid topic
            DeclareLaunchArgument(
                "enable_localization", default_value="true"
            ),  # Whether to enable PF localization logic
            DeclareLaunchArgument(
                "enable_map_server", default_value=enable_localization
            ),  # Whether to start Nav2 map server
            DeclareLaunchArgument(
                "enable_lifecycle_manager", default_value=enable_map_server
            ),  # Whether to start Nav2 lifecycle manager
            DeclareLaunchArgument(
                "enable_yaw_publisher", default_value=enable_localization
            ),  # Whether to start yaw publisher
            DeclareLaunchArgument(
                "enable_topdown_pf_localization_node_v2",
                default_value="true",
            ),  # Whether to start topdown PF node
            DeclareLaunchArgument("meters_per_pixel", default_value="0.0036"),  # Camera projection scale
            DeclareLaunchArgument("forward_axis", default_value="v+"),  # Image axis treated as robot forward
            DeclareLaunchArgument("left_axis", default_value="u-"),  # Image axis treated as robot left
            DeclareLaunchArgument("max_points", default_value="3000"),  # Maximum observation points per frame
            DeclareLaunchArgument(
                "publish_debug_pointcloud", default_value="true"
            ),  # Whether to publish debug point cloud
            DeclareLaunchArgument(
                "debug_pointcloud_topic",
                default_value="/field_line_observations_debug",
            ),  # Debug point cloud topic
            DeclareLaunchArgument("num_particles", default_value="1000"),  # Number of particles
            DeclareLaunchArgument("sigma_hit", default_value="0.10"),  # Likelihood-field sigma
            DeclareLaunchArgument("noise_xy", default_value="0.05"),  # XY motion noise
            DeclareLaunchArgument("noise_theta", default_value="0.10"),  # Heading motion noise
            DeclareLaunchArgument("alpha_fast_rate", default_value="0.1"),  # Fast weight adaptation rate
            DeclareLaunchArgument("alpha_slow_rate", default_value="0.001"),  # Slow weight adaptation rate
            DeclareLaunchArgument(
                "random_injection_max_ratio", default_value="0.25"
            ),  # Maximum random particle injection ratio
            DeclareLaunchArgument("off_map_penalty", default_value="1.0"),  # Penalty for off-map particles
            DeclareLaunchArgument("occupancy_threshold", default_value="50"),  # Occupancy threshold for map queries
            DeclareLaunchArgument(
                "distance_transform_mask_size", default_value="5"
            ),  # Distance transform mask size
            DeclareLaunchArgument("init_field_width", default_value="2.0"),  # Initial particle field width
            DeclareLaunchArgument("init_field_height", default_value="3.0"),  # Initial particle field height
            DeclareLaunchArgument("filter_period_ms", default_value="80"),  # PF update period in milliseconds
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
            ),  # PF timing log frame interval
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(base_launch_file)),
                launch_arguments={
                    "camera_index": camera_index,
                    "role": role,
                    "format": image_format,
                    "width": width,
                    "height": height,
                    "orientation": orientation,
                    "sensor_mode": sensor_mode,
                    "frame_id": frame_id,
                    "camera_info_url": camera_info_url,
                    "use_node_time": use_node_time,
                    "camera_info_topic": camera_info_topic,
                    "input_topic": input_topic,
                    "remap_topic": remap_topic,
                    "white_mask_topic": white_mask_topic,
                    "robot_mask_path": robot_mask_path,
                    "use_latest_fastmap": LaunchConfiguration("use_latest_fastmap"),
                    "fastmap_file": fastmap_file,
                    "input_transport": input_transport,
                    "interpolation": interpolation,
                    "remap_enable_image_view": LaunchConfiguration(
                        "remap_enable_image_view"
                    ),
                    "remap_enable_timing_log": LaunchConfiguration(
                        "remap_enable_timing_log"
                    ),
                    "remap_timing_log_interval": LaunchConfiguration(
                        "remap_timing_log_interval"
                    ),
                    "white_h_min": LaunchConfiguration("white_h_min"),
                    "white_h_max": LaunchConfiguration("white_h_max"),
                    "white_s_max": LaunchConfiguration("white_s_max"),
                    "white_v_min": LaunchConfiguration("white_v_min"),
                    "black_v_max": LaunchConfiguration("black_v_max"),
                    "green_h_min": LaunchConfiguration("green_h_min"),
                    "green_h_max": LaunchConfiguration("green_h_max"),
                    "green_s_min": LaunchConfiguration("green_s_min"),
                    "green_v_min": LaunchConfiguration("green_v_min"),
                    "hsv_enable_timing_log": LaunchConfiguration(
                        "hsv_enable_timing_log"
                    ),
                    "hsv_timing_log_interval": LaunchConfiguration(
                        "hsv_timing_log_interval"
                    ),
                    "hsv_enable_image_view": LaunchConfiguration(
                        "hsv_enable_image_view"
                    ),
                    "hsv_enable_controls_window": LaunchConfiguration(
                        "hsv_enable_controls_window"
                    ),
                    "hsv_show_input_image": LaunchConfiguration(
                        "hsv_show_input_image"
                    ),
                    "hsv_show_white_mask": LaunchConfiguration(
                        "hsv_show_white_mask"
                    ),
                    "hsv_show_green_mask": LaunchConfiguration(
                        "hsv_show_green_mask"
                    ),
                    "hsv_show_black_mask": LaunchConfiguration(
                        "hsv_show_black_mask"
                    ),
                    "hsv_show_noise_mask": LaunchConfiguration(
                        "hsv_show_noise_mask"
                    ),
                    "hsv_show_overlay_image": LaunchConfiguration(
                        "hsv_show_overlay_image"
                    ),
                    "hsv_display_max_width": LaunchConfiguration(
                        "hsv_display_max_width"
                    ),
                    "hsv_display_max_height": LaunchConfiguration(
                        "hsv_display_max_height"
                    ),
                    "ridge_orientation_window_radius_px": LaunchConfiguration(
                        "ridge_orientation_window_radius_px"
                    ),
                    "ridge_min_orientation_neighbors": LaunchConfiguration(
                        "ridge_min_orientation_neighbors"
                    ),
                    "ridge_enable_orientation_estimate": LaunchConfiguration(
                        "ridge_enable_orientation_estimate"
                    ),
                    "ridge_enable_parallel_orientation_estimate": LaunchConfiguration(
                        "ridge_enable_parallel_orientation_estimate"
                    ),
                    "ridge_side_margin_px": LaunchConfiguration(
                        "ridge_side_margin_px"
                    ),
                    "ridge_side_band_depth_px": LaunchConfiguration(
                        "ridge_side_band_depth_px"
                    ),
                    "ridge_min_green_ratio": LaunchConfiguration(
                        "ridge_min_green_ratio"
                    ),
                    "ridge_min_boundary_ratio": LaunchConfiguration(
                        "ridge_min_boundary_ratio"
                    ),
                    "ridge_enable_boundary_mode": LaunchConfiguration(
                        "ridge_enable_boundary_mode"
                    ),
                    "ridge_width_floor_px": LaunchConfiguration(
                        "ridge_width_floor_px"
                    ),
                    "ridge_width_ceil_px": LaunchConfiguration(
                        "ridge_width_ceil_px"
                    ),
                    "ridge_width_mad_scale": LaunchConfiguration(
                        "ridge_width_mad_scale"
                    ),
                    "ridge_min_width_samples": LaunchConfiguration(
                        "ridge_min_width_samples"
                    ),
                    "ridge_enable_candidate_prefilter": LaunchConfiguration(
                        "ridge_enable_candidate_prefilter"
                    ),
                    "ridge_candidate_min_component_px": LaunchConfiguration(
                        "ridge_candidate_min_component_px"
                    ),
                    "ridge_candidate_prune_rounds": LaunchConfiguration(
                        "ridge_candidate_prune_rounds"
                    ),
                    "ridge_side_scan_stride": LaunchConfiguration(
                        "ridge_side_scan_stride"
                    ),
                    "ridge_side_template_direction_bins": LaunchConfiguration(
                        "ridge_side_template_direction_bins"
                    ),
                    "ridge_enable_parallel_side_scan": LaunchConfiguration(
                        "ridge_enable_parallel_side_scan"
                    ),
                    "ridge_enable_length_filter": LaunchConfiguration(
                        "ridge_enable_length_filter"
                    ),
                    "ridge_min_skeleton_length_px": LaunchConfiguration(
                        "ridge_min_skeleton_length_px"
                    ),
                    "ridge_reconstruction_margin_px": LaunchConfiguration(
                        "ridge_reconstruction_margin_px"
                    ),
                    "ridge_enable_image_view": LaunchConfiguration(
                        "ridge_enable_image_view"
                    ),
                    "ridge_show_morph_mask": LaunchConfiguration(
                        "ridge_show_morph_mask"
                    ),
                    "ridge_show_distance_transform": LaunchConfiguration(
                        "ridge_show_distance_transform"
                    ),
                    "ridge_show_green_mask": LaunchConfiguration(
                        "ridge_show_green_mask"
                    ),
                    "ridge_show_black_mask": LaunchConfiguration(
                        "ridge_show_black_mask"
                    ),
                    "ridge_show_noise_mask": LaunchConfiguration(
                        "ridge_show_noise_mask"
                    ),
                    "ridge_show_ridge_mask": LaunchConfiguration(
                        "ridge_show_ridge_mask"
                    ),
                    "ridge_show_candidate_prefilter_mask": LaunchConfiguration(
                        "ridge_show_candidate_prefilter_mask"
                    ),
                    "ridge_show_orientation_valid_mask": LaunchConfiguration(
                        "ridge_show_orientation_valid_mask"
                    ),
                    "ridge_show_side_support_seed_mask": LaunchConfiguration(
                        "ridge_show_side_support_seed_mask"
                    ),
                    "ridge_show_side_support_mask": LaunchConfiguration(
                        "ridge_show_side_support_mask"
                    ),
                    "ridge_show_width_supported_ridge_mask": LaunchConfiguration(
                        "ridge_show_width_supported_ridge_mask"
                    ),
                    "ridge_show_length_filtered_ridge_mask": LaunchConfiguration(
                        "ridge_show_length_filtered_ridge_mask"
                    ),
                    "ridge_show_reconstructed_mask": LaunchConfiguration(
                        "ridge_show_reconstructed_mask"
                    ),
                    "ridge_show_white_final_mask": LaunchConfiguration(
                        "ridge_show_white_final_mask"
                    ),
                    "ridge_show_debug_image": LaunchConfiguration(
                        "ridge_show_debug_image"
                    ),
                    "ridge_enable_timing_debug": LaunchConfiguration(
                        "ridge_enable_timing_debug"
                    ),
                    "ridge_timing_summary_interval": LaunchConfiguration(
                        "ridge_timing_summary_interval"
                    ),
                }.items(),
            ),
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                condition=IfCondition(enable_map_server),
                parameters=[{"yaml_filename": map_yaml_file}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                condition=IfCondition(enable_lifecycle_manager),
                parameters=[
                    {
                        "autostart": True,
                        "node_names": ["map_server"],
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
                        ),  # Whether to log yaw publisher output
                    }
                ],
                remappings=[("/robot/yaw", yaw_topic)],
            ),
            Node(
                package="rcj_localization",
                executable="topdown_pf_localization_node_v2",
                name="topdown_pf_localization_node_v2",
                output="screen",
                condition=IfCondition(enable_topdown_pf_localization_node_v2),
                parameters=[
                    {
                        "mask_topic": "/white_line_dt_ridge_filter_node/white_final_mask",  # PF input white mask topic
                        "meters_per_pixel": ParameterValue(
                            meters_per_pixel, value_type=float
                        ),  # Camera projection scale
                        "forward_axis": forward_axis,  # Image axis treated as robot forward
                        "left_axis": left_axis,  # Image axis treated as robot left
                        "max_points": ParameterValue(
                            max_points, value_type=int
                        ),  # Maximum observation points per frame
                        "enable_localization": ParameterValue(
                            enable_localization, value_type=bool
                        ),  # Whether to enable PF localization logic
                        "publish_debug_pointcloud": ParameterValue(
                            publish_debug_pointcloud, value_type=bool
                        ),  # Whether to publish debug point cloud
                        "debug_pointcloud_topic": debug_pointcloud_topic,  # Debug point cloud topic
                        "num_particles": ParameterValue(
                            num_particles, value_type=int
                        ),  # Number of particles
                        "map_topic": map_topic,  # Occupancy grid topic
                        "yaw_topic": yaw_topic,  # Robot yaw topic
                        "sigma_hit": ParameterValue(
                            sigma_hit, value_type=float
                        ),  # Likelihood-field sigma
                        "noise_xy": ParameterValue(
                            noise_xy, value_type=float
                        ),  # XY motion noise
                        "noise_theta": ParameterValue(
                            noise_theta, value_type=float
                        ),  # Heading motion noise
                        "alpha_fast_rate": ParameterValue(
                            alpha_fast_rate, value_type=float
                        ),  # Fast weight adaptation rate
                        "alpha_slow_rate": ParameterValue(
                            alpha_slow_rate, value_type=float
                        ),  # Slow weight adaptation rate
                        "random_injection_max_ratio": ParameterValue(
                            random_injection_max_ratio, value_type=float
                        ),  # Maximum random particle injection ratio
                        "off_map_penalty": ParameterValue(
                            off_map_penalty, value_type=float
                        ),  # Penalty for off-map particles
                        "occupancy_threshold": ParameterValue(
                            occupancy_threshold, value_type=int
                        ),  # Occupancy threshold for map queries
                        "distance_transform_mask_size": ParameterValue(
                            distance_transform_mask_size, value_type=int
                        ),  # Distance transform mask size
                        "init_field_width": ParameterValue(
                            init_field_width, value_type=float
                        ),  # Initial particle field width
                        "init_field_height": ParameterValue(
                            init_field_height, value_type=float
                        ),  # Initial particle field height
                        "filter_period_ms": ParameterValue(
                            filter_period_ms, value_type=int
                        ),  # PF update period in milliseconds
                        "publish_processing_time": ParameterValue(
                            topdown_pf_publish_processing_time, value_type=bool
                        ),  # Whether to publish PF processing time
                        "processing_time_topic": topdown_pf_processing_time_topic,  # PF processing time topic
                        "enable_timing_log": ParameterValue(
                            topdown_pf_enable_timing_log, value_type=bool
                        ),  # Whether to log PF timing
                        "timing_log_interval": ParameterValue(
                            topdown_pf_timing_log_interval, value_type=int
                        ),  # PF timing log frame interval
                    }
                ],
            ),
        ]
    )
