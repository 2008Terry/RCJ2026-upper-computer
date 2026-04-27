from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, str(Path(__file__).resolve().parent))
from rcj_fastmap_hsv_pipeline import build_camera_hsv_dt_ridge_fastmap_nodes
from rcj_shared_launch_params import (
    camera_control_launch_arguments,
    camera_control_parameters,
    declare_camera_control_arguments,
    declare_camera_ros_arguments,
    declare_hsv_green_white_black_arguments,
    hsv_green_white_black_launch_arguments,
    hsv_green_white_black_parameters,
)


def generate_launch_description():
    package_share = Path(get_package_share_directory("rcj_localization"))
    stm32_gateway_launch_file = (
        package_share / "launch" / "stm32_serial_gateway.launch.py"
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
    fake_yaw_degrees = LaunchConfiguration("fake_yaw_degrees")
    yaw_zero_map_degrees = LaunchConfiguration("yaw_zero_map_degrees")
    map_topic = LaunchConfiguration("map_topic")
    enable_localization = LaunchConfiguration("enable_localization")
    enable_map_server = LaunchConfiguration("enable_map_server")
    enable_lifecycle_manager = LaunchConfiguration("enable_lifecycle_manager")
    enable_topdown_pf_localization_node_v2 = LaunchConfiguration(
        "enable_topdown_pf_localization_node_v2"
    )
    odom_topic = LaunchConfiguration("odom_topic")
    use_stm32_gateway_odometry = LaunchConfiguration("use_stm32_gateway_odometry")
    stm32_command_service = LaunchConfiguration("stm32_command_service")
    stm32_request_timeout_ms = LaunchConfiguration("stm32_request_timeout_ms")
    stm32_enable_odometry_log = LaunchConfiguration("stm32_enable_odometry_log")
    use_random_search_when_unlocalized = LaunchConfiguration(
        "use_random_search_when_unlocalized"
    )
    enable_global_search = LaunchConfiguration("enable_global_search")
    global_search_random_ratio = LaunchConfiguration("global_search_random_ratio")
    global_search_noise_xy = LaunchConfiguration("global_search_noise_xy")
    global_search_noise_theta = LaunchConfiguration("global_search_noise_theta")
    localized_xy_std_threshold = LaunchConfiguration("localized_xy_std_threshold")
    localized_theta_std_threshold = LaunchConfiguration(
        "localized_theta_std_threshold"
    )
    localized_min_updates = LaunchConfiguration("localized_min_updates")
    lost_alpha_ratio_threshold = LaunchConfiguration("lost_alpha_ratio_threshold")
    lost_min_updates = LaunchConfiguration("lost_min_updates")
    stm32_port = LaunchConfiguration("stm32_port")
    stm32_baudrate = LaunchConfiguration("stm32_baudrate")
    stm32_tick_period_ms = LaunchConfiguration("stm32_tick_period_ms")
    stm32_resend_period_ms = LaunchConfiguration("stm32_resend_period_ms")
    stm32_command_timeout_ms = LaunchConfiguration("stm32_command_timeout_ms")
    stm32_motion_timeout_ms = LaunchConfiguration("stm32_motion_timeout_ms")
    stm32_max_queue_size = LaunchConfiguration("stm32_max_queue_size")
    stm32_enable_serial_log = LaunchConfiguration("stm32_enable_serial_log")
    stm32_enable_raw_reply_log = LaunchConfiguration("stm32_enable_raw_reply_log")
    meters_per_pixel = LaunchConfiguration("meters_per_pixel")
    forward_axis = LaunchConfiguration("forward_axis")
    left_axis = LaunchConfiguration("left_axis")
    max_points = LaunchConfiguration("max_points")
    use_weighted_mean_pose = LaunchConfiguration("use_weighted_mean_pose")
    publish_debug_pointcloud = LaunchConfiguration("publish_debug_pointcloud")
    debug_pointcloud_topic = LaunchConfiguration("debug_pointcloud_topic")
    publish_particle_weight_markers = LaunchConfiguration(
        "publish_particle_weight_markers"
    )
    particle_weight_marker_topic = LaunchConfiguration(
        "particle_weight_marker_topic"
    )
    particle_weight_marker_scale = LaunchConfiguration(
        "particle_weight_marker_scale"
    )
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
    odom_noise_x_from_x = LaunchConfiguration("odom_noise_x_from_x")
    odom_noise_x_from_y = LaunchConfiguration("odom_noise_x_from_y")
    odom_noise_x_from_theta = LaunchConfiguration("odom_noise_x_from_theta")
    odom_noise_x_bias = LaunchConfiguration("odom_noise_x_bias")
    odom_noise_y_from_x = LaunchConfiguration("odom_noise_y_from_x")
    odom_noise_y_from_y = LaunchConfiguration("odom_noise_y_from_y")
    odom_noise_y_from_theta = LaunchConfiguration("odom_noise_y_from_theta")
    odom_noise_y_bias = LaunchConfiguration("odom_noise_y_bias")
    odom_noise_theta_from_x = LaunchConfiguration("odom_noise_theta_from_x")
    odom_noise_theta_from_y = LaunchConfiguration("odom_noise_theta_from_y")
    odom_noise_theta_from_theta = LaunchConfiguration("odom_noise_theta_from_theta")
    odom_noise_theta_bias = LaunchConfiguration("odom_noise_theta_bias")
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
            DeclareLaunchArgument("use_latest_fastmap", default_value="false"),  # Whether to auto-select the latest Fastmap XML
            DeclareLaunchArgument(
                "fastmap_file", default_value=""
            ),  # Specific Fastmap XML path when auto-select is disabled
            DeclareLaunchArgument("input_transport", default_value="raw"),  # Remap input transport
            DeclareLaunchArgument("interpolation", default_value="linear"),  # Remap interpolation mode
            DeclareLaunchArgument(
                "remap_enable_image_view", default_value="false"
            ),  # Whether to show remap windows
            DeclareLaunchArgument("remap_show_input_image", default_value="true"),  # Show remap input window when remap image_view is true
            DeclareLaunchArgument("remap_show_output_image", default_value="true"),  # Show remap output window when remap image_view is true
            DeclareLaunchArgument(
                "remap_publish_debug_images", default_value="false"
            ),  # Master switch for remap debug image topics
            DeclareLaunchArgument("remap_publish_input_image", default_value="true"),  # Publish remap input debug topic when subscribed
            DeclareLaunchArgument("remap_publish_output_image", default_value="true"),  # Publish remap output debug topic when subscribed
            DeclareLaunchArgument(
                "remap_enable_timing_log", default_value="false"
            ),  # Whether to log remap timing
            DeclareLaunchArgument(
                "remap_timing_log_interval", default_value="30"
            ),  # Remap timing log frame interval
            *declare_hsv_green_white_black_arguments(),
            DeclareLaunchArgument(
                "hsv_enable_timing_log", default_value="false"
            ),  # Whether to log HSV timing
            DeclareLaunchArgument(
                "hsv_timing_log_interval", default_value="15"
            ),  # HSV timing log frame interval
            DeclareLaunchArgument(
                "hsv_enable_image_view", default_value="false"
            ),  # Master switch for HSV debug windows; false means no window creation or GUI processing
            DeclareLaunchArgument(
                "hsv_enable_controls_window", default_value="false"
            ),  # Whether to show HSV slider controls window
            DeclareLaunchArgument("hsv_show_input_image", default_value="true"),  # Show the HSV input image window when hsv_enable_image_view is true
            DeclareLaunchArgument("hsv_show_white_mask", default_value="false"),  # Show the white-priority mask window when hsv_enable_image_view is true
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
                "ridge_enable_image_view", default_value="false"
            ),  # Whether to show ridge debug windows
            DeclareLaunchArgument("ridge_show_morph_mask", default_value="false"),  # Whether to show input white mask
            DeclareLaunchArgument("ridge_show_distance_transform", default_value="false"),  # Whether to show the DT image before orientation filtering
            DeclareLaunchArgument("ridge_show_green_mask", default_value="false"),  # Whether to show input green mask
            DeclareLaunchArgument("ridge_show_black_mask", default_value="false"),  # Whether to show input black mask
            DeclareLaunchArgument("ridge_show_noise_mask", default_value="false"),  # Whether to show input noise mask
            DeclareLaunchArgument("ridge_show_ridge_mask", default_value="false"),  # Whether to show extracted ridge mask
            DeclareLaunchArgument(
                "ridge_show_candidate_prefilter_mask", default_value="false"
            ),  # Whether to show candidate-prefilter ridge mask
            DeclareLaunchArgument(
                "ridge_show_orientation_valid_mask", default_value="false"
            ),  # Whether to show orientation-valid seed mask
            DeclareLaunchArgument(
                "ridge_show_side_support_seed_mask", default_value="false"
            ),  # Whether to show supported side-scan seed mask
            DeclareLaunchArgument(
                "ridge_show_side_support_mask", default_value="false"
            ),  # Whether to show side-support mask
            DeclareLaunchArgument(
                "ridge_show_width_supported_ridge_mask", default_value="false"
            ),  # Whether to show width-filtered ridge mask
            DeclareLaunchArgument(
                "ridge_show_length_filtered_ridge_mask", default_value="false"
            ),  # Whether to show length-filtered ridge mask
            DeclareLaunchArgument(
                "ridge_show_reconstructed_mask", default_value="false"
            ),  # Whether to show reconstructed mask
            DeclareLaunchArgument(
                "ridge_show_white_final_mask", default_value="false"
            ),  # Whether to show final white mask
            DeclareLaunchArgument("ridge_show_debug_image", default_value="true"),  # Whether to show composite debug image
            DeclareLaunchArgument(
                "ridge_publish_debug_images", default_value="true"
            ),  # Master switch for ridge debug image topics
            DeclareLaunchArgument("ridge_publish_morph_mask", default_value="false"),  # Publish ridge input morph mask when subscribed
            DeclareLaunchArgument("ridge_publish_distance_transform", default_value="false"),  # Publish ridge distance-transform debug image when subscribed
            DeclareLaunchArgument("ridge_publish_green_mask", default_value="false"),  # Publish ridge input green mask when subscribed
            DeclareLaunchArgument("ridge_publish_black_mask", default_value="false"),  # Publish ridge input black mask when subscribed
            DeclareLaunchArgument("ridge_publish_noise_mask", default_value="false"),  # Publish ridge input noise mask when subscribed
            DeclareLaunchArgument("ridge_publish_ridge_mask", default_value="false"),  # Publish extracted ridge mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_candidate_prefilter_mask", default_value="false"
            ),  # Publish candidate-prefilter ridge mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_orientation_valid_mask", default_value="false"
            ),  # Publish orientation-valid seed mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_side_support_seed_mask", default_value="false"
            ),  # Publish supported side-scan seed mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_side_support_mask", default_value="false"
            ),  # Publish side-support mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_width_supported_ridge_mask", default_value="false"
            ),  # Publish width-filtered ridge mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_length_filtered_ridge_mask", default_value="false"
            ),  # Publish length-filtered ridge mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_reconstructed_mask", default_value="false"
            ),  # Publish reconstructed mask when subscribed
            DeclareLaunchArgument(
                "ridge_publish_white_final_mask", default_value="false"
            ),  # Keep final white mask topic available when subscribed
            DeclareLaunchArgument("ridge_publish_debug_image", default_value="true"),  # Publish composite debug image when subscribed
            DeclareLaunchArgument(
                "ridge_enable_timing_log", default_value="false"
            ),  # Whether to log ridge timing summary
            DeclareLaunchArgument(
                "ridge_timing_summary_interval", default_value="10"
            ),  # Ridge timing summary frame interval
            DeclareLaunchArgument(
                "map_yaml_file", default_value=str(map_yaml_default)
            ),  # Nav2 map YAML path
            DeclareLaunchArgument("use_fake_yaw", default_value="true"),  # Whether to use synthetic yaw
            DeclareLaunchArgument("yaw_topic", default_value="/robot/yaw"),  # Robot yaw topic
            DeclareLaunchArgument(
                "fake_yaw_degrees", default_value="0.0"
            ),  # Fixed yaw angle used when use_fake_yaw is true
            DeclareLaunchArgument(
                "yaw_zero_map_degrees", default_value="0.0"
            ),  # Field-heading offset for robot yaw 0; 0 means the top side of the field
            DeclareLaunchArgument("odom_topic", default_value="/wheel_odometry"),  # Wheel odometry topic
            DeclareLaunchArgument(
                "use_stm32_gateway_odometry", default_value="true"
            ),  # Whether AMCL requests odometry from the STM32 gateway
            DeclareLaunchArgument(
                "stm32_command_service", default_value="/stm32/send_command"
            ),  # STM32 gateway service name
            DeclareLaunchArgument(
                "stm32_request_timeout_ms", default_value="50"
            ),  # Local timeout for one async STM32 odometry request
            DeclareLaunchArgument(
                "stm32_enable_odometry_log", default_value="true"
            ),  # Whether AMCL logs each STM32 odometry dx/dy/dtheta response
            DeclareLaunchArgument(
                "use_random_search_when_unlocalized", default_value="false"
            ),  # Whether unlocalized/lost AMCL uses random diffusion and random particle injection
            DeclareLaunchArgument(
                "enable_global_search",
                default_value=use_random_search_when_unlocalized,
            ),  # Backward-compatible alias passed to amcl_fusion
            DeclareLaunchArgument(
                "global_search_random_ratio", default_value="0.50"
            ),  # Fraction of particles randomly injected while globally searching
            DeclareLaunchArgument(
                "global_search_noise_xy", default_value="0.12"
            ),  # XY diffusion used while globally searching
            DeclareLaunchArgument(
                "global_search_noise_theta", default_value="0.10"
            ),  # Heading diffusion used while globally searching
            DeclareLaunchArgument(
                "localized_xy_std_threshold", default_value="0.20"
            ),  # Weighted particle XY spread below this can exit global search
            DeclareLaunchArgument(
                "localized_theta_std_threshold", default_value="0.35"
            ),  # Weighted particle heading spread below this can exit global search
            DeclareLaunchArgument(
                "localized_min_updates", default_value="5"
            ),  # Consecutive concentrated updates required before using odometry
            DeclareLaunchArgument(
                "lost_alpha_ratio_threshold", default_value="0.45"
            ),  # Alpha-fast/alpha-slow ratio below this re-enters global search
            DeclareLaunchArgument(
                "lost_min_updates", default_value="3"
            ),  # Consecutive low alpha-ratio updates required to mark lost
            DeclareLaunchArgument("stm32_port", default_value="/dev/ttyUSB0"),  # STM32 serial port
            DeclareLaunchArgument("stm32_baudrate", default_value="115200"),  # STM32 serial baudrate
            DeclareLaunchArgument(
                "stm32_tick_period_ms", default_value="10"
            ),  # STM32 gateway tick period
            DeclareLaunchArgument(
                "stm32_resend_period_ms", default_value="20"
            ),  # Deprecated compatibility parameter; gateway commands are sent once
            DeclareLaunchArgument(
                "stm32_command_timeout_ms", default_value="30"
            ),  # STM32 gateway command timeout
            DeclareLaunchArgument(
                "stm32_motion_timeout_ms", default_value="5000"
            ),  # STM32 cmd_dis/cmd_turn completion ACK timeout
            DeclareLaunchArgument(
                "stm32_max_queue_size", default_value="32"
            ),  # STM32 gateway queue capacity
            DeclareLaunchArgument(
                "stm32_enable_serial_log", default_value="true"
            ),  # Whether the STM32 gateway logs command summaries
            DeclareLaunchArgument(
                "stm32_enable_raw_reply_log", default_value="true"
            ),  # Whether the STM32 gateway logs raw serial replies
            DeclareLaunchArgument(
                "yaw_enable_publish_log", default_value="false"
            ),  # Deprecated: fake yaw is handled inside the localization node
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
            ),  # Deprecated: fake yaw is handled inside the localization node
            DeclareLaunchArgument(
                "enable_topdown_pf_localization_node_v2",
                default_value="true",
            ),  # Whether to start AMCL fusion node
            DeclareLaunchArgument("meters_per_pixel", default_value="0.0034"),  # Camera projection scale
            DeclareLaunchArgument("forward_axis", default_value="v-"),  # Image axis treated as robot forward
            DeclareLaunchArgument("left_axis", default_value="u-"),  # Image axis treated as robot left
            DeclareLaunchArgument("max_points", default_value="3000"),  # Maximum observation points per frame
            DeclareLaunchArgument(
                "use_weighted_mean_pose", default_value="true"
            ),  # Use weighted mean pose instead of the highest-weight particle
            DeclareLaunchArgument(
                "publish_debug_pointcloud", default_value="true"
            ),  # Whether to publish debug point cloud
            DeclareLaunchArgument(
                "debug_pointcloud_topic",
                default_value="/field_line_observations_debug",
            ),  # Debug point cloud topic
            DeclareLaunchArgument(
                "publish_particle_weight_markers", default_value="true"
            ),  # Whether to publish RViz Marker particles colored by weight
            DeclareLaunchArgument(
                "particle_weight_marker_topic",
                default_value="/particle_weights",
            ),  # Weighted particle Marker topic
            DeclareLaunchArgument(
                "particle_weight_marker_scale", default_value="0.01"
            ),  # Weighted particle Marker point size in meters
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
            DeclareLaunchArgument("odom_noise_x_from_x", default_value="0.06"),  # Forward motion contribution to x variance
            DeclareLaunchArgument("odom_noise_x_from_y", default_value="0.01"),  # Lateral motion contribution to x variance
            DeclareLaunchArgument("odom_noise_x_from_theta", default_value="0.0025"),  # Rotation contribution to x variance
            DeclareLaunchArgument("odom_noise_x_bias", default_value="0.0001"),  # Constant x variance term
            DeclareLaunchArgument("odom_noise_y_from_x", default_value="0.01"),  # Forward motion contribution to y variance
            DeclareLaunchArgument("odom_noise_y_from_y", default_value="0.06"),  # Lateral motion contribution to y variance
            DeclareLaunchArgument("odom_noise_y_from_theta", default_value="0.0025"),  # Rotation contribution to y variance
            DeclareLaunchArgument("odom_noise_y_bias", default_value="0.0001"),  # Constant y variance term
            DeclareLaunchArgument("odom_noise_theta_from_x", default_value="0.30"),  # Forward motion contribution to heading variance
            DeclareLaunchArgument("odom_noise_theta_from_y", default_value="0.60"),  # Lateral motion contribution to heading variance
            DeclareLaunchArgument("odom_noise_theta_from_theta", default_value="0.09"),  # Rotation contribution to heading variance
            DeclareLaunchArgument("odom_noise_theta_bias", default_value="0.000304617"),  # Constant heading variance term
            DeclareLaunchArgument("filter_period_ms", default_value="80"),  # PF update period in milliseconds
            DeclareLaunchArgument(
                "topdown_pf_publish_processing_time", default_value="true"
            ),  # Whether to publish PF processing time
            DeclareLaunchArgument(
                "topdown_pf_processing_time_topic",
                default_value="~/processing_time_ms",
            ),  # PF processing time topic
            DeclareLaunchArgument(
                "topdown_pf_enable_timing_log", default_value="false"
            ),  # Whether to log PF timing
            DeclareLaunchArgument(
                "topdown_pf_timing_log_interval", default_value="10"
            ),  # PF timing log frame interval
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(stm32_gateway_launch_file)),
                condition=IfCondition(use_stm32_gateway_odometry),
                launch_arguments={
                    "port": stm32_port,
                    "baudrate": stm32_baudrate,
                    "tick_period_ms": stm32_tick_period_ms,
                    "resend_period_ms": stm32_resend_period_ms,
                    "command_timeout_ms": stm32_command_timeout_ms,
                    "motion_timeout_ms": stm32_motion_timeout_ms,
                    "max_queue_size": stm32_max_queue_size,
                    "enable_serial_log": stm32_enable_serial_log,
                    "enable_raw_reply_log": stm32_enable_raw_reply_log,
                }.items(),
            ),
            OpaqueFunction(function=build_camera_hsv_dt_ridge_fastmap_nodes),
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
                executable="amcl_fusion",
                name="amcl_fusion",
                output="screen",
                condition=IfCondition(enable_topdown_pf_localization_node_v2),
                parameters=[
                    {
                        "mask_topic": "/white_line_dt_ridge_filter_node/white_final_mask",
                        "meters_per_pixel": ParameterValue(
                            meters_per_pixel, value_type=float
                        ),
                        "forward_axis": forward_axis,
                        "left_axis": left_axis,
                        "max_points": ParameterValue(
                            max_points, value_type=int
                        ),
                        "use_weighted_mean_pose": ParameterValue(
                            use_weighted_mean_pose, value_type=bool
                        ),
                        "enable_localization": ParameterValue(
                            enable_localization, value_type=bool
                        ),
                        "publish_debug_pointcloud": ParameterValue(
                            publish_debug_pointcloud, value_type=bool
                        ),
                        "debug_pointcloud_topic": debug_pointcloud_topic,
                        "publish_particle_weight_markers": ParameterValue(
                            publish_particle_weight_markers, value_type=bool
                        ),
                        "particle_weight_marker_topic": particle_weight_marker_topic,
                        "particle_weight_marker_scale": ParameterValue(
                            particle_weight_marker_scale, value_type=float
                        ),
                        "num_particles": ParameterValue(
                            num_particles, value_type=int
                        ),
                        "map_topic": map_topic,
                        "yaw_topic": yaw_topic,
                        "use_fake_yaw": ParameterValue(
                            use_fake_yaw, value_type=bool
                        ),
                        "fake_yaw_degrees": ParameterValue(
                            fake_yaw_degrees, value_type=float
                        ),
                        "yaw_zero_map_degrees": ParameterValue(
                            yaw_zero_map_degrees, value_type=float
                        ),
                        "odom_topic": odom_topic,
                        "use_stm32_gateway_odometry": ParameterValue(
                            use_stm32_gateway_odometry, value_type=bool
                        ),
                        "stm32_command_service": stm32_command_service,
                        "stm32_request_timeout_ms": ParameterValue(
                            stm32_request_timeout_ms, value_type=int
                        ),
                        "stm32_enable_odometry_log": ParameterValue(
                            stm32_enable_odometry_log, value_type=bool
                        ),
                        "enable_global_search": ParameterValue(
                            enable_global_search, value_type=bool
                        ),
                        "global_search_random_ratio": ParameterValue(
                            global_search_random_ratio, value_type=float
                        ),
                        "global_search_noise_xy": ParameterValue(
                            global_search_noise_xy, value_type=float
                        ),
                        "global_search_noise_theta": ParameterValue(
                            global_search_noise_theta, value_type=float
                        ),
                        "localized_xy_std_threshold": ParameterValue(
                            localized_xy_std_threshold, value_type=float
                        ),
                        "localized_theta_std_threshold": ParameterValue(
                            localized_theta_std_threshold, value_type=float
                        ),
                        "localized_min_updates": ParameterValue(
                            localized_min_updates, value_type=int
                        ),
                        "lost_alpha_ratio_threshold": ParameterValue(
                            lost_alpha_ratio_threshold, value_type=float
                        ),
                        "lost_min_updates": ParameterValue(
                            lost_min_updates, value_type=int
                        ),
                        "sigma_hit": ParameterValue(
                            sigma_hit, value_type=float
                        ),
                        "noise_xy": ParameterValue(
                            noise_xy, value_type=float
                        ),
                        "noise_theta": ParameterValue(
                            noise_theta, value_type=float
                        ),
                        "alpha_fast_rate": ParameterValue(
                            alpha_fast_rate, value_type=float
                        ),
                        "alpha_slow_rate": ParameterValue(
                            alpha_slow_rate, value_type=float
                        ),
                        "random_injection_max_ratio": ParameterValue(
                            random_injection_max_ratio, value_type=float
                        ),
                        "off_map_penalty": ParameterValue(
                            off_map_penalty, value_type=float
                        ),
                        "occupancy_threshold": ParameterValue(
                            occupancy_threshold, value_type=int
                        ),
                        "distance_transform_mask_size": ParameterValue(
                            distance_transform_mask_size, value_type=int
                        ),
                        "init_field_width": ParameterValue(
                            init_field_width, value_type=float
                        ),
                        "init_field_height": ParameterValue(
                            init_field_height, value_type=float
                        ),
                        "odom_noise_x_from_x": ParameterValue(
                            odom_noise_x_from_x, value_type=float
                        ),
                        "odom_noise_x_from_y": ParameterValue(
                            odom_noise_x_from_y, value_type=float
                        ),
                        "odom_noise_x_from_theta": ParameterValue(
                            odom_noise_x_from_theta, value_type=float
                        ),
                        "odom_noise_x_bias": ParameterValue(
                            odom_noise_x_bias, value_type=float
                        ),
                        "odom_noise_y_from_x": ParameterValue(
                            odom_noise_y_from_x, value_type=float
                        ),
                        "odom_noise_y_from_y": ParameterValue(
                            odom_noise_y_from_y, value_type=float
                        ),
                        "odom_noise_y_from_theta": ParameterValue(
                            odom_noise_y_from_theta, value_type=float
                        ),
                        "odom_noise_y_bias": ParameterValue(
                            odom_noise_y_bias, value_type=float
                        ),
                        "odom_noise_theta_from_x": ParameterValue(
                            odom_noise_theta_from_x, value_type=float
                        ),
                        "odom_noise_theta_from_y": ParameterValue(
                            odom_noise_theta_from_y, value_type=float
                        ),
                        "odom_noise_theta_from_theta": ParameterValue(
                            odom_noise_theta_from_theta, value_type=float
                        ),
                        "odom_noise_theta_bias": ParameterValue(
                            odom_noise_theta_bias, value_type=float
                        ),
                        "filter_period_ms": ParameterValue(
                            filter_period_ms, value_type=int
                        ),
                        "publish_processing_time": ParameterValue(
                            topdown_pf_publish_processing_time, value_type=bool
                        ),
                        "processing_time_topic": topdown_pf_processing_time_topic,
                        "enable_timing_log": ParameterValue(
                            topdown_pf_enable_timing_log, value_type=bool
                        ),
                        "timing_log_interval": ParameterValue(
                            topdown_pf_timing_log_interval, value_type=int
                        ),
                    }
                ],
            ),
        ]
    )
