from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
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


def generate_launch_description():
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
        DeclareLaunchArgument("interpolation", default_value="linear"),  # Remap interpolation mode
        DeclareLaunchArgument("camera_enable_image_view", default_value="false"),  # Show camera debug window
        DeclareLaunchArgument("camera_show_published_image", default_value="true"),  # Show the camera output image
        DeclareLaunchArgument("remap_enable_image_view", default_value="true"),  # Show remap debug windows
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
        DeclareLaunchArgument("morph_enable_timing_debug", default_value="false"),  # Enable morph timing logs
        DeclareLaunchArgument("morph_timing_summary_interval", default_value="10"),  # Morph timing summary frame interval

        DeclareLaunchArgument("skeleton_enable_image_view", default_value="true"),  # Show skeleton debug windows
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
    )
