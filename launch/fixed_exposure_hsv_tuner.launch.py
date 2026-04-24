from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            Node(
                package="camera_ros",
                executable="camera_node",
                name="camera",
                parameters=[
                    {
                        "height": 600,
                        "width": 800,
                        "sensor_mode": "1332:990",
                        "AeEnable": False,
                        "ExposureTime": 10000,
                        "ExposureTimeMode": 1,
                    }
                ],
            ),
            Node(
                package="rcj_localization",
                executable="white_line_hsv_white_node",
                parameters=[
                    {
                        "input_topic": "/camera/image_raw",
                        "enable_image_view": True,
                        "enable_controls_window": True,
                    }
                ],
            ),
        ]
    )
