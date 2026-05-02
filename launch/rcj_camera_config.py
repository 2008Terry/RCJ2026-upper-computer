from pathlib import Path

from ament_index_python.packages import get_package_share_directory


# Change this value when switching the calibrated camera XML pair.
# Expected config filenames:
#   <CAMERA_CONFIG_ID>_undistort_map_*.xml
#   <CAMERA_CONFIG_ID>_raw_ball_top_lut_*.xml
CAMERA_CONFIG_ID = "camera3"


def _config_dir():
    return Path(get_package_share_directory("rcj_localization")) / "config"


def _resolve_single_xml(pattern, description):
    matches = sorted(_config_dir().glob(pattern))
    if not matches:
        raise FileNotFoundError(
            f"No {description} XML matched pattern '{pattern}' in {_config_dir()}"
        )
    if len(matches) > 1:
        match_list = "\n  ".join(str(path) for path in matches)
        raise RuntimeError(
            f"Multiple {description} XML files matched pattern '{pattern}'. "
            f"Keep exactly one for {CAMERA_CONFIG_ID}:\n  {match_list}"
        )
    return matches[0]


def resolve_fastmap_file():
    return _resolve_single_xml(
        f"{CAMERA_CONFIG_ID}_undistort_map_*.xml",
        "FastMap undistort",
    )


def resolve_raw_ball_lut_file():
    return _resolve_single_xml(
        f"{CAMERA_CONFIG_ID}_raw_ball_top_lut_*.xml",
        "raw ball LUT",
    )
