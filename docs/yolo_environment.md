# YOLO Environment

This repo uses a local Python virtual environment for Ultralytics YOLO so the
ROS 2 Jazzy system Python remains clean.

## Setup

From `rcj_localization`:

```bash
./scripts/setup_yolo_env.sh
cd ../..
colcon build --packages-select rcj_localization
source install/setup.bash
```

The virtual environment is created at `.venv-yolo` with
`--system-site-packages`, which lets future Python YOLO ROS nodes import
`rclpy`, `cv_bridge`, and the apt-provided OpenCV.

If Ubuntu's `python3.12-venv` package is not installed, the setup script
bootstraps `virtualenv` into `.yolo-bootstrap` and still creates the same
`.venv-yolo` environment without sudo.

The package installs an `ament` environment hook. Once the workspace has been
rebuilt, sourcing `install/setup.bash` automatically prepends the YOLO virtual
environment to `PATH` and `PYTHONPATH`, so ROS launch files and Python nodes can
import Ultralytics without manually activating the virtual environment. The hook
also sets `YOLO_CONFIG_DIR` and `MPLCONFIGDIR` under `.yolo-config` so
Ultralytics and Matplotlib keep runtime configuration local to this ignored repo
directory instead of the default home config path.

The setup script installs:

- `torch==2.11.0+cpu`
- `torchvision==0.26.0+cpu`
- `ultralytics==8.4.45`
- `polars==1.40.1`
- `ultralytics-thop==2.0.19`

PyTorch is installed from the official CPU wheel index. Ultralytics is installed
with `--no-deps` after the runtime dependencies so pip does not install a
separate `opencv-python` wheel over ROS OpenCV.

## Verify

```bash
./scripts/test_yolo_auto_env.sh
```

This starts a clean shell, sources the workspace `install/setup.bash`, and then
checks that `python` resolves to `.venv-yolo/bin/python`, `yolo` is on `PATH`,
and the YOLO/ROS Python imports work.

For a manual dependency-only check inside an already sourced terminal:

```bash
./scripts/check_yolo_env.py
yolo checks
```

`yolo checks` may report `opencv-python` as missing because this setup uses
Ubuntu's apt OpenCV package instead of pip's OpenCV wheel. That is expected as
long as `check_yolo_env.py` reports `opencv`, `rclpy`, and `cv_bridge` present.

## Export To NCNN

After `source install/setup.bash`, both `python3` and `yolo` resolve from the
repo-local `.venv-yolo` environment, so you can export the existing trained
YOLO `.pt` model to NCNN format with:

```bash
python3 scripts/export_yolo_to_ncnn.py \
  --model path/to/best.pt \
  --imgsz 320
```

The helper prints the generated `*_ncnn_model/` directory path on success.
Generated NCNN export directories are ignored by git alongside `*.pt` weights.

The runtime now accepts the exported NCNN directory directly via
`model_path:=/path/to/best_ncnn_model`.

## Live Black Circle YOLO Debugging

Run the live remapped-camera YOLO web viewer with:

```bash
ros2 launch rcj_localization yolo_black_circle_debug.launch.py
```

Then open `http://<robot-ip>:8081/` in a browser. If you are on the same
machine, `http://localhost:8081/` also works.

The default model path is `~/Downloads/train-6/weights/best_ncnn_model`.
Supported `model_path` forms are:

- `~/Downloads/train-6/weights/best_ncnn_model`
- `~/Downloads/train-6/weights/best.pt`
- `~/Downloads/train-6`

Override it explicitly when needed:

```bash
ros2 launch rcj_localization yolo_black_circle_debug.launch.py \
  model_path:=~/Downloads/train-6/weights/best_ncnn_model confidence:=0.25 imgsz:=320
```

The browser view streams annotated detections and live stats for received FPS,
processed FPS, model inference time, total frame time, skipped frames, detections, and
maximum confidence. It does not require Qt or a graphical desktop session on the
robot.

The node now also publishes a ROS ROI mask for downstream integration:

- `/yolo_black_circle_debug/roi_mask` as `sensor_msgs/msg/Image`
- encoding: `mono8`
- semantics: `255` inside the selected YOLO bounding box, `0` elsewhere

Step 1 selection behavior is intentionally simple:

- YOLO runs on the remapped top-down image
- if multiple detections exist, the node uses the highest-confidence detection
- if there are no detections, the ROI mask is all zeros
- the node does not reuse stale detections from previous frames

Use a different port when needed:

```bash
ros2 launch rcj_localization yolo_black_circle_debug.launch.py web_port:=8082
```

If `camera_ros` reports `no cameras available`, the YOLO node is fine; the
camera is not visible to `camera_ros`. Check that the camera is connected/enabled
and not already owned by another process, or run with `enable_camera:=false`
when another node is already publishing `/camera/image_raw`.

## ROI Mask Validation

Run the launch file:

```bash
ros2 launch rcj_localization yolo_black_circle_debug.launch.py
```

Then validate the new ROI output with:

```bash
ros2 topic echo --once /yolo_black_circle_debug/roi_mask/header
ros2 topic hz /yolo_black_circle_debug/roi_mask
```

For image inspection, either subscribe in RViz/rqt or republish the mask through
your usual image-viewing workflow. The expected checks for step 1 are:

1. Single detection: the web/debug image shows one box and the ROI mask contains one filled rectangle aligned to it.
2. Multiple detections: only the highest-confidence box appears in the ROI mask.
3. No detections: the ROI mask becomes all zeros.
4. Consistency: the ROI mask resolution and header stamp match the remapped YOLO input frame.

ROI publishing is enabled by default in the launch file. Disable it explicitly
only if needed:

```bash
ros2 launch rcj_localization yolo_black_circle_debug.launch.py publish_roi_mask:=false
```

## ROI-Gated Black Mask Validation

Step 2 adds a sidecar HSV node that turns the remapped image plus the YOLO ROI
into a separate ROI-gated black mask. It does not modify the existing white-line
or AMCL pipeline yet.

Run the combined step-1 plus step-2 debug launch with:

```bash
ros2 launch rcj_localization yolo_roi_black_mask_debug.launch.py \
  model_path:=~/Downloads/train-6/weights/best_ncnn_model
```

The new node publishes:

- `/yolo_roi_black_mask/black_mask` as `sensor_msgs/msg/Image`
- `/yolo_roi_black_mask/debug/overlay_image` as `sensor_msgs/msg/Image`

The black-mask output uses the matched remapped image header and is all zeros
when the synced YOLO ROI is all zeros.

Useful checks:

```bash
ros2 topic echo --once /yolo_roi_black_mask/black_mask/header
ros2 topic hz /yolo_roi_black_mask/black_mask
```

Expected validation results for step 2:

1. Single detection over a black feature: black pixels appear only inside the YOLO rectangle.
2. Single detection over a non-black area: the output is mostly or completely zero.
3. No detections: the output mask becomes all zeros.
4. Outside-ROI guarantee: no black pixels appear outside `/yolo_black_circle_debug/roi_mask`.

For visual inspection, compare `/yolo_roi_black_mask/black_mask`,
`/yolo_roi_black_mask/debug/overlay_image`, and
`/yolo_black_circle_debug/roi_mask` in RViz or `rqt_image_view`.

## Notes for the Future YOLO Black Circle Node

- Keep model weights out of git; `*.pt` is ignored.
- Prefer `yolo11n.pt` or a small custom-trained model first on Raspberry Pi.
- Publish detections into the existing `BlackFeatureDetectionArray` message so
  the AMCL pipeline can consume the same semantic black-feature stream.

Official references checked while setting this up:

- Ultralytics install docs: https://docs.ultralytics.com/quickstart/
- PyTorch install guidance: https://pytorch.org/get-started/locally/
