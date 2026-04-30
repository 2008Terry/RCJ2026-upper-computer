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

## Notes for the Future YOLO Black Circle Node

- Keep model weights out of git; `*.pt` is ignored.
- Prefer `yolo11n.pt` or a small custom-trained model first on Raspberry Pi.
- Publish detections into the existing `BlackFeatureDetectionArray` message so
  the AMCL pipeline can consume the same semantic black-feature stream.

Official references checked while setting this up:

- Ultralytics install docs: https://docs.ultralytics.com/quickstart/
- PyTorch install guidance: https://pytorch.org/get-started/locally/
