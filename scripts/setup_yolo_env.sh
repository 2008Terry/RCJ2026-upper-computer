#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV_DIR="${YOLO_VENV_DIR:-${REPO_ROOT}/.venv-yolo}"
BOOTSTRAP_DIR="${REPO_ROOT}/.yolo-bootstrap"
PYTHON_BIN="${PYTHON_BIN:-python3}"

TORCH_VERSION="2.11.0+cpu"
TORCHVISION_VERSION="0.26.0+cpu"
POLARS_VERSION="1.40.1"
ULTRALYTICS_THOP_VERSION="2.0.19"
ULTRALYTICS_VERSION="8.4.45"

echo "Creating YOLO virtual environment at ${VENV_DIR}"
if ! "${PYTHON_BIN}" -m venv --system-site-packages "${VENV_DIR}"; then
  echo "python3-venv is unavailable; bootstrapping virtualenv locally"
  rm -rf "${VENV_DIR}" "${BOOTSTRAP_DIR}"
  "${PYTHON_BIN}" -m pip install --target "${BOOTSTRAP_DIR}" "virtualenv==20.35.4"
  PYTHONPATH="${BOOTSTRAP_DIR}" "${PYTHON_BIN}" -m virtualenv \
    --system-site-packages "${VENV_DIR}"
fi

PY="${VENV_DIR}/bin/python"
"${PY}" -m pip install --upgrade pip wheel "setuptools<80"

echo "Installing CPU PyTorch and TorchVision"
"${PY}" -m pip install \
  "torch==${TORCH_VERSION}" \
  "torchvision==${TORCHVISION_VERSION}" \
  --index-url https://download.pytorch.org/whl/cpu

echo "Installing Ultralytics runtime dependencies"
"${PY}" -m pip install \
  "polars==${POLARS_VERSION}" \
  "ultralytics-thop==${ULTRALYTICS_THOP_VERSION}"

echo "Installing Ultralytics without the pip OpenCV wheel"
"${PY}" -m pip install --no-deps "ultralytics==${ULTRALYTICS_VERSION}"

export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-${REPO_ROOT}/.yolo-config}"
export MPLCONFIGDIR="${MPLCONFIGDIR:-${REPO_ROOT}/.yolo-config/matplotlib}"
mkdir -p "${YOLO_CONFIG_DIR}" "${MPLCONFIGDIR}"

echo "Running YOLO/ROS Python smoke test"
"${PY}" "${REPO_ROOT}/scripts/check_yolo_env.py"

cat <<EOF

YOLO environment is ready.

Rebuild/source the workspace once so ROS installs the automatic YOLO environment hook:
  cd ${REPO_ROOT}/../..
  colcon build --packages-select rcj_localization
  source install/setup.bash

After that, new terminals that source this workspace will pick up YOLO automatically.
EOF
