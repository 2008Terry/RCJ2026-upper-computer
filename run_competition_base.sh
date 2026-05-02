#!/usr/bin/env bash

set -eo pipefail

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${PACKAGE_DIR}/../.." && pwd)"
ROS_SETUP="/opt/ros/humble/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
    echo "ROS setup file not found: ${ROS_SETUP}" >&2
    exit 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
    echo "Workspace has not been built yet: ${WORKSPACE_SETUP}" >&2
    echo "Run ./run_rcj.sh once before using the match scripts." >&2
    exit 1
fi

set +u
source "${ROS_SETUP}"
source "${WORKSPACE_SETUP}"
set -u

cd "${WORKSPACE_DIR}"
exec ros2 launch rcj_localization competition_base.launch.py "$@"
