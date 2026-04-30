#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd "$(dirname "${SCRIPT_PATH}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WORKSPACE_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"
SETUP_FILE="${WORKSPACE_ROOT}/install/setup.bash"

if [ ! -f "${SETUP_FILE}" ]; then
  echo "Workspace setup file not found: ${SETUP_FILE}" >&2
  echo "Run: cd ${WORKSPACE_ROOT} && colcon build --packages-select rcj_localization" >&2
  exit 1
fi

echo "Testing YOLO auto environment hook"
echo "Workspace: ${WORKSPACE_ROOT}"
echo "Package:   ${REPO_ROOT}"

env -i \
  HOME="${HOME:-}" \
  USER="${USER:-}" \
  LOGNAME="${LOGNAME:-}" \
  TERM="${TERM:-dumb}" \
  PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
  REPO_ROOT="${REPO_ROOT}" \
  WORKSPACE_ROOT="${WORKSPACE_ROOT}" \
  bash --noprofile --norc -eo pipefail -c '
    source "${WORKSPACE_ROOT}/install/setup.bash"

    echo
    echo "Resolved tools after source install/setup.bash:"
    echo "  python: $(command -v python)"
    echo "  yolo:   $(command -v yolo)"
    echo "  YOLO_CONFIG_DIR: ${YOLO_CONFIG_DIR:-unset}"
    echo "  MPLCONFIGDIR:    ${MPLCONFIGDIR:-unset}"

    case "$(command -v python)" in
      "${REPO_ROOT}/.venv-yolo/bin/python") ;;
      *)
        echo "Expected python to resolve to ${REPO_ROOT}/.venv-yolo/bin/python" >&2
        exit 1
        ;;
    esac

    "${REPO_ROOT}/scripts/check_yolo_env.py"
  '

echo
echo "YOLO auto environment hook test passed."
