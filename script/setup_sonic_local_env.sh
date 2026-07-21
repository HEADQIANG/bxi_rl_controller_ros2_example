#!/usr/bin/env bash

# Source this file from each terminal before running the local SONIC stack.
_SONIC_ENV_SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_SONIC_ENV_ROOT="$(cd "${_SONIC_ENV_SCRIPT_DIR}/.." && pwd)"

_SONIC_ENV_ROS_SETUP="/opt/ros/humble/setup.bash"
_SONIC_ENV_BXI_SETUP="${_SONIC_ENV_ROOT}/.local_deps/bxi_ros2_pkg/setup.bash"
_SONIC_ENV_VENV="${_SONIC_ENV_ROOT}/.venv_teleop"
_SONIC_ENV_OVERLAY_SETUP="${_SONIC_ENV_ROOT}/.local_install/setup.bash"

for _SONIC_ENV_REQUIRED in \
  "${_SONIC_ENV_ROS_SETUP}" \
  "${_SONIC_ENV_BXI_SETUP}" \
  "${_SONIC_ENV_VENV}/bin/activate" \
  "${_SONIC_ENV_OVERLAY_SETUP}"; do
  if [[ ! -r "${_SONIC_ENV_REQUIRED}" ]]; then
    echo "[sonic-env] missing required file: ${_SONIC_ENV_REQUIRED}" >&2
    unset _SONIC_ENV_REQUIRED
    return 1
  fi
done

_SONIC_ENV_NOUNSET=0
case $- in
  *u*) _SONIC_ENV_NOUNSET=1; set +u ;;
esac

# shellcheck disable=SC1090
source "${_SONIC_ENV_ROS_SETUP}"
# shellcheck disable=SC1090
source "${_SONIC_ENV_BXI_SETUP}"
# shellcheck disable=SC1090
source "${_SONIC_ENV_VENV}/bin/activate"
# shellcheck disable=SC1090
source "${_SONIC_ENV_OVERLAY_SETUP}"

export SONIC_PICO_PYTHON="${_SONIC_ENV_VENV}/bin/python"
export SONIC_PNLINK_PYTHON="${SONIC_PNLINK_PYTHON:-${_SONIC_ENV_VENV}/bin/python}"
export SONIC_XRT_SERVICE_DIR="${SONIC_XRT_SERVICE_DIR:-/opt/apps/roboticsservice}"
export SONIC_PICO_USE_CUDA="${SONIC_PICO_USE_CUDA:-0}"

_SONIC_ENV_PNLINK_SDK="${_SONIC_ENV_ROOT}/../PNlink_mocap/mocapapi_python"
if [[ -d "${_SONIC_ENV_PNLINK_SDK}" ]]; then
  _SONIC_ENV_PNLINK_SDK="$(cd "${_SONIC_ENV_PNLINK_SDK}" && pwd)"
fi
if [[ -z "${SONIC_PNLINK_SDK_PATH:-}" && -f "${_SONIC_ENV_PNLINK_SDK}/mocap_api.py" ]]; then
  export SONIC_PNLINK_SDK_PATH="${_SONIC_ENV_PNLINK_SDK}"
fi

for _SONIC_ENV_LIB_DIR in \
  "${_SONIC_ENV_ROOT}/.local_deps/glfw/usr/lib/x86_64-linux-gnu" \
  "${SONIC_XRT_SERVICE_DIR}/SDK/x64" \
  "${SONIC_XRT_SERVICE_DIR}" \
  "${SONIC_XRT_SERVICE_DIR}/lib"; do
  if [[ -d "${_SONIC_ENV_LIB_DIR}" ]]; then
    case ":${LD_LIBRARY_PATH:-}:" in
      *:"${_SONIC_ENV_LIB_DIR}":*) ;;
      *) export LD_LIBRARY_PATH="${_SONIC_ENV_LIB_DIR}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ;;
    esac
  fi
done

if (( _SONIC_ENV_NOUNSET )); then
  set -u
fi

echo "[sonic-env] ROS 2 Humble + BXI + local overlay ready"
echo "[sonic-env] SONIC_PICO_PYTHON=${SONIC_PICO_PYTHON}"
echo "[sonic-env] SONIC_PNLINK_PYTHON=${SONIC_PNLINK_PYTHON}"
if [[ -n "${SONIC_PNLINK_SDK_PATH:-}" ]]; then
  echo "[sonic-env] SONIC_PNLINK_SDK_PATH=${SONIC_PNLINK_SDK_PATH}"
fi

unset _SONIC_ENV_SCRIPT_DIR _SONIC_ENV_ROOT _SONIC_ENV_ROS_SETUP
unset _SONIC_ENV_BXI_SETUP _SONIC_ENV_VENV _SONIC_ENV_OVERLAY_SETUP
unset _SONIC_ENV_REQUIRED _SONIC_ENV_LIB_DIR _SONIC_ENV_NOUNSET
unset _SONIC_ENV_PNLINK_SDK
