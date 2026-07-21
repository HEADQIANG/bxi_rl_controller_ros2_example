#!/usr/bin/env bash
# Run PN-Link -> pose -> smpl_ref entirely in Python. The source stays in the
# foreground so it can read the vendor-compatible single-key controls.
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUNTIME_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PACKAGE_SOURCE="${RUNTIME_ROOT}/src/bxi_example_py_elf3"
PNLINK_PYTHON="${SONIC_PNLINK_PYTHON:-${RUNTIME_ROOT}/.venv_teleop/bin/python}"

if [[ ! -x "${PNLINK_PYTHON}" ]]; then
  PNLINK_PYTHON="$(command -v python3)"
fi

DEFAULT_SDK_PATH="${RUNTIME_ROOT}/../PNlink_mocap/mocapapi_python"
export SONIC_PNLINK_SDK_PATH="${SONIC_PNLINK_SDK_PATH:-${DEFAULT_SDK_PATH}}"
export PYTHONPATH="${PACKAGE_SOURCE}${PYTHONPATH:+:${PYTHONPATH}}"
export PNLINK_LOCAL_IP="${PNLINK_LOCAL_IP:-10.42.0.101}"
export PNLINK_LOCAL_PORT="${PNLINK_LOCAL_PORT:-8002}"
export PNLINK_SERVER_IP="${PNLINK_SERVER_IP:-10.42.0.202}"
export PNLINK_SERVER_PORT="${PNLINK_SERVER_PORT:-8080}"
export SONIC_PNLINK_BIND_HOST="${SONIC_PNLINK_BIND_HOST:-127.0.0.1}"
export SONIC_PNLINK_PORT="${SONIC_PNLINK_PORT:-5556}"
export BXI_SONIC_SMPL_REF_ZMQ_HOST="${BXI_SONIC_SMPL_REF_ZMQ_HOST:-127.0.0.1}"
export BXI_SONIC_SMPL_REF_ZMQ_PORT="${BXI_SONIC_SMPL_REF_ZMQ_PORT:-5557}"
export BXI_SONIC_SMPL_REF_ZMQ_TOPIC="${BXI_SONIC_SMPL_REF_ZMQ_TOPIC:-smpl_ref}"

if [[ ! -f "${SONIC_PNLINK_SDK_PATH}/mocap_api.py" ]]; then
  echo "[sonic-pnlink] missing PN-Link SDK: ${SONIC_PNLINK_SDK_PATH}" >&2
  exit 1
fi

bridge_pid=""
viewer_pid=""
cleanup() {
  if [[ -n "${viewer_pid}" ]] && kill -0 "${viewer_pid}" 2>/dev/null; then
    kill -INT "${viewer_pid}" 2>/dev/null || true
    wait "${viewer_pid}" 2>/dev/null || true
  fi
  if [[ -n "${bridge_pid}" ]] && kill -0 "${bridge_pid}" 2>/dev/null; then
    kill -INT "${bridge_pid}" 2>/dev/null || true
    wait "${bridge_pid}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

"${PNLINK_PYTHON}" -m \
  bxi_example_py_elf3.sonic_pico.pico_pose_to_smpl_ref_bridge \
  --source-kind pnlink \
  --pose-host "${SONIC_PNLINK_BIND_HOST}" \
  --pico-port "${SONIC_PNLINK_PORT}" \
  --pico-topic pose \
  --out-host "${BXI_SONIC_SMPL_REF_ZMQ_HOST}" \
  --out-port "${BXI_SONIC_SMPL_REF_ZMQ_PORT}" \
  --out-topic "${BXI_SONIC_SMPL_REF_ZMQ_TOPIC}" \
  --stale-warning-seconds "${SONIC_PNLINK_STALE_SECONDS:-0.2}" \
  --disable-ros-pico-topics \
  --disable-ros-diagnostics &
bridge_pid=$!

case "${SONIC_PNLINK_MUJOCO_VIEWER:-1}" in
  1|true|TRUE|yes|YES|on|ON)
    export SONIC_PNLINK_DIAGNOSTICS=1
    nice -n 10 "${PNLINK_PYTHON}" -m \
      bxi_example_py_elf3.sonic_pnlink.mujoco_viewer &
    viewer_pid=$!
    ;;
esac

echo "[sonic-pnlink] pure-Python bridge pid=${bridge_pid}"
if [[ -n "${viewer_pid}" ]]; then
  echo "[sonic-pnlink] MuJoCo viewer pid=${viewer_pid}"
fi
echo "[sonic-pnlink] SDK=${SONIC_PNLINK_SDK_PATH}"
echo "[sonic-pnlink] keep this terminal focused for PN-Link key control"

"${PNLINK_PYTHON}" -m bxi_example_py_elf3.sonic_pnlink.pose_source
