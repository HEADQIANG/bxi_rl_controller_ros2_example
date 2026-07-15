#!/usr/bin/env bash
set -euo pipefail

REPO_URL="${REPO_URL:-https://github.com/Cloudpilot-Liftingwater/bxi_rl_controller_ros2_example.git}"
BRANCH="${BRANCH:-feature/sonic-elf3-runtime}"
SRC_DIR="${SRC_DIR:-${HOME}/bxi_rl_controller_ros2_example}"
OPT_PREFIX="${OPT_PREFIX:-/opt/bxi/bxi_rl_controller_ros2_example}"
BUILD_INSTALL="${BUILD_INSTALL:-/tmp/elf3_sonic_install}"
BACKUP_DIR="${BACKUP_DIR:-/opt/bxi/deploy_backups}"

if [[ "${EUID}" -eq 0 ]]; then
  SUDO=()
else
  SUDO=(sudo)
fi

echo "[deploy] repo=${REPO_URL}"
echo "[deploy] branch=${BRANCH}"
echo "[deploy] src=${SRC_DIR}"
echo "[deploy] opt=${OPT_PREFIX}"

if [[ -d "${SRC_DIR}/.git" ]]; then
  cd "${SRC_DIR}"
  if [[ "${ALLOW_DIRTY:-0}" != "1" ]] && [[ -n "$(git status --porcelain)" ]]; then
    echo "[deploy] ERROR: ${SRC_DIR} has local changes."
    echo "[deploy] Commit/stash them first, or rerun with ALLOW_DIRTY=1 to overwrite intentionally."
    exit 2
  fi
  git remote add sonic "${REPO_URL}" 2>/dev/null || git remote set-url sonic "${REPO_URL}"
  git fetch sonic "${BRANCH}"
  git checkout -B "${BRANCH}" "sonic/${BRANCH}"
else
  git clone -b "${BRANCH}" --single-branch "${REPO_URL}" "${SRC_DIR}"
  cd "${SRC_DIR}"
fi

echo "[deploy] commit=$(git rev-parse --short HEAD)"

source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash

rm -rf build log "${BUILD_INSTALL}"

colcon build \
  --merge-install \
  --install-base "${BUILD_INSTALL}" \
  --packages-select bxi_example_py_elf3 remote_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

"${SUDO[@]}" mkdir -p "${BACKUP_DIR}"
backup_name="bxi_rl_controller_ros2_example.before_sonic_$(date +%Y%m%d_%H%M%S).tgz"

if [[ -d "${OPT_PREFIX}" ]]; then
  "${SUDO[@]}" tar -C "$(dirname "${OPT_PREFIX}")" -czf "${BACKUP_DIR}/${backup_name}" "$(basename "${OPT_PREFIX}")"
  echo "[deploy] backup=${BACKUP_DIR}/${backup_name}"
else
  echo "[deploy] warning: ${OPT_PREFIX} does not exist; creating it"
  "${SUDO[@]}" mkdir -p "${OPT_PREFIX}"
fi

"${SUDO[@]}" cp -a "${BUILD_INSTALL}/." "${OPT_PREFIX}/"

source "${OPT_PREFIX}/setup.bash"

ros2 pkg prefix bxi_example_py_elf3
ros2 pkg prefix remote_controller
ros2 pkg executables bxi_example_py_elf3 | grep sonic_pico_runtime_supervisor
grep -n "sonic_pico_python" "${OPT_PREFIX}/share/bxi_example_py_elf3/launch/example_demo_hw.launch.py"
grep -n "sonic_teleop" "${OPT_PREFIX}/share/bxi_example_py_elf3/config/elf3_state_machine.yaml" | head

echo "[deploy] done"
echo "[deploy] next: bash ${SRC_DIR}/script/check_robot_sonic_runtime.sh"
