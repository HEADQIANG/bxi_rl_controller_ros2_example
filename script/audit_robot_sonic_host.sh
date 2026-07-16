#!/usr/bin/env bash
set -u

BLOCKERS=0

ok() { echo "[OK] $*"; }
warn() { echo "[WARN] $*"; }
block() { echo "[BLOCK] $*"; BLOCKERS=$((BLOCKERS + 1)); }
section() { echo; echo "== $* =="; }

section "Host"
echo "hostname=$(hostname)"
echo "architecture=$(uname -m)"
echo "kernel=$(uname -r)"
if [[ -r /etc/os-release ]]; then
  sed -n -E 's/^(NAME|VERSION|VERSION_ID)=/\1=/p' /etc/os-release
fi
if [[ "$(uname -m)" == "x86_64" ]]; then
  ok "x86_64 architecture"
else
  block "offline payload only supports x86_64"
fi

if python3 - <<'PY'
import sys
print("python=" + sys.version.replace("\n", " "))
raise SystemExit(0 if sys.version_info[:2] == (3, 10) else 1)
PY
then
  ok "Python 3.10"
else
  block "Python 3.10 is required by the cp310 wheels"
fi

section "Disk"
df -h /home /opt /tmp 2>/dev/null | awk 'NR == 1 || !seen[$1]++'
available_mb="$(df -Pm /tmp | awk 'NR == 2 {print $4}')"
if [[ "${available_mb}" =~ ^[0-9]+$ ]] && (( available_mb >= 1800 )); then
  ok "/tmp has at least 1800 MiB free"
else
  block "/tmp needs at least 1800 MiB free for bundle, build and backup"
fi

section "Required base installs"
for path in \
  /opt/ros/humble/setup.bash \
  /opt/bxi/bxi_ros2_pkg/setup.bash; do
  if [[ -r "${path}" ]]; then
    ok "${path}"
  else
    block "missing ${path}"
  fi
done
command -v colcon >/dev/null 2>&1 && ok "colcon exists" || block "colcon is missing"

section "Current example install"
for path in \
  /opt/bxi/bxi_rl_controller_ros2_example/setup.bash \
  /opt/bxi/bxi_rl_controller_ros2_example/lib/python3.10/site-packages; do
  if [[ -e "${path}" ]]; then
    ok "${path}"
  else
    warn "missing ${path}; the deployment can create it"
  fi
done

section "Existing PICO runtime"
for path in \
  /home/bxi/bxi_rl_controller_ros2_example-main/.venv_teleop/bin/python \
  /opt/apps/roboticsservice/RoboticsServiceProcess \
  /opt/apps/roboticsservice/SDK/x64/libPXREARobotSDK.so; do
  if [[ -e "${path}" ]]; then
    ok "${path}"
  else
    warn "missing ${path}; the offline payload will install it"
  fi
done

section "Active control processes"
if processes="$(pgrep -af 'hardware_elf3|bxi_example_py_elf3_demo|pico_manager_legacy|pico_pose_to_smpl_ref_bridge|RoboticsServiceProcess' || true)" && \
   [[ -n "${processes}" ]]; then
  echo "${processes}"
  block "stop active robot control/PICO processes before replacing /opt"
else
  ok "no active robot control/PICO processes"
fi

section "Relevant ports"
if ports="$(ss -lntup 2>/dev/null | grep -E ':(8081|60061|5556|5557)\b' || true)" && \
   [[ -n "${ports}" ]]; then
  echo "${ports}"
else
  ok "8081/60061/5556/5557 are currently free"
fi

section "ROS environment hints"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-unset}"
hostname -I 2>/dev/null || true

section "Result"
if (( BLOCKERS == 0 )); then
  ok "host is ready for the offline deployment"
else
  echo "[BLOCK] ${BLOCKERS} pre-deployment blocker(s)"
fi
exit "${BLOCKERS}"
