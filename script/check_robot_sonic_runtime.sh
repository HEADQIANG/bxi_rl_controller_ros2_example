#!/usr/bin/env bash
set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FAILURES=0

ok() { echo "[OK] $*"; }
warn() { echo "[WARN] $*"; }
fail() { echo "[FAIL] $*"; FAILURES=$((FAILURES + 1)); }
section() { echo; echo "== $* =="; }

source_if_exists() {
  local file="$1"
  if [[ -f "${file}" ]]; then
    set +u
    # shellcheck disable=SC1090
    source "${file}"
    set -u
    ok "sourced ${file}"
  else
    warn "missing ${file}"
  fi
}

section "ROS environment"
source_if_exists /opt/ros/humble/setup.bash
source_if_exists /opt/bxi/bxi_ros2_pkg/setup.bash
source_if_exists /opt/bxi/bxi_rl_controller_ros2_example/setup.bash
echo "[INFO] ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-}"

section "Installed package"
if prefix="$(ros2 pkg prefix bxi_example_py_elf3 2>/dev/null)"; then
  ok "bxi_example_py_elf3 prefix=${prefix}"
else
  fail "ros2 cannot find bxi_example_py_elf3"
fi

if prefix="$(ros2 pkg prefix remote_controller 2>/dev/null)"; then
  ok "remote_controller prefix=${prefix}"
else
  fail "ros2 cannot find remote_controller"
fi

if ros2 pkg executables bxi_example_py_elf3 2>/dev/null | grep -q sonic_pico_runtime_supervisor; then
  ok "sonic_pico_runtime_supervisor executable is installed"
else
  fail "missing sonic_pico_runtime_supervisor executable"
fi

if [[ -f /opt/bxi/bxi_rl_controller_ros2_example/share/bxi_example_py_elf3/launch/example_demo_hw.launch.py ]]; then
  if grep -q "sonic_pico_python" /opt/bxi/bxi_rl_controller_ros2_example/share/bxi_example_py_elf3/launch/example_demo_hw.launch.py; then
    ok "hardware launch exposes sonic_pico_python"
  else
    warn "hardware launch does not expose sonic_pico_python; redeploy latest package"
  fi
else
  warn "installed hardware launch file not found under /opt/bxi"
fi

section "PICO Python dependencies"
if "${SCRIPT_DIR}/check_sonic_pico_python.sh"; then
  ok "PICO Python dependency check passed"
else
  fail "PICO Python dependency check failed"
fi

section "Current ROS state"
if timeout 3 ros2 topic echo --once /hardware/state_machine_info >/tmp/sonic_state_machine_info.txt 2>/tmp/sonic_state_machine_info.err; then
  ok "/hardware/state_machine_info is publishing"
  sed -n '1,8p' /tmp/sonic_state_machine_info.txt
else
  warn "no /hardware/state_machine_info sample within 3s; T1 may not be running"
  sed -n '1,8p' /tmp/sonic_state_machine_info.err
fi

section "Processes"
if pgrep -af 'bxi_example_py_elf3_demo|hardware_elf3|remote_controller|sonic_pico_runtime_supervisor|pico_manager_legacy|pico_pose_to_smpl_ref_bridge|RoboticsServiceProcess'; then
  ok "runtime processes listed above"
else
  warn "no matching runtime processes"
fi

section "Ports"
if ss -lntup 2>/dev/null | grep -E ':(5556|5557|60061)\b'; then
  ok "SONIC/PICO ports listed above"
else
  warn "no 5556/5557/60061 listeners; this is expected before SONIC/PICO starts"
fi

section "Result"
if (( FAILURES == 0 )); then
  ok "robot SONIC runtime checks passed"
else
  fail "${FAILURES} blocking check(s) failed"
fi

exit "${FAILURES}"
