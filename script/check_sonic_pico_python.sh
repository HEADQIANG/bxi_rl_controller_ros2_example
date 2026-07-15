#!/usr/bin/env bash
set -u

PYTHON_BIN="${1:-${SONIC_PICO_PYTHON:-}}"

if [[ -z "${PYTHON_BIN}" ]]; then
  for candidate in \
    /home/bxi/bxi_rl_controller_ros2_example-main/.venv_teleop/bin/python \
    /home/bxi/bxi_rl_controller_ros2_example/.venv_teleop/bin/python \
    /home/bxi/bxi_ws/bxi_rl_controller_ros2_example/.venv_teleop/bin/python \
    /opt/bxi/bxi_rl_controller_ros2_example/.venv_teleop/bin/python \
    python3; do
    if command -v "${candidate}" >/dev/null 2>&1 || [[ -x "${candidate}" ]]; then
      PYTHON_BIN="${candidate}"
      break
    fi
  done
fi

if [[ -z "${PYTHON_BIN}" ]]; then
  echo "[FAIL] no Python interpreter found"
  exit 2
fi

if ! command -v "${PYTHON_BIN}" >/dev/null 2>&1 && [[ ! -x "${PYTHON_BIN}" ]]; then
  echo "[FAIL] Python interpreter is not executable: ${PYTHON_BIN}"
  exit 2
fi

echo "[INFO] PICO Python: ${PYTHON_BIN}"

"${PYTHON_BIN}" - <<'PY'
import importlib
import sys

required = ["numpy", "zmq", "msgpack", "torch", "xrobotoolkit_sdk"]
failed = False

for name in required:
    try:
        module = importlib.import_module(name)
        version = getattr(module, "__version__", "unknown")
        print(f"[OK] import {name}: {version}")
    except Exception as exc:
        failed = True
        print(f"[FAIL] import {name}: {exc!r}")

try:
    import torch
    print(f"[INFO] torch.cuda.is_available={torch.cuda.is_available()}")
except Exception:
    pass

sys.exit(1 if failed else 0)
PY
PY_STATUS=$?

if [[ -x /opt/apps/roboticsservice/RoboticsServiceProcess ]]; then
  echo "[OK] RoboticsServiceProcess exists"
else
  echo "[WARN] missing /opt/apps/roboticsservice/RoboticsServiceProcess"
fi

exit "${PY_STATUS}"
