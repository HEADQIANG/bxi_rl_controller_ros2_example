#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source /opt/bxi/bxi_rl_controller_ros2_example/setup.bash

echo "[robot-sonic-controller] keyboard: ! -> pd_brake, 1 -> normal, 6 -> sonic_teleop"
echo "[robot-sonic-controller] PICO after sonic: ABXY -> calibrate/start, A+X -> POSE/live"

ros2 launch remote_controller remote_controller_keyboard.launch.py "$@"
