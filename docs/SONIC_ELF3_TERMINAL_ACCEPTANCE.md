# ELF3 SONIC 平板前终端验收

目标是在不依赖平板启动控制框架的情况下，先从远程终端验证硬件主控、状态机、
SONIC fallback、自动 PICO runtime、5556/5557 数据流和退出清理。完成后再停止终端
进程，使用平板重复同一条状态链路。

## 安全前提

- 机器人可靠悬挂或处于经过批准的安全测试姿态，急停可用；
- 操作人员确认周围无人，先测试状态机和日志，再做连续动作；
- 不在硬件主控运行时覆盖 `/opt`；
- 不同时运行自动 supervisor 和 `run_sonic_pico_sources.sh`；
- 不因为看到 `8081` 被 gateway 占用就自动杀 gateway。

## 0. 静态与残留检查

先执行：

```bash
BUNDLE=/tmp/elf3_sonic_deploy_<commit>_ubuntu22_amd64

bash "$BUNDLE/source/script/audit_robot_sonic_host.sh"
bash "$BUNDLE/source/script/check_robot_sonic_runtime.sh"
```

记录当前进程和端口：

```bash
ps -ef | grep -E 'robot_gateway|hardware_elf3|bxi_example_py_elf3_demo|remote_controller|sonic_pico_runtime_supervisor|pico_manager|pico_pose|RoboticsServiceProcess' | grep -v grep || true
sudo ss -lntup | grep -E ':(8081|60061|5556|5557)\b' || true
```

开始 T1 前不能已有另一份 `hardware_elf3`、controller 或 PICO manager。若硬件日志出现
`bxi pci busy`，先定位现有主控所有者，不要并行启动第二份硬件节点。

## 1. T1：硬件主控、状态机和自动 supervisor

root 终端执行：

```bash
BUNDLE=/tmp/elf3_sonic_deploy_<commit>_ubuntu22_amd64
bash "$BUNDLE/source/script/run_robot_sonic_hw.sh"
```

必须看到：

```text
state graph loaded
robot reset 1!
robot reset 2!
[CONTROL RATE] state=zero_torque, hz=50.0
watching hardware/state_machine_info for state=sonic_teleop
```

若出现 `No module named zmq`、`can init failed`、`bxi pci busy` 或 controller exit code 1，
不要继续切状态。

## 2. T2：终端状态控制

另一个终端执行：

```bash
BUNDLE=/tmp/elf3_sonic_deploy_<commit>_ubuntu22_amd64
bash "$BUNDLE/source/script/run_robot_sonic_controller.sh"
```

在确认机器人安全后，按既定顺序测试：

```text
! -> pd_brake
1 -> normal
6 -> sonic_teleop
```

进入 `sonic_teleop` 后，T1 应继续以约 50 Hz 控制，并先报告
`idle_reference`。这证明无 live 数据时的安全 fallback 正常，但尚不能证明 PICO 链路。

## 3. T3：进程和端口观察

第三个终端执行：

```bash
watch -n 1 "ps -ef | grep -E 'hardware_elf3|bxi_example_py_elf3_demo|sonic_pico_runtime_supervisor|pico_manager_legacy|pico_pose_to_smpl_ref_bridge|RoboticsServiceProcess' | grep -v grep; echo PORTS; sudo ss -lntup | grep -E ':(8081|60061|5556|5557)\\b' || true"
```

进入 SONIC 后应只有一份 manager 和一份 bridge。`8081` 只记录，不作为单项失败条件。

## 4. T4：ROS 状态机观察

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source /opt/bxi/bxi_rl_controller_ros2_example/setup.bash

ros2 topic echo /hardware/state_machine_info
```

依次确认 `pd_brake`、`normal`、`sonic_teleop`。如果终端看不到 topic，先核对
`ROS_DOMAIN_ID`/RMW；不要仅凭这个现象否定正在运行的主控，可同时参考 T1 当前日志。

## 5. T5：PICO 与 live 数据

PICO 连接机器人 IPv4 并 send。成功顺序必须是：

```text
Body data available
ZMQ socket bound to port 5556
ABXY -> Calibration completed / OFF -> PLANNER
A+X -> PLANNER -> POSE
PoseLoop FPS: 50.xx
```

然后使用部署报告第 3.7 节的采样命令确认：

- `5556 pose`：约 50 Hz、`mode=1`、`calib=true`、frame 持续增长；
- `5557 smpl_ref`：约 50 Hz、`ready=true`、`mode=1`、frame 持续增长；
- policy 日志：`reference status: live_reference`；
- 机器人连续跟随动作。

若 PICO 报 socket error，先判断 `Body data available` 和 `5556` 是否出现，再结合当前
进程、当前日志和抓包定位；不要只根据 `8081` owner 下结论。

## 6. 退出与清理验收

从 SONIC 切回 `normal`，确认 manager、bridge 和其子进程退出，5556/5557 消失；
supervisor 本身仍由 T1 launch 管理。随后先停止 T2，再 Ctrl+C 停止 T1。

```bash
pgrep -af 'hardware_elf3|bxi_example_py_elf3_demo|remote_controller|sonic_pico_runtime_supervisor|pico_manager_legacy|pico_pose_to_smpl_ref_bridge|RoboticsServiceProcess' || true
sudo ss -lntup | grep -E ':(60061|5556|5557)\b' || true
```

清理通过后再由平板启动控制框架，重复 `normal -> sonic_teleop -> ABXY -> A+X`。
平板测试与终端测试都以 5556/5557 持续流动和 `live_reference` 为最终标准。
