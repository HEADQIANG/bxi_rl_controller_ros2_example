# ELF3 SONIC 离线部署手册

这套流程用于 Ubuntu 22.04、x86_64、Python 3.10 的 ELF3 主控。目标是把同一个已
验证的 Git commit、同一组固定版本依赖部署到每台机器人，并继续覆盖平板 App 实际
使用的 `/opt/bxi/bxi_rl_controller_ros2_example`。

## 部署包边界

GitHub 只存源码、部署脚本和检查脚本。PICO/XRoboToolkit vendor 文件与大体积 wheel
不提交到 GitHub；它们由工作站上的 `prepare_robot_sonic_bundle.sh` 合入离线包。

离线包只保留实机路径需要的版本：

- `numpy==1.26.4`，不包含 NumPy 2.x；
- `torch==2.6.0+cpu`，不启用 CUDA；
- `scipy==1.15.3`、`pyzmq==27.1.0`、`msgpack==1.1.2`；
- `pin==2.7.0`、`eigenpy==3.5.1`、`hpp-fcl==2.4.4` 及其 cmeel 依赖；
- `onnxruntime==1.23.2` 及控制节点所需依赖；
- 离线 `pip`/`setuptools` 启动 wheel，不依赖目标机 apt 或公网；
- RoboticsService deb 与 `xrobotoolkit_sdk` CPython 3.10 扩展。

安装脚本分别处理两套 Python 环境：

1. PICO venv：`/home/bxi/bxi_rl_controller_ros2_example-main/.venv_teleop`；
2. App 启动的控制节点环境：`/opt/bxi/bxi_rl_controller_ros2_example/lib/python3.10/site-packages`。

第二套环境会显式安装 `pyzmq`，避免 PICO 检查通过但
`bxi_example_py_elf3_demo` 因缺 `zmq` 退出。

## 工作站生成部署包

要求源码仓库没有未提交修改。执行：

```bash
cd /path/to/bxi_rl_controller_ros2_example

bash script/prepare_robot_sonic_bundle.sh \
  /tmp/elf3_sonic_runtime_deps_20260716 \
  /home/huangchenwei/elf3_sonic_artifacts
```

脚本会：

- 用 `git archive HEAD` 固定源码 commit；
- 只选择唯一的固定版本 wheel；
- 拒绝缺文件、重复版本或混入 NumPy 2.x 的 payload；
- 生成逐文件 `MANIFEST.sha256` 和整个 tgz 的 SHA256。

## 机器人部署前盘点

先把小型审计脚本传到机器人并执行，或者解包后运行：

```bash
bash source/script/audit_robot_sonic_host.sh
```

必须满足：

- x86_64、Python 3.10；
- `/opt/ros/humble` 和 `/opt/bxi/bxi_ros2_pkg` 存在；
- `/tmp` 至少约 1800 MiB 可用；
- `hardware_elf3`、控制节点和 PICO runtime 已停止。

审计只读取状态，不会停止进程或改机器人。

## 上传与一键部署

在工作站先核对并上传：

```bash
sha256sum -c elf3_sonic_deploy_<commit>_ubuntu22_amd64.tgz.sha256
scp elf3_sonic_deploy_<commit>_ubuntu22_amd64.tgz bxi@<robot>:/tmp/
```

在机器人上执行：

```bash
cd /tmp
tar -xzf elf3_sonic_deploy_<commit>_ubuntu22_amd64.tgz

cd elf3_sonic_deploy_<commit>_ubuntu22_amd64
bash source/script/deploy_robot_sonic_bundle.sh "$PWD"
```

部署器按顺序执行：校验所有文件、确认控制进程已停止、离线 colcon build、备份旧
`/opt` 包、覆盖 App 使用的 `/opt` 包、安装两套 Python 依赖、验证 XRT native 库并
运行完整健康检查。它不会自动停止机器人控制进程，也不会自动杀掉 robot gateway。

## 成功标准

静态部署至少需要全部通过：

- `ros2 pkg prefix bxi_example_py_elf3` 指向 `/opt/bxi/bxi_rl_controller_ros2_example`；
- `sonic_pico_runtime_supervisor` 已安装；
- PICO venv 的 NumPy、SciPy、torch CPU、pinocchio、XRT 全部可导入；
- 控制节点 Python 可导入 `numpy`、`onnxruntime`、`zmq` 和 SONIC policy；
- `RoboticsServiceProcess` 与 `SDK/x64/libPXREARobotSDK.so` 存在；
- 记录 `8081` owner 作为诊断基线，但不把 gateway 占用本身当成失败条件。

动态验证时还必须看到：

```text
Body data available
ZMQ socket bound to port 5556
Calibration completed
StreamMode switch: OFF -> PLANNER
StreamMode switch: PLANNER -> POSE
PoseLoop FPS: 50.xx
```

并确认 `5556 pose`、`5557 smpl_ref` 均持续约 50 Hz，SONIC 状态为
`live_reference`。一次姿势变化不等于实时遥操成功。

## 回滚

代码覆盖前会在这里生成带时间戳的备份：

```text
/opt/bxi/deploy_backups/bxi_rl_controller_ros2_example.before_sonic_*.tgz
```

如需回滚，先停止控制进程，再把目标备份解压回 `/opt/bxi`。不要在电机主控运行时
覆盖或回滚 `/opt`。
