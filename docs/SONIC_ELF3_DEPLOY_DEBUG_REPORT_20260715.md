# ELF3 SONIC/PICO 部署 Debug 报告

日期：2026-07-15  
目标：机器人切到 `sonic_teleop` 后自动启动 PICO runtime，PICO 连接机器人，ABXY 完成校准，A+X 切到 POSE/live，最终由 PICO 实时驱动 SONIC 遥操。

## 1. 最终跑通状态

最终成功日志关键点如下：

```text
[Manager] Body data available after 12.99s
[Manager] ZMQ socket bound to port 5556
[PoseLoop] Robot model loaded for FK calibration
[PoseLoop] Calibration completed (zero-pose reference)
[Manager] StreamMode switch: OFF -> PLANNER
[Manager] StreamMode switch: PLANNER -> POSE
[PoseLoop] FPS: 50.25
```

这说明完整链路已打通：

```text
PICO
  -> RoboticsServiceProcess / xrobotoolkit_sdk
  -> pico_manager_legacy
  -> 5556 pose
  -> pico_pose_to_smpl_ref_bridge
  -> 5557 smpl_ref
  -> SonicTeleopPolicy live_reference
```

## 2. 主要问题和根因

### 2.1 缺 Python 依赖：torch

现象：

```text
ModuleNotFoundError: No module named 'torch'
```

处理：

机器人无 CUDA，安装 CPU 版 torch 即可。后续应使用 PICO 专用 venv，而不是系统 Python。

### 2.2 缺 XRoboToolkit SDK / RoboticsService

现象：

```text
ImportError: XRoboToolkit SDK not available. Install xrobotoolkit_sdk to run the manager.
```

处理：

补齐：

- `/home/bxi/bxi_rl_controller_ros2_example-main/.venv_teleop`
- `xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so`
- `/opt/apps/roboticsservice/RoboticsServiceProcess`

验证：

```bash
/home/bxi/bxi_rl_controller_ros2_example-main/.venv_teleop/bin/python - <<'PY'
import xrobotoolkit_sdk
print("xrt OK", xrobotoolkit_sdk)
PY

ls -lh /opt/apps/roboticsservice/RoboticsServiceProcess
```

### 2.3 ROS setup 在 `set -u` 下报 `AMENT_TRACE_SETUP_FILES: unbound variable`

现象：

```text
/opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
```

处理：

部署脚本需要兼容 ROS setup 对未定义变量的使用。已在 helper scripts 中修正，不应在 `set -u` 状态下直接 source ROS setup。

### 2.4 PICO manager 默认带 `--cuda`

现象：

机器人端没有 CUDA 硬件，`--cuda` 启动可疑，也不符合部署预期。

处理：

`runtime_supervisor.py` 改为默认 CPU，仅在显式设置环境变量时启用 CUDA：

```python
if env_flag_enabled(env, "SONIC_PICO_USE_CUDA", default=False):
    manager.append("--cuda")
```

验证：

```bash
grep -n -- '--cuda\|SONIC_PICO_USE_CUDA' \
  /opt/bxi/bxi_rl_controller_ros2_example/lib/python3.10/site-packages/bxi_example_py_elf3/sonic_pico/runtime_supervisor.py
```

期望只看到条件追加 `--cuda`，不能固定带 `--cuda`。

### 2.5 PICO 能连接但一 send 就 socket error

这是本次最关键问题。

现象：

- 机器人可以切到 `sonic_teleop`
- SONIC idle 姿势正常
- PICO 显示能 connect
- 但 PICO 一点 send 就报 socket error
- ABXY / A+X 无效果
- manager / bridge 进程存在，但无 `5556 LISTEN`

关键排查：

```bash
sudo ss -lntup | grep -E ':(8081|60061|5556|5557)\b'
sudo ss -antup | grep -E '192\.168\.88\.210|8081|60061|5556|5557'
```

异常状态：

```text
0.0.0.0:8081 users:(api_server_node / robot_gateway / ...)
192.168.88.152:8081 <-> 192.168.88.210 ESTAB
127.0.0.1:5557 LISTEN
[::ffff:127.0.0.1]:60061 LISTEN RoboticsService
no 5556 LISTEN
```

根因：

PICO 端不能配置端口，默认连接机器人 `8081`。但机器人启动平板 gateway 后，`api_server_node` 占用了 `0.0.0.0:8081`。因此 PICO 实际连到的是 robot gateway，不是 RoboticsService 的 PICO/XRT body tracking 入口。连接可建立，但协议不匹配，一 send 就 socket error。

正确理解：

- SONIC idle 正常，只说明 fallback `idle_reference` 正常；
- 它不证明 PICO live 数据进来了；
- 没有 `5556 LISTEN` 时，ABXY / A+X 必然无效。

临时验证方案：

```bash
sudo pkill -f 'socat.*60061' 2>/dev/null || true

sudo pkill -INT -f 'robot_gateway gateway.launch.py|api_server_node|bxi_example_bms|bxi_bms|hardware_elf3|bxi_example_py_elf3_demo|sonic_pico_runtime_supervisor|pico_manager_legacy|pico_pose_to_smpl_ref_bridge|RoboticsServiceProcess' 2>/dev/null || true

sleep 3
sudo ss -lntup | grep -E ':(8081|60061|5556|5557)\b' || true
```

然后手动启动 manager：

```bash
source /opt/ros/humble/setup.bash
source /opt/bxi/bxi_ros2_pkg/setup.bash
source /opt/bxi/bxi_rl_controller_ros2_example/setup.bash

PY=/home/bxi/bxi_rl_controller_ros2_example-main/.venv_teleop/bin/python

$PY -m bxi_example_py_elf3.sonic_pico.pico_manager_legacy \
  --manager \
  --num_frames_to_send 10 \
  --target_fps 50 \
  --port 5556
```

成功后可看到：

```text
device found
Body data available
ZMQ socket bound to port 5556
```

长期方案：

- SONIC/PICO 模式下不能让 `robot_gateway/api_server_node` 占用 `8081`；
- 需要调整启动架构：进入 SONIC/PICO 前释放 `8081`，或改 gateway 端口；
- 如果 PICO 端将来支持端口配置，也可以改 PICO 目标端口，但当前 PICO 不能配置端口。

### 2.6 缺 pinocchio

现象：

PICO body data 已进来，manager 已绑定 `5556`，但随后崩溃：

```text
ModuleNotFoundError: No module named 'pinocchio'
```

根因：

`ThreePointPose` 初始化 FK calibration 时会加载 `gear_sonic.data.robot_model...`，需要 `pinocchio`。

处理：

安装离线 wheel：

- `pin`
- `eigenpy`
- `hpp_fcl`
- `cmeel*`

### 2.7 NumPy 2.x ABI 与 pin/eigenpy 不兼容

现象：

安装 `pin/eigenpy/hpp_fcl` 后 import 报：

```text
A module that was compiled using NumPy 1.x cannot be run in NumPy 2.2.6
Segmentation fault
```

根因：

当前 wheel 是按 NumPy 1.x ABI 编译的，不能和 NumPy 2.2.6 混用。

处理：

PICO venv 降级：

```bash
PY=/home/bxi/bxi_rl_controller_ros2_example-main/.venv_teleop/bin/python

$PY -m pip install --no-index --force-reinstall --no-deps \
  /tmp/sonic_numpy126_wheels/numpy-1.26.4-*.whl \
  /tmp/sonic_numpy126_wheels/scipy-1.15.3-*.whl
```

最终 PICO venv 推荐版本：

```text
numpy==1.26.4
scipy==1.15.3
pin==2.7.0
eigenpy==3.5.1
hpp-fcl==2.4.4
torch CPU
pyzmq
msgpack
xrobotoolkit_sdk
```

验证：

```bash
PY=/home/bxi/bxi_rl_controller_ros2_example-main/.venv_teleop/bin/python

$PY - <<'PY'
import sys
import numpy
import scipy
import eigenpy
import hppfcl
import pinocchio as pin
import torch
import zmq
import msgpack
import xrobotoolkit_sdk

print("python", sys.executable)
print("numpy", numpy.__version__, numpy.__file__)
print("scipy", scipy.__version__, scipy.__file__)
print("pinocchio", pin.__version__)
print("torch", torch.__version__)
print("all OK")
PY
```

## 3. 新机器人部署 Checklist

### 3.1 拉取并部署 example 包

```bash
cd ~

if [ ! -d "$HOME/bxi_rl_controller_ros2_example/.git" ]; then
  git clone -b feature/sonic-elf3-runtime --single-branch \
    https://github.com/Cloudpilot-Liftingwater/bxi_rl_controller_ros2_example.git \
    "$HOME/bxi_rl_controller_ros2_example"
else
  cd "$HOME/bxi_rl_controller_ros2_example"
  git remote add sonic https://github.com/Cloudpilot-Liftingwater/bxi_rl_controller_ros2_example.git 2>/dev/null || \
    git remote set-url sonic https://github.com/Cloudpilot-Liftingwater/bxi_rl_controller_ros2_example.git
  git fetch sonic feature/sonic-elf3-runtime
  git checkout -B feature/sonic-elf3-runtime sonic/feature/sonic-elf3-runtime
fi

bash "$HOME/bxi_rl_controller_ros2_example/script/deploy_robot_sonic_example.sh"
```

如果 GitHub 网络不稳定，可使用离线 patch/bundle，不要在机器人上长时间等待公网 pip/git。

### 3.2 安装/确认 PICO venv 和 RoboticsService

必须存在：

```text
/home/bxi/bxi_rl_controller_ros2_example-main/.venv_teleop/bin/python
/opt/apps/roboticsservice/RoboticsServiceProcess
```

验证：

```bash
bash ~/bxi_rl_controller_ros2_example/script/check_sonic_pico_python.sh \
  /home/bxi/bxi_rl_controller_ros2_example-main/.venv_teleop/bin/python

bash ~/bxi_rl_controller_ros2_example/script/check_robot_sonic_runtime.sh
```

注意：检查脚本应补充 `pinocchio/eigenpy/hppfcl` 和 NumPy 版本检查。

### 3.3 检查 8081 端口冲突

PICO 不能配端口，默认使用 `8081`。部署 SONIC/PICO 前必须确认：

```bash
sudo ss -lntup | grep ':8081' || true
```

如果看到：

```text
0.0.0.0:8081 users:(api_server_node/robot_gateway/...)
```

则 PICO 会连错服务，send 后 socket error。

### 3.4 验证 PICO body data 是否真正进 manager

启动 manager 后观察：

```text
Body data available
ZMQ socket bound to port 5556
```

同时检查：

```bash
ss -lntup | grep -E ':(5556|5557|60061|8081)\b'
```

关键判断：

- 有 `5556 LISTEN`：PICO pose manager 已起来；
- 没有 `5556 LISTEN`：ABXY/A+X 不可能有效；
- 只有 `5557 LISTEN`：bridge 在等 pose，但 manager 没出数据；
- SONIC idle 姿势正常只代表 fallback `idle_reference` 正常，不代表 live PICO 正常。

### 3.5 ABXY / A+X 成功判断

成功日志：

```text
[PoseLoop] Calibration completed
[Manager] StreamMode switch: OFF -> PLANNER
[Manager] StreamMode switch: PLANNER -> POSE
[PoseLoop] FPS: 50.xx
```

桥接成功后还应看到：

```text
[pico->smpl_ref] sent ...
```

SONIC policy 成功切 live 后应从 `idle_reference` 变为 `live_reference`。

## 4. 建议固化到 GitHub / 部署包的内容

### 4.1 必须补进依赖检查脚本

`script/check_sonic_pico_python.sh` 应检查：

- `numpy`，且建议要求 `<2`
- `scipy`
- `zmq`
- `msgpack`
- `torch`
- `xrobotoolkit_sdk`
- `pinocchio`
- `eigenpy`
- `hppfcl`
- `/opt/apps/roboticsservice/RoboticsServiceProcess`

### 4.2 必须补进离线 wheelhouse

机器人公网慢，不应依赖现场 pip 下载。部署包应包含：

```text
numpy-1.26.4
scipy-1.15.3
pin-2.7.0
eigenpy-3.5.1
hpp_fcl-2.4.4
cmeel*
torch CPU
pyzmq
msgpack
```

### 4.3 必须补进 8081 端口检查

部署/运行脚本应在启动 PICO runtime 前检查：

```bash
ss -lntup | grep ':8081'
```

如果 `api_server_node` 或 `robot_gateway` 占用 `8081`，应明确报错或自动进入互斥流程，避免 PICO 连接错误服务。

### 4.4 CPU-only 默认必须保留

机器人无 CUDA，`runtime_supervisor.py` 默认不能传 `--cuda`。只允许通过：

```bash
SONIC_PICO_USE_CUDA=1
```

显式开启。

### 4.5 文档必须说明 fallback reference

部署文档里要明确：

- 机器人切到 sonic idle 不代表 PICO live 数据已通；
- 没有 `5556 LISTEN` 时，ABXY/A+X 必然无效；
- `idle_reference` 是安全 fallback；
- `live_reference` 需要 PICO body data + ABXY + A+X + bridge `source_ready=true`。

## 5. 推荐的一键排雷命令

```bash
PY=/home/bxi/bxi_rl_controller_ros2_example-main/.venv_teleop/bin/python

echo "===== processes ====="
ps -ef | grep -E 'robot_gateway|api_server_node|RoboticsServiceProcess|pico_manager|pico_pose|sonic_pico_runtime_supervisor|bxi_example_py_elf3_demo|hardware_elf3' | grep -v grep || true

echo "===== ports ====="
sudo ss -lntup | grep -E ':(8081|60061|5556|5557)\b' || true
sudo ss -antup | grep -E ':(8081|60061|5556|5557)\b' || true

echo "===== python deps ====="
$PY - <<'PY'
import sys
print("python", sys.executable)

mods = [
    "numpy",
    "scipy",
    "zmq",
    "msgpack",
    "torch",
    "xrobotoolkit_sdk",
    "eigenpy",
    "hppfcl",
    "pinocchio",
]

for name in mods:
    try:
        mod = __import__(name)
        version = getattr(mod, "__version__", "unknown")
        print(f"[OK] {name}: {version}")
    except Exception as exc:
        print(f"[FAIL] {name}: {exc!r}")
PY

echo "===== roboticsservice ====="
ls -lh /opt/apps/roboticsservice/RoboticsServiceProcess || true
```

## 6. 最短结论

本次不是 ROS2 domain 问题，也不是 SONIC policy 不支持 live input。真正踩坑点是：

1. PICO 默认连 `8081`，但 robot gateway 占了 `8081`，导致 send 即 socket error；
2. PICO venv 缺 `pinocchio/eigenpy/hpp_fcl`；
3. `pin/eigenpy` 与 NumPy 2.x ABI 不兼容，需要 NumPy 1.26；
4. 机器人无 CUDA，manager 默认必须 CPU-only；
5. 新机器人部署必须使用离线 wheelhouse，不能依赖现场公网下载。

