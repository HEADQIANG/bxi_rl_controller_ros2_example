# PN-Link 替代 PICO 接入 ELF3 SONIC 实施计划（单机第一版）

## 1. 文档目的

本文定义第一版 PN-Link 全身动捕接入 ELF3 SONIC 的实现边界、数据处理方法、
运行拓扑、接口契约、安全状态机、测试步骤和验收标准。目标是让实现人员不需要再决定
骨架映射、数据格式、进程关系或失联行为，可以直接按本文完成开发和验证。

第一版只替代 PICO 的全身姿态输入，不控制夹爪：

- PN-Link source、bridge、SONIC controller 和硬件主控运行在同一台机器人电脑；
- 覆盖躯干、头部、双臂、手腕和双腿的人体动作参考；
- 可选 MuJoCo 诊断进程在同一场景实时并排显示原始 PN-Link 骨架和重定向后的
  SMPL-24 骨架；
- 任一数据校验失败都能定位到处理阶段、原始/SMPL 关节、位置/旋转字段和失败规则；
- SONIC 仍输出原有 ELF3 29 关节控制命令；
- 不根据 PN-Link 手指数据生成夹爪命令；
- 不发布 `pico/left_trigger`、`pico/right_trigger` 等夹爪控制输入；
- `BXI_SONIC_GRIPPER_ENABLE` 必须保持为 `0`；
- PICO 路径继续保留，并保持为默认路径，便于回归和回退。

## 2. 当前接口分析

### 2.1 PN-Link 当前输出

`PNlink_mocap/mocapapi_python` 中有两条现成输出路径，但都不能直接作为 SONIC 输入：

1. `mocap_main_stickman_ros2.py` 发布 PN-Link BVH 骨架的局部 TF。位置单位为厘米，
   四元数由 SDK 以 `wxyz` 返回；ROS 侧转换为米和 `xyzw`。
2. `mocap_main_robot_ros2.py` 调用 PN-Link SDK 自带的 G1 retarget，并以 10 Hz 发布
   `/joint_states`。这已经是目标机器人关节角，不再是 SONIC 所需的人体运动参考。

第二条路径不能复用。SONIC 本身是人体参考到 ELF3 关节控制的策略，如果先经过 G1
关节重定向再送入 SONIC，会丢失 SMPL 人体结构、根朝向和未来窗口语义。

PN-Link SDK 的有效原始输入是每次 `AvatarUpdated` 事件中的：

```text
joint name
local position:   (x, y, z), centimeters
local rotation:   (w, x, y, z)
```

### 2.2 SONIC 当前输入

`SonicTeleopPolicy` 不订阅 TF 或 `/joint_states`。它通过 ZMQ `smpl_ref` topic 消费：

```text
term1_local : float32 [10, 72]
root_quat   : float32 [10, 4]   # wxyz
wrist       : float32 [10, 6]
```

这 10 帧会直接写入 SONIC ONNX 的人体 tokenizer。当前 PICO 链路为：

```text
PICO / XRoboToolkit
  -> pico_manager_legacy
  -> tcp://*:5556 topic=pose
  -> pico_pose_to_smpl_ref_bridge
  -> tcp://*:5557 topic=smpl_ref
  -> SonicTeleopPolicy
```

现有 bridge 已经实现以下关键行为，应保留而不是在 PN-Link 代码中重新实现：

- 10 帧流式窗口合并；
- 连续 3 个有效包的 readiness gate；
- 帧号递增检查；
- 0.2 秒输入超时；
- stale 后停止 live 发布；
- SONIC 侧 live/idle 选择以及 0.4 秒平滑切换。

## 3. 第一版总体架构

第一版固定采用单机部署：PN-Link source、bridge、SONIC policy 和 ELF3 硬件控制程序
全部运行在机器人电脑上。PN-Link suit/server 仍通过独立的 UDP 网络接口向该电脑发送
动捕数据，两个 ZMQ 端口仅使用 loopback。

```text
PN-Link suit/server
        |
        | UDP, existing PN-Link SDK protocol
        v
+-------------------- ELF3 robot computer --------------------+
| sonic_pnlink_pose_source                    |
|  - read AvatarUpdated                       |
|  - PN-Link -> SMPL-24 retarget              |
|  - normalize with SONIC human FK            |
|  - publish 10-frame pose chunks at 50 Hz    |
|                    |                        |
|                    | 127.0.0.1:5556 pose    |
|                    v                        |
| pose_to_smpl_ref_bridge                     |
|  - readiness and stale gate                 |
|  - streamed future-window merge             |
|  - publish 127.0.0.1:5557 topic=smpl_ref     |
|                                             |
| SonicTeleopPolicy -> ELF3 29-DoF command     |
|                                             |
| source -- 127.0.0.1:5558 pnlink_debug       |
|                    v                        |
| sonic_pnlink_mujoco_viewer                  |
|  - raw PN-Link and retargeted SMPL together |
|  - stage/joint/position/rotation diagnostics |
+--------------------------------------------------------------+
```

职责边界：

- 机器人电脑安装 PN-Link 原生 SDK，并同时承担人体标定、SMPL 转换和 SONIC 推理；
- 机器人电脑必须拥有能访问 PN-Link server 的 IPv4 地址，同时保持原有机器人控制网络；
- `pose` 只在 `127.0.0.1:5556` 发布，`smpl_ref` 只在 `127.0.0.1:5557` 发布；
- PN-Link 程序不得直接发布机器人电机命令；
- `script/run_sonic_pnlink_sources.sh` 在前台管理 source，并在后台管理 bridge；
- source 直接读取当前终端的单键输入，不创建 ROS node、service 或 topic；
- bridge 在 PN-Link 模式关闭 ROS 按钮和 ROS diagnostics，5556 到 5557 全程只使用
  Python/ZMQ；
- 脚本退出时同时停止 source 和 bridge，不留下 PN-Link UDP、5556 或 5557 owner；
- MuJoCo viewer 是 source 的只读消费者；viewer 卡顿、关闭或崩溃不得影响 source、bridge
  或机器人控制；
- 跨电脑传输不是第一版范围。

## 4. 数据处理流水线

### 4.1 原始帧采集

每次收到 `AvatarUpdated` 时执行一次原始帧读取：

1. 读取 avatar 全部 joints，并按名字放入字典；
2. 确认第 4.5 节列出的所有必要关节均存在；
3. 检查位置和四元数全部为有限值；
4. 对每个四元数归一化；模长小于 `1e-6` 时丢弃整帧；
5. 使用 `time.monotonic_ns()` 记录本机采样时刻；
6. 只在新的 `AvatarUpdated` 上增加源帧序号，不允许定时器重复旧帧并伪造新帧号。

PN-Link 当前轮询周期是 0.02 秒。第一版保持 50 Hz 目标输出，不使用
`mocap_main_robot_ros2.py` 中的 0.1 秒周期。

### 4.2 坐标系与单位

沿用 PN-Link 项目中已经用于 ROS/MuJoCo 可视化的坐标关系：

```text
PN-Link position (x, y, z) cm
        -> robot position (z, x, y) / 100 m

PN-Link quaternion (w, x, y, z)
        -> robot quaternion (w, z, x, y)
```

矩阵形式定义：

```text
    [0 0 1]
C = [1 0 0]
    [0 1 0]

p_robot = C * p_pnlink / 100
R_robot = C * R_pnlink * C^T
```

`det(C)=1`，因此这是旋转基变换，不是镜像。实现中必须使用旋转矩阵或四元数库完成
基变换，不能通过交换四元数元素之外的额外符号猜测来修正动作方向。

原始位置只用于调试骨架和标定质量检查。SONIC 的 `term1_local` 必须由标准 SMPL
人体模型前向运动学重新生成，不能直接拼接 PN-Link 的厘米位置。这样可避免操作者身高、
四肢长度和穿戴误差改变策略输入分布。

### 4.3 局部到全局旋转

按 PN-Link 层级对每个关节做前向运动学：

```text
G_root = L_root
G_joint = G_parent * L_joint
```

其中 `L` 是 PN-Link 给出的局部旋转，`G` 是 PN-Link 世界坐标下的全局旋转。

SMPL 映射必须基于全局旋转再重新计算 SMPL 局部旋转。不能直接把 PN-Link 局部旋转
复制到 SMPL，因为两套骨架存在 `Neck1` 等额外关节，父子层级并不完全相同。

### 4.4 中立姿态标定

PN-Link 厂商动作标定和 SONIC 中立姿态标定是两个不同步骤：

1. 厂商标定：执行 PN-Link `CommandCalibrateMotion`，保证传感器和人体骨架解算正常；
2. SONIC 标定：操作者面向约定方向保持标准 T 姿态，双腿自然伸直、双脚平行、双臂
   水平展开，采集连续 25 个有效帧作为 SMPL 零位。

25 帧中立数据必须满足：

- 必要关节无缺失；
- 每个关节四元数均有效；
- 每个关节相对平均姿态的最大角差不超过 5 度；
- 25 帧采集总时长不超过 1 秒；
- 不满足稳定性条件时标定失败，不保留部分结果。

四元数平均应使用符号对齐后的 Markley 平均或等价的旋转平均方法，不能直接对四个元素
求算术平均。得到每个源关节的中立全局旋转 `G0[i]`。

对当前帧，先得到机器人 Z-up 坐标下相对中立姿态的全局运动：

```text
D[i] = C * G[i] * inverse(G0[i]) * C^T
```

为了与现有 `process_smpl_joints()` 的 SMPL Y-up 到机器人 Z-up 处理完全一致，定义：

```text
B_yz   = rotation_x(+pi/2)
B_smpl = quaternion(wxyz=[0.5, 0.5, 0.5, 0.5])
S[i]   = inverse(B_yz) * D[i] * B_smpl
```

然后按 SMPL 父节点重新生成局部旋转：

```text
L_smpl[0] = S[0]
L_smpl[i] = inverse(S[parent[i]]) * S[i]
```

这个公式保证：

- 中立 T 姿态下所有非根 SMPL 局部旋转为单位旋转；
- `process_smpl_joints()` 完成 Y-up/Z-up 和 base rotation removal 后，中立根四元数接近
  `[1, 0, 0, 0]`；
- 操作者整体转身表现为根朝向变化，而不是所有身体关节同时产生伪局部旋转。

实现后必须通过测试验证上述三个性质，不能只依赖目视效果。

### 4.5 SMPL-24 映射

使用标准 SMPL 24 关节顺序。PN-Link 到 SMPL 的映射固定如下：

| SMPL index | SMPL joint | PN-Link source | 说明 |
|---:|---|---|---|
| 0 | pelvis | `Hips` | 根节点 |
| 1 | left_hip | `LeftUpLeg` | 左髋 |
| 2 | right_hip | `RightUpLeg` | 右髋 |
| 3 | spine1 | `Spine` | 下躯干 |
| 4 | left_knee | `LeftLeg` | 左膝 |
| 5 | right_knee | `RightLeg` | 右膝 |
| 6 | spine2 | `Spine1` | 中躯干 |
| 7 | left_ankle | `LeftFoot` | 左踝 |
| 8 | right_ankle | `RightFoot` | 右踝 |
| 9 | spine3 | `Spine2` | 上躯干 |
| 10 | left_foot | `LeftTiptoe` | 缺失时继承 `LeftFoot` 全局旋转 |
| 11 | right_foot | `RightTiptoe` | 缺失时继承 `RightFoot` 全局旋转 |
| 12 | neck | `Neck` | 颈根 |
| 13 | left_collar | `LeftShoulder` | 左锁骨 |
| 14 | right_collar | `RightShoulder` | 右锁骨 |
| 15 | head | `Head` | `Neck1` 通过全局旋转折叠 |
| 16 | left_shoulder | `LeftArm` | 左肩 |
| 17 | right_shoulder | `RightArm` | 右肩 |
| 18 | left_elbow | `LeftForeArm` | 左肘 |
| 19 | right_elbow | `RightForeArm` | 右肘 |
| 20 | left_wrist | `LeftHand` | 左腕 |
| 21 | right_wrist | `RightHand` | 右腕 |
| 22 | left_hand | SMPL FK | 由人体模型生成，不读取手指 |
| 23 | right_hand | SMPL FK | 由人体模型生成，不读取手指 |

`LeftTiptoe` 和 `RightTiptoe` 在当前 PN-Link 示例中可能是合成节点。若 SDK 帧中没有这两个
关节，不把整帧判为缺失；其全局旋转分别继承左右脚踝，使对应 SMPL 局部旋转为单位旋转。

### 4.6 SMPL 标准化输出

将 `L_smpl` 转成 axis-angle：

```text
global_orient = axis_angle(L_smpl[0])       # [1, 3]
body_pose     = axis_angle(L_smpl[1:22])    # [1, 63]
```

调用现有 SONIC vendor 路径中的 `process_smpl_joints()`，得到：

```text
smpl_joints_local : [1, 24, 3]
global_orient_quat: [1, 4], wxyz
smpl_pose          : [1, 63]
```

发布前执行：

- 转成 CPU `numpy.float32`；
- 检查所有元素有限；
- 重新归一化根四元数；
- 将根四元数符号调整为与上一帧点积非负，避免 `q`/`-q` 跳变。

### 4.7 ELF3 手腕参考

虽然第一版不控制夹爪，但 SONIC 模型仍需要左右腕部六个关节参考。必须复用当前 PICO
manager 的手腕分解算法，不能把 PN-Link 欧拉角直接按 XYZ 填入。

处理顺序：

1. 从 `body_pose` 取左右肘和左右手腕的 SMPL axis-angle；
2. 围绕 Y 轴做 elbow twist/swing 分解；
3. 将 elbow swing 的 X/Z 分量合入 wrist roll/yaw；
4. 使用现有左右手符号约定生成：

```text
wrist = [
  l_wrist_x, l_wrist_y, l_wrist_z,
  r_wrist_x, r_wrist_y, r_wrist_z,
]
```

建议把当前 PICO manager 内这段纯数学逻辑提取为共享函数，由 PICO 和 PN-Link 两个 source
共同调用，并用现有 PICO 结果做逐元素回归测试。

### 4.8 50 Hz 重采样与窗口

source 内保存最近 10 个标准化帧，并以 50 Hz 发布一个重叠窗口：

```text
message 0: frames [0 ... 9]
message 1: frames [1 ... 10]
message 2: frames [2 ... 11]
```

规则：

- 首次填满 10 个有效帧之前不发布 live pose；
- 原始帧频率高于 50 Hz 时，以单调时钟为基准重采样；
- 位置类张量线性插值，四元数使用 SLERP；
- 原始帧频率不足或超过 0.1 秒没有新帧时停止产生新窗口；
- 不通过复制最后一帧维持假 50 Hz；
- `frame_index` 使用 source 会话内从 0 开始的连续 `int64` 序号；
- 重新标定或 source 重启时允许帧号归零，现有 bridge 已支持会话重启。

第一版不额外添加低通滤波。PN-Link 已提供姿态解算，额外滤波可能增加遥操作延迟。
只实现四元数连续性、重采样和异常帧拒绝；如果实测抖动超标，再基于录制数据确定滤波参数。

## 5. `pose` ZMQ 契约

PN-Link source 在机器人电脑的 loopback 地址绑定：

```text
tcp://127.0.0.1:5556
topic: pose
```

复用 `zmq_messages.pack_pose_message()` 的 1280 字节 JSON header + 连续二进制 payload 格式。

每个 PN-Link pose 包必须包含：

| Field | dtype | shape | 含义 |
|---|---|---|---|
| `frame_index` | `int64` | `[10]` | 窗口内连续帧号 |
| `smpl_joints` | `float32` | `[10,24,3]` | 根坐标系 SMPL joints |
| `body_quat_w` | `float32` | `[10,4]` | SMPL 根四元数，wxyz |
| `wrist` | `float32` | `[10,6]` | ELF3 左右腕参考 |
| `stream_mode` | `int32` | `[1]` | `1` 表示 live pose |
| `calibration_ready` | `bool` | `[1]` | 厂商标定、中立标定和人工使能均完成 |
| `timestamp_realtime` | `float64` | `[1]` | 最新源帧 Unix 时间，日志对照用 |
| `timestamp_monotonic` | `float64` | `[1]` | 机器人电脑单调时钟，端到端延迟诊断用 |

第一版明确不包含：

```text
left_trigger
right_trigger
left_grip
right_grip
left_hand_joints
right_hand_joints
```

bridge 需要做向后兼容扩展：

- 如果 pose 包含 `wrist`，直接校验为 `[N,6]` 并使用；
- 否则保留 PICO 的 `joint_pos[N,29]` 提取路径；
- readiness finite check 对 PN-Link 检查 `smpl_joints/body_quat_w/wrist`；
- 对 PICO 继续检查 `smpl_joints/body_quat_w/joint_pos`；
- 输出 `smpl_ref` 契约保持不变，Sonic policy 不做数据格式修改。

## 6. Source 状态机与控制接口

PN-Link source 使用以下状态：

```text
DISCONNECTED
  -> CAPTURING
  -> VENDOR_CALIBRATING -> NEEDS_NEUTRAL
  -> NEEDS_NEUTRAL
  -> READY_PAUSED
  -> LIVE
```

状态规则：

- 启动时为 `DISCONNECTED`，成功打开 SDK 并收到有效 avatar 后进入 `CAPTURING`；
- 厂商标定成功后进入 `NEEDS_NEUTRAL`；
- `CAPTURING` 时可不执行厂商标定，直接按 `T` 进入 `NEEDS_NEUTRAL`；
- `READY_PAUSED` 时可再次按 `T` 重做中立标定；`LIVE` 时必须先按 `P`；
- 中立 T 姿态 25 帧标定成功后进入 `READY_PAUSED`；
- 只有操作者按 `L` 才进入 `LIVE`；
- 只有 `LIVE` 状态发布 `stream_mode=1, calibration_ready=true` 的 pose；
- 任一必要关节连续缺失、SDK 断开或输入超过 0.1 秒时退出 `LIVE`；
- 断线恢复后回到 `READY_PAUSED`，不得自动重新使能；
- 重新执行厂商标定会清除旧中立标定和 10 帧窗口。

控制直接读取启动 source 的前台终端，不使用命令行参数或 ROS 2 service：

```text
N  Start Capture                 与参考工程一致
F  Stop Capture                  与参考工程一致
C  Calibrate Motion              与参考工程一致
R  Resume Hands                  与参考工程一致；Linux SDK 不支持时明确报错
0  Clear Zero Motion Drift       与参考工程一致
O  Resume Body                   与参考工程一致
Z  Zero Position                 与参考工程一致
T  采集 25 帧 SONIC 中立 T 姿态（C 可选，暂停时可重做）
L  开启 live
P  暂停 live 并清空窗口
H  显示按键帮助
ESC 退出 source 和 bridge
```

状态以 JSON 行直接输出到当前终端，至少包含：

```json
{
  "state": "READY_PAUSED",
  "sdk_connected": true,
  "vendor_calibrated": true,
  "neutral_calibrated": true,
  "live_enabled": false,
  "source_hz": 50.0,
  "last_frame_age_ms": 12.0,
  "missing_joints": []
}
```

按键只提交异步厂商命令；只有收到成功的 `CommandReply` 后才改变厂商标定/采集状态。

## 7. 代码组织与改动范围

### 7.1 新增 PN-Link source

在 `bxi_example_py_elf3` 中新增独立模块，建议结构：

```text
sonic_pnlink/
  __init__.py
  skeleton.py          # PN-Link hierarchy, SMPL mapping, coordinate conversion
  calibration.py       # quaternion averaging and neutral-pose calibration
  retarget.py          # PN-Link global rotations -> SMPL -> wrist
  diagnostics.py       # stage checks, structured issues, ring buffer and bundles
  debug_wire.py        # non-blocking 5558 debug frame schema
  pose_source.py       # SDK lifecycle, terminal-key state machine, ZMQ publisher
  mujoco_viewer.py     # raw and SMPL skeletons in one MuJoCo scene
```

新增 console script：

```text
sonic_pnlink_pose_source
```

PN-Link SDK 不复制进 Python package。source 通过：

```text
SONIC_PNLINK_SDK_PATH=/opt/bxi/vendor/pnlink/mocapapi_python
```

加载 `mocap_api.py`。启动时检查：

- 路径存在；
- 当前架构对应的 `librobotapi_*.so` 存在；
- `mocap_api` 可以导入；
- PN-Link UDP endpoint 可以打开；
- ZMQ 5556 端口未被占用。
- diagnostics 启用时 ZMQ 5558 端口未被占用。

任一检查失败时进程以非零状态退出，并打印具体路径或端口错误。

source 和 bridge 使用独立于 ROS controller 的纯 Python 环境；不需要继承 ROS 系统包：

```bash
python3 -m venv /opt/bxi/venvs/sonic_pnlink
```

环境中安装 SONIC runtime、`docutils`、`PyYAML`；启用 viewer 时还要安装 `mujoco`。
验证可同时导入 `torch`、`scipy`、`zmq`、`mocap_api` 和 SONIC vendor 模块；PN-Link
运行环境不要求 `rclpy`、`std_srvs` 或 `diagnostic_msgs`。
PN-Link source 启动时设置：

```text
torch.set_num_threads(1)
torch.set_num_interop_threads(1)
```

避免 SMPL FK 与 ONNX controller 在同一台电脑上产生 CPU 线程过量竞争。controller 仍按
现有方式使用 ROS 安装环境，不改成 PN-Link venv 启动。

### 7.2 通用化 bridge

保留旧 console script 和参数，避免 PICO 部署回归，同时增加无 PICO 命名的别名：

```text
sonic_pose_to_smpl_ref_bridge
```

内部类名和日志逐步从 `PicoSourceReadinessGate` 泛化为 `PoseSourceReadinessGate`，但第一版
不要求删除旧导入名；旧名可以作为兼容 alias 保留。

bridge 新增：

- 直接 `wrist` 字段解析；
- `--source-kind pico|pnlink`，默认 `pico`；
- 根据 source kind 选择 finite check 和日志文本；
- `--pose-host` 作为 `--pico-host` 的通用别名；
- 通过共享 `DiagnosticReporter` 发布 `SMPL_REF_BRIDGE` stage 的 decode、readiness、frame
  merge 和 stale 事件；
- 原有 5557 `smpl_ref` 输出完全不变。

### 7.3 进程管理

增加以下环境配置：

```text
SONIC_TELEOP_SOURCE=pico|pnlink           # 默认 pico
SONIC_PNLINK_PORT=5556                    # 默认 5556
SONIC_PNLINK_PYTHON=/opt/bxi/venvs/sonic_pnlink/bin/python
SONIC_PNLINK_SDK_PATH=/opt/bxi/vendor/pnlink/mocapapi_python
SONIC_PNLINK_DIAGNOSTICS=0|1              # 默认 0
SONIC_PNLINK_MUJOCO_VIEWER=0|1            # 默认 1，随脚本启动
```

行为：

- `pico` 保持现有 ROS supervisor 的 manager + bridge 生命周期；
- `pnlink` 时 ROS supervisor 不创建 PN-Link 进程、service 或 topic；
- `script/run_sonic_pnlink_sources.sh` 启动纯 Python bridge 后在前台运行 source，source
  独占当前终端按键；
- bridge 未收到 live 数据时不发布新 `smpl_ref`，SONIC 按原逻辑回到
  `idle_reference`；
- source 异常退出或按 ESC 退出时，脚本清理 bridge，禁止残留窗口继续 live；
- 再次启动脚本需要重新执行厂商标定和 T 姿态标定。

### 7.4 部署配置

机器人电脑安装 PN-Link SDK 到部署目录，并配置同一台电脑的运行环境：

```bash
export SONIC_TELEOP_SOURCE=pnlink
export SONIC_PNLINK_PYTHON=/opt/bxi/venvs/sonic_pnlink/bin/python
export SONIC_PNLINK_SDK_PATH=/opt/bxi/vendor/pnlink/mocapapi_python

export PNLINK_LOCAL_IP=10.42.0.101
export PNLINK_LOCAL_PORT=8002
export PNLINK_SERVER_IP=10.42.0.202
export PNLINK_SERVER_PORT=8080

export SONIC_PNLINK_BIND_HOST=127.0.0.1
export SONIC_PNLINK_PORT=5556
export SONIC_PNLINK_DIAGNOSTICS=1
export SONIC_PNLINK_DEBUG_HOST=127.0.0.1
export SONIC_PNLINK_DEBUG_PORT=5558
export SONIC_PNLINK_MUJOCO_VIEWER=1
export SONIC_PNLINK_VIEWER_RENDER_HZ=30
export SONIC_PNLINK_DIAG_DIR=/tmp/sonic_pnlink_diagnostics
export BXI_SONIC_GRIPPER_ENABLE=0

./script/run_sonic_pnlink_sources.sh
```

本地开发时，如果 `PNlink_mocap` 与本仓库位于同一父目录，
`script/setup_sonic_local_env.sh` 会自动将 `SONIC_PNLINK_SDK_PATH` 指向
`../PNlink_mocap/mocapapi_python`，并使用当前 `.venv_teleop` 作为
`SONIC_PNLINK_PYTHON`。生产部署仍应使用上述 `/opt/bxi` 固定路径。

source 直接加载厂商 `MCPApplication`/`MCPSettings`，以 50 Hz 调用
`poll_next_event()`。`CommandStartCapture` 和 `CommandCalibrateMotion` 都是异步命令：
按键成功只表示命令已进入厂商队列；source 只有收到成功的 `CommandReply` 后才会
确认命令完成，标定状态才会从 `VENDOR_CALIBRATING` 进入 `NEEDS_NEUTRAL`。

`PNLINK_LOCAL_IP` 必须真实配置在机器人电脑的网卡上。若机器人原控制网卡已有其他网段，
优先使用第二块网卡；也可以在不会改变默认路由的前提下增加 `10.42.0.101/24` secondary
address。部署检查必须确认：

```bash
ip -brief address
ip route get 10.42.0.202
```

到 `10.42.0.202` 的路由必须走 PN-Link 接口，不能覆盖机器人控制、ROS 或维护网络的
默认路由。5556、5557 和 5558 都只绑定 loopback，不增加防火墙入站规则，也不允许
配置为 `0.0.0.0`。

### 7.5 Controller 侧诊断

`SonicTeleopPolicy` 不是 ROS node。为它增加可选的 `diagnostic_reporter` callback，由
`bxi_example_demo` 所属 node 创建并传入；测试、离线推理或未启用 diagnostics 时可以为
`None`，不改变现有构造路径。

当前 `poll_reference()` 对 decode/shape/nonfinite 错误直接丢弃。第一版保留“拒绝错误包并
继续 idle/live fallback”的控制行为，但在每次拒绝时通过 callback 发布
`POLICY_INPUT` issue，至少区分：

```text
source metadata rejected
term1_local shape/nonfinite
root_quat shape/nonfinite/norm
wrist shape/nonfinite
wire decode failure
```

成功接受新 reference 时发布 `POLICY_INPUT=OK` 和对应 frame index。Reporter 自身异常必须
被隔离，不能让 policy inference 抛错或停止电机控制。

## 8. MuJoCo 实时双骨架与分阶段诊断

### 8.1 诊断进程边界

新增独立 console script：

```text
sonic_pnlink_mujoco_viewer
```

viewer 不嵌入 PN-Link SDK 回调，也不在 source 的 50 Hz 转换线程中调用 MuJoCo。source
通过独立的非阻塞 ZMQ PUB socket 发布单帧阶段快照：

```text
tcp://127.0.0.1:5558
topic: pnlink_debug
rate: 50 Hz source data, viewer renders latest frame at 30 Hz
ZMQ SNDHWM/RCVHWM: 1
```

viewer 永远只取最新 debug frame。启动后立即显示 PN-Link 原始骨架；按 `T` 完成 25 帧
中立位标定后，才在同一场景加入 SMPL-24 骨架。viewer 处理不过来时丢弃可视化帧，不反压 source。
debug socket send 失败、无订阅者或 viewer 退出只增加诊断计数，不改变 pose 发布和机器人
状态。viewer 默认由纯 Python 启动脚本启动；无须显示时可显式关闭：

```text
SONIC_PNLINK_MUJOCO_VIEWER=0
SONIC_PNLINK_DEBUG_HOST=127.0.0.1
SONIC_PNLINK_DEBUG_PORT=5558
SONIC_PNLINK_VIEWER_RENDER_HZ=30
```

viewer 只订阅 5558 `pnlink_debug`，不初始化 ROS 2。debug frame 提供 source 各阶段的
骨架数组和有效性 mask；policy 侧 diagnostics 仍属于机器人主控，不进入 PN-Link 数据链。

viewer 使用 `nice=10` 的较低调度优先级。没有可用的 `DISPLAY`/OpenGL 上下文时，viewer
启动失败只记录 WARN，source 和 bridge 继续运行；Sim2Sim 和可视化验收环境必须提供
可工作的 MuJoCo viewer。

### 8.2 Debug frame 数据契约

debug frame 是单个源帧在各处理阶段的只读快照，不复用 10 帧 `pose` wire contract。
它使用固定版本的 binary-array header，关节顺序由 source 和 viewer 共同导入的
`skeleton.py` 常量定义。每个包至少包含：

| Field | dtype | shape | 阶段/用途 |
|---|---|---|---|
| `schema_version` | `int32` | `[1]` | 第一版固定为 1 |
| `frame_index` | `int64` | `[1]` | 当前源帧 |
| `timestamp_monotonic` | `float64` | `[1]` | 单机延迟测量 |
| `raw_local_pos_cm` | `float32` | `[J,3]` | SDK 原始局部位置，厘米 |
| `raw_local_quat` | `float32` | `[J,4]` | SDK 原始局部旋转，wxyz |
| `raw_world_pos` | `float32` | `[J,3]` | 坐标转换和 PN-Link FK 后世界位置，米 |
| `raw_world_quat` | `float32` | `[J,4]` | 坐标转换和 PN-Link FK 后世界旋转，wxyz |
| `retarget_global_quat` | `float32` | `[24,4]` | 中立标定和 SMPL 全局映射后旋转 |
| `smpl_local_axis_angle` | `float32` | `[22,3]` | 送入人体 FK 的 root + body pose |
| `smpl_joints` | `float32` | `[24,3]` | 最终根坐标系 SMPL 骨架 |
| `smpl_root_quat` | `float32` | `[4]` | 最终 SONIC 根旋转 |
| `wrist` | `float32` | `[6]` | 最终 ELF3 wrist 参考 |
| `raw_present` | `bool` | `[J]` | 原始关节是否存在 |
| `raw_position_valid` | `bool` | `[J]` | 原始/FK 位置是否有效 |
| `raw_rotation_valid` | `bool` | `[J]` | 原始/FK 旋转是否有效 |
| `smpl_mapping_valid` | `bool` | `[24]` | 每个 SMPL 映射是否有效 |
| `smpl_position_valid` | `bool` | `[24]` | SMPL FK 位置是否有效 |
| `smpl_rotation_valid` | `bool` | `[24]` | SMPL 局部/全局旋转是否有效 |
| `source_stage_valid` | `bool` | `[7]` | source 拥有的前七个阶段汇总状态 |

`J` 是 PN-Link 固定骨架关节数。缺失或非法浮点值在 debug frame 中仍保留有效性 mask；
数组中的非法值不能直接交给 MuJoCo。viewer 必须用每个关节最后一次有效值替代不可渲染
值，并以错误颜色标记，原始非法值只写入诊断 bundle。

debug frame 只用于观察，不作为 pose、bridge 或 policy 的输入，防止诊断功能改变控制链。

### 8.3 同场景双骨架布局

MuJoCo viewer 在同一个 world 中同时绘制两套骨架：

```text
viewer left / +Y:  RAW PN-LINK
viewer right / -Y: RETARGETED SMPL-24
```

默认显示规则：

- 原始骨架使用 PN-Link 实际骨长和 `raw_world_pos`；
- 这里的 raw 明确定义为“只经过单位/坐标转换和 PN-Link FK、尚未经过 neutral/SMPL
  retarget”的源骨架；
- SMPL 骨架使用 `smpl_joints` 的标准人体比例；
- 两套骨架分别减去自身 pelvis/`Hips` 的水平位置，再加显示专用的 Y 方向偏移；
- 只改变显示平移，不改变任何关节旋转、相对位置或发布数据；
- overlay 显示原始 `Hips` 世界位置，保留操作者绝对移动信息；
- 原始骨架使用 PN-Link parent tree，SMPL 使用标准 24 关节 parent tree；
- 合成的 `LeftTiptoe/RightTiptoe` 使用灰色虚线骨段，避免误认为 SDK 真实测量；
- 显示世界 XYZ 坐标轴、两套 pelvis 坐标轴和当前选中关节坐标轴。

正常颜色固定为：

```text
raw valid joint/bone      cyan
SMPL valid joint/bone     green
warning                   yellow
position error origin     red sphere
rotation error origin     orange sphere + magenta rotation axes
position and rotation     white sphere + red/magenta axes
downstream affected       dark red/gray
last-valid frozen pose    50% opacity
```

当选中一个 SMPL 关节时，viewer 绘制到其 PN-Link source joint 的细黄色连线，并在
overlay 中显示 mapping，例如：

```text
SMPL[18] left_elbow <- PN-Link LeftForeArm
```

### 8.4 非法数据的画面行为

viewer 必须区分“原发错误”和“被上游错误传播影响”：

- `LeftArm` 原始 rotation 非法时，只把 `LeftArm.rotation` 报为 origin；
- `LeftForeArm`、`LeftHand` 等后代的 FK 结果标记为 downstream affected，并引用
  `caused_by=SDK_RAW/LeftArm/rotation`；
- 不为每个后代重复生成同一个根因的 ERROR；
- 原始 position 非有限时，在最后有效位置显示红色球，不把 NaN 写进 MuJoCo scene；
- rotation 非法但 position 仍可显示时保留关节球，隐藏非法坐标轴并使用橙色标记；
- `SMPL_RETARGET` 出错时，原始骨架仍按其自身 validity 显示，SMPL 侧冻结最后有效姿态；
- `SMPL_FK` 出错时，映射后的旋转可在 overlay 查看，但 SMPL position 使用最后有效值；
- `TEMPORAL_BUFFER`、`POSE_WIRE`、`SMPL_REF_BRIDGE` 或 `POLICY_INPUT` 出错时，两套骨架
  可保持绿色，但 pipeline 状态条显示对应下游阶段为红色，避免错误归因到人体关节。

viewer 顶部状态条固定显示：

```text
source state | source Hz | frame | frame age | calibration | live
SDK_RAW | PNLINK_FK | CALIBRATION | SMPL_RETARGET | SMPL_FK
TEMPORAL_BUFFER | POSE_WIRE | SMPL_REF_BRIDGE | POLICY_INPUT
```

右侧 issue 列表显示最近 10 个不同问题，格式固定为：

```text
[ERROR] stage=SMPL_RETARGET frame=1842
joint=LeftForeArm -> SMPL[18]/left_elbow
field=rotation code=ROTATION_NONFINITE action=FRAME_DROPPED
```

### 8.5 结构化诊断事件

PN-Link source 和 bridge 将诊断写入前台日志及本地 bundle，不发布 ROS topic。只有机器人
controller 中的 policy 输入诊断继续使用现有标准 ROS 诊断 topic：

```text
/sonic_pnlink/diagnostics    diagnostic_msgs/msg/DiagnosticArray
```

每个 policy `DiagnosticStatus` 表示一个明确的问题或阶段汇总：

```text
name:        sonic_pnlink/<stage>/<joint-or-component>/<field>
hardware_id: sonic_policy
level:       OK | WARN | ERROR | STALE
message:     stable issue code
```

`values` 必须包含以下 key；没有对应关节时使用空字符串或 `-1`：

| Key | 含义 |
|---|---|
| `frame_index` | 问题首次出现的源帧 |
| `stage` | 第 8.6 节固定 stage 名称 |
| `code` | 稳定、可测试的错误代码 |
| `field` | `position`、`rotation`、`timestamp`、`shape`、`wire` 或 `state` |
| `source_joint` | PN-Link joint 名称 |
| `source_parent` | PN-Link parent 名称 |
| `smpl_index` | SMPL index |
| `smpl_joint` | SMPL joint 名称 |
| `origin` | `true` 表示原发错误，`false` 表示传播影响 |
| `caused_by` | 上游 origin 的稳定路径 |
| `observed` | 实际值或简短统计量 |
| `expected` | 期望范围、shape 或 dtype |
| `action` | `NORMALIZED`、`FRAME_DROPPED`、`LIVE_REVOKED` 等 |
| `first_seen_ns` | 首次出现时间 |
| `last_seen_ns` | 最近出现时间 |
| `occurrences` | 合并后的累计次数 |

相同 `stage/code/joint/field` 的连续问题每秒最多发布一次，但 `occurrences` 必须持续累加；
状态恢复时发布一次 `OK`，包含错误持续时间和总次数。日志使用同一 issue code，不再生成
无法与 viewer/bundle 对照的自由格式错误文本。

### 8.6 阶段定义与校验规则

固定九个 stage，所有异常必须归入其中之一：

| Stage | 输入/输出 | 必须执行的校验 | 典型 issue code |
|---|---|---|---|
| `SDK_RAW` | SDK joint local pose | joint presence、position finite、quaternion finite/norm、原始时间推进 | `JOINT_MISSING`, `POSITION_NONFINITE`, `QUATERNION_NORM` |
| `PNLINK_FK` | PN-Link world pose | parent 有效、world position/quaternion finite、旋转正交性、骨长连续性 | `PARENT_INVALID`, `FK_NONFINITE`, `BONE_LENGTH_JUMP` |
| `CALIBRATION` | 25 帧 neutral | 样本数、1 秒 deadline、每关节 5 度稳定性、平均四元数有效 | `NEUTRAL_UNSTABLE`, `CALIBRATION_TIMEOUT` |
| `SMPL_RETARGET` | `D/S/L_smpl` | mapping 存在、matrix finite/orthogonal、axis-angle finite、provenance 可解析 | `MAPPING_MISSING`, `ROTATION_NONFINITE`, `ROTATION_NOT_ORTHOGONAL` |
| `SMPL_FK` | SMPL joints/root/wrist | exact shape、finite、root norm、固定模型骨长、wrist finite | `SMPL_FK_NONFINITE`, `ROOT_QUATERNION_INVALID`, `WRIST_NONFINITE` |
| `TEMPORAL_BUFFER` | 50 Hz frames/window | timestamp 单调、frame 递增、gap 小于 0.1 秒、窗口恰好 10 帧 | `TIMESTAMP_REGRESSION`, `SOURCE_STALE`, `WINDOW_INCOMPLETE` |
| `POSE_WIRE` | 5556 pose | topic、version、字段、dtype、shape、payload bounds | `POSE_SCHEMA_INVALID`, `POSE_SHAPE_INVALID` |
| `SMPL_REF_BRIDGE` | merge/readiness/5557 | 3 包 gate、chunk frame 对齐、merge 输出 exact shape、stale 0.2 秒 | `SOURCE_NOT_READY`, `FRAME_REGRESSION`, `MERGE_INVALID` |
| `POLICY_INPUT` | Sonic `_frame_from_fields` | source metadata、三个输入 exact shape/finite、root quaternion norm | `REFERENCE_REJECTED`, `POLICY_SHAPE_INVALID`, `POLICY_ROOT_INVALID` |

字段级规则：

- position 和 rotation 独立校验并独立记录，不用一个笼统的 `joint invalid`；
- SDK quaternion norm 与 1.0 偏差超过 0.02 时发 WARN 并归一化；norm 小于 0.5、
  大于 1.5 或包含非有限值时发 ERROR 并丢弃整帧；
- 非根 local bone length 相对 neutral 中位数偏差超过 10% 发 WARN，超过 30% 发 ERROR；
- `Hips` 相邻原始位置跳变超过 1 米发 ERROR；
- 任一关节相邻有效帧旋转角跳变超过 90 度发 ERROR；
- 旋转矩阵 `abs(det(R)-1) > 1e-3` 或 `||R^T R-I|| > 1e-3` 发 ERROR；
- SMPL axis-angle norm 必须不大于 `pi + 1e-4`；
- SMPL 固定模型骨长与该模型 neutral 骨长偏差超过 `1e-4` 米发 ERROR；
- WARN 可以继续处理并在 viewer 标黄；ERROR 丢弃当前源帧，保持最后有效控制数据，并按
  现有 freshness timeout 撤销 live，不能用错误帧更新 10 帧窗口。

### 8.7 可复现诊断 bundle

诊断开启时，各阶段快照进入内存 ring buffer：

```text
100 frames before first ERROR (2 seconds at 50 Hz)
50 frames after first ERROR  (1 second at 50 Hz)
```

第一个 ERROR 或 viewer 中手动 dump 会生成：

```text
${SONIC_PNLINK_DIAG_DIR}/<session>/<timestamp>_<stage>_<code>/
  manifest.json
  issues.jsonl
  raw_frames.npz
  retarget_frames.npz
  output_frames.npz
```

默认目录为 `/tmp/sonic_pnlink_diagnostics`。`manifest.json` 包含代码版本、schema version、
环境配置、关节映射、neutral 标定摘要、触发 issue 和每个数组的 shape/dtype。所有原始
NaN/Inf 必须保存在 bundle 中，不能只保存 viewer 使用的 last-valid 替代值。

写盘由独立 worker 完成，队列长度固定为 2；队列满时丢弃新的 bundle 请求并发布
`DIAGNOSTIC_DUMP_DROPPED` WARN，不能阻塞 source。相同 issue key 五秒内不重复自动 dump。
每个 session 最多保留 20 个 bundle，超过后删除最旧 bundle，避免填满机器人磁盘。
source 在 post-error 50 帧收集完成前退出时，保存已经收集到的帧并标记
`post_frames_incomplete=true`。

viewer 提供以下诊断操作：

```text
Space       pause/resume viewer only
Left/Right step through the in-memory 2-second viewer history
Mouse click select the nearest raw/SMPL joint
[/]         cycle through joints with active issues
E           jump to latest ERROR frame
D           dump current frame/history bundle
F           focus camera on selected invalid joint
```

暂停 viewer 不暂停 source 或机器人控制。

### 8.8 可视化与诊断验收

必须用录制回放自动注入以下故障并在终端日志、viewer、bundle 三方对照：

1. `LeftForeArm.position=NaN`：raw 左肘位置红色，stage=`SDK_RAW`、field=`position`；
2. `RightArm.quaternion=0`：raw 右肩橙色，右臂后代标记 affected，origin 只有一个；
3. 删除 `Spine1`：显示 `JOINT_MISSING`，SMPL spine2 映射冻结并指出 source joint；
4. 构造非正交 retarget matrix：raw 保持正常，SMPL 侧标红，stage=`SMPL_RETARGET`；
5. 让 `process_smpl_joints()` 返回 NaN：stage=`SMPL_FK`，不错误归因到 `SDK_RAW`；
6. 重复 frame index：两套骨架仍可显示，pipeline 条在 `TEMPORAL_BUFFER` 标红；
7. 破坏 pose shape：`POSE_WIRE` 标红，bridge 不发布 ready；
8. 制造 merge frame regression：`SMPL_REF_BRIDGE` 标红并停止 5557 live；
9. 向 policy 注入错误 root quaternion：`POLICY_INPUT` 标红并拒绝该 reference；
10. 关闭 viewer：source 仍保持 50 Hz，pose/smpl_ref 和控制频率不受影响。

每个案例必须能从 issue 中同时回答：哪个 stage、哪个 source joint、对应哪个 SMPL joint、
position 还是 rotation、原发还是传播、采取了什么动作，并能从 bundle 离线复现。

## 9. 安全策略

第一版必须满足以下 fail-closed 行为：

- source 启动后默认 paused，不自动进入 live；
- 标定失败、骨架缺失、NaN、非法四元数和超时都停止 live 发布；
- 单关节相邻 20 ms 角度跳变超过 90 度时丢弃整帧；
- 连续 3 个合法、递增、已标定的 pose 包之后 bridge 才声明 ready；
- pose 超过 0.2 秒不更新时 bridge 停止 `smpl_ref`；
- Sonic 超过自身 live timeout 后清除旧引用并平滑回到 `idle_reference`；
- source 或 bridge 重启后不能复用上一会话的 10 帧窗口；
- 进入 live 前必须能从终端 JSON 状态行明确看到 `LIVE` 和正常 source Hz；
- MuJoCo 只消费经过复制的 debug frame，不能持有或修改 source 控制缓冲区；
- 任何 NaN/Inf 都必须在写入 MuJoCo scene 前替换为 last-valid 显示值；
- viewer 关闭、卡顿或诊断 bundle 写盘失败不能降低 source/bridge/control 频率；
- bundle 数量和写盘队列必须有硬上限，防止诊断功能耗尽内存或磁盘；
- 第一版禁止夹爪控制，所有验收环境固定 `BXI_SONIC_GRIPPER_ENABLE=0`。

PN-Link 数据异常不直接触发机器人 `zero_torque`，因为现有 SONIC 策略已经定义了
live-to-idle 平滑回退。是否在更长时间故障后退出 `sonic_teleop` 属于后续独立安全策略，
不在第一版中隐式新增。

## 10. 测试计划

### 10.1 单元测试

为纯数学代码构造无硬件测试：

1. 坐标转换：`(100,200,300) cm -> (3,1,2) m`；单位四元数保持单位；
2. 层级 FK：父节点旋转能够正确影响子节点全局旋转；
3. SMPL 映射：左右髋、膝、肩、肘和腕不会交换；
4. 缺失 toe：继承 ankle 后对应 SMPL foot 局部旋转为单位旋转；
5. 中立标定：稳定 T 姿态成功，超过 5 度抖动失败；
6. 四元数 `q/-q` 输入得到连续输出；
7. 中立根经过 `process_smpl_joints()` 后接近 `[1,0,0,0]`；
8. 操作者整体 yaw 只改变根参考，不产生明显四肢伪旋转；
9. 单独弯左肘只主要改变左肘/左腕链，右臂保持不变；
10. wrist 共享函数对相同 SMPL pose 产生与当前 PICO 代码一致的六维结果；
11. `pose` pack/decode 后 dtype、shape 和数值保持一致；
12. PN-Link 直接 wrist 和 PICO legacy `joint_pos` 两条 bridge 路径都通过。
13. 每个 stage 的 issue 能定位到 source/SMPL joint 和 position/rotation 字段；
14. 上游 origin 只报告一次，下游 affected joint 正确引用 `caused_by`；
15. debug frame 中的 NaN 不进入 MuJoCo geom，viewer 使用 last-valid pose；
16. viewer 无订阅、处理变慢或退出时，source 发送路径保持非阻塞；
17. 自动 bundle 包含 100 个 pre-error 帧和最多 50 个 post-error 帧，并按 20 个上限轮转。

### 10.2 录制数据回放

source 增加仅用于诊断的 `--record-dir` 和 `--replay`：

- 原始记录保存 PN-Link joint names、局部位置、局部四元数和时间戳；
- 标准化记录保存最终 pose payload；
- 回放不加载 PN-Link `.so`，便于 CI 和机器人外调试；
- 录制格式使用 `.npz`，字段固定，不使用 pickle。

至少录制以下动作：

```text
T pose neutral
左臂抬起/放下
右臂抬起/放下
左右肘分别弯曲
左右手腕分别转动
左右腿分别屈膝
原地深蹲
身体左转/右转
短暂断开 PN-Link 数据
```

回放验收：所有输出有限、帧号连续、左右方向正确、断流后不继续生成窗口。

### 10.3 ZMQ 集成测试

不启动机器人控制器，单独运行 source 和 bridge，持续采样 10 秒：

```text
5556 pose:
  rate >= 45 Hz
  stream_mode = 1
  calibration_ready = true
  frame_delta = 1
  smpl_joints = (10,24,3)
  body_quat_w = (10,4)
  wrist = (10,6)

5557 smpl_ref:
  rate >= 45 Hz
  source_ready = true
  source_stream_mode = 1
  frame_delta = 1
  term1_local = (10,72)
  root_quat = (10,4)
  wrist = (10,6)
```

随后执行：

- 按 `P`，确认 5557 在 0.2 秒内停止 live；
- 按 `L`，确认连续 3 个有效包后恢复；
- 停止 source，确认 bridge 不重放旧窗口；
- 重启 source、帧号从 0 开始，确认 bridge 能建立新会话；
- 制造一个 NaN 和一个 90 度以上突变帧，确认该帧不会进入 5557。

再验证纯 Python 进程生命周期：

1. 启动脚本后 source 占用 5556、bridge 占用 5557，ROS graph 中没有新增 PN-Link node、
   service 或 topic；
2. 按 `N`、`C`、`T` 完成采集、厂商标定和 T 姿态标定，source 保持
   `READY_PAUSED`；
3. 按 `L` 后进入 live，进入 `sonic_teleop` 时策略切换到 `live_reference`；
4. 按 `P` 后清空 source 窗口，bridge 在 stale 门限内停止 live 输出；
5. 按 `L` 可复用本次穿戴标定重新进入 live；
6. 按 ESC，确认 source/bridge/viewer 进程全部退出，5556/5557/5558 均无 owner。

诊断启用时还必须确认 5558 只监听 loopback，viewer 同时显示两套骨架，关闭 viewer 后
5556/5557 频率和 controller 频率不变；停止脚本后 5558 也必须释放。

### 10.4 MuJoCo / Sim2Sim

先运行新增的 MuJoCo 双骨架 viewer，确认 raw 与 SMPL 姿态、左右映射和 stage 状态条
正确，再连接 SONIC Sim2Sim：

1. 无 PN-Link live 时进入 `sonic_teleop`，状态必须为 `idle_reference`；
2. 按 `N`、`C`、`T`、`L` 完成采集、厂商标定、中立标定和 live；
3. 状态变为 `live_reference`，模拟机器人跟随人体动作；
4. 验证左右、前后、屈伸和手腕方向；
5. 暂停/断流后状态依次为 `live_stale_to_idle`、`idle_reference`；
6. 从 SONIC 切回 normal 后按 `P`，策略平滑回 idle；
7. 再次进入 SONIC 前按 `L`，连续 3 包 gate 后恢复 live。

Sim2Sim 连续运行 10 分钟，不得出现 source/bridge 崩溃、帧号倒退、内存持续增长或
重复进程占用端口。

### 10.5 实机分阶段验收

实机测试必须按顺序进行：

1. 机器人悬挂、急停可触达，夹爪禁用；
2. 只进入 SONIC idle，确认 50 Hz 控制和状态机正常；
3. source live 后先保持 T 姿态，确认没有突然大幅跳变；
4. 分别做小幅左臂、右臂、肘、腕动作；
5. 再做小幅屈膝、深蹲和转身；
6. 主动暂停 source，确认机器人平滑回 idle；
7. 停止 PN-Link server 或断开 PN-Link 专用接口，重复失联测试；不得断开机器人控制网；
8. 离开 SONIC，确认 bridge 和 5557 清理；
9. 悬挂测试全部通过后，才进入经批准的落地低幅度测试。

任何一步出现左右反向、根朝向突跳、单腿异常抬起、明显高频抖动或失联后继续动作，
都停止后续阶段并回到录制数据/Sim2Sim 排查。

## 11. 验收标准

第一版完成必须同时满足：

- PICO 未连接、PICO manager 和 RoboticsService 未运行时，PN-Link 可以独立驱动
  `live_reference`；
- PN-Link source 稳定输出不低于 45 Hz，目标均值为 50 Hz；
- `pose` 和 `smpl_ref` 所有字段 shape、dtype、四元数顺序符合本文契约；
- T 姿态进入 live 时无明显姿态突跳；
- 左右臂、左右腿和手腕方向在可视化、Sim2Sim、悬挂实机三层一致；
- source pause、PN-Link UDP 断流或 source 进程退出后，0.2 秒内停止新的 live
  `smpl_ref`；
- Sonic 随后进入 `live_stale_to_idle` 并回到 `idle_reference`，不冻结旧动作；
- source 重启和帧号归零可以恢复，不复用上一会话窗口；
- 按 ESC 停止纯 Python 运行脚本后 bridge 被清理，没有残留 5557 owner；
- PICO 模式的现有测试全部继续通过；
- `BXI_SONIC_GRIPPER_ENABLE=0`，没有 PN-Link 手指或夹爪控制副作用；
- 使用同一台电脑的单调时钟测量，端到端 source-to-policy 延迟小于 200 ms；
- 同时运行硬件主控、controller、source 和 bridge 时，硬件与 SONIC 控制频率仍稳定在
  50 Hz，不能因 Torch 线程竞争持续降频；
- MuJoCo 同一场景持续、同时显示原始 PN-Link 和最终 SMPL-24 骨架，左右和父子层级正确；
- 注入任一第 8.8 节故障时，viewer、终端 diagnostics 和保存 bundle 对 stage、joint、
  position/rotation、origin/affected 的判断一致；
- viewer 关闭或故障时 source、bridge、SONIC 仍满足原有频率和 stale 回退标准；
- Sim2Sim 连续运行 10 分钟无崩溃，实机悬挂测试按第 10.5 节通过。

## 12. 实施顺序

按以下提交顺序开发，每一步保持可独立测试：

1. 提取共享 wrist 数学函数，并证明 PICO 输出不变；
2. 扩展 bridge 支持直接 `wrist` 和 `source-kind=pnlink`，补兼容测试；
3. 实现 PN-Link 骨架、坐标转换和纯数学 FK/SMPL retarget；
4. 实现中立姿态标定及其稳定性检查；
5. 实现九阶段 validator、结构化 issue、provenance 和 last-valid 缓冲；
6. 实现单帧 debug wire、诊断 ring buffer、bundle worker 和录制/回放；
7. 实现同场景原始/SMPL MuJoCo viewer，并完成第 8.8 节故障注入；
8. 实现 10 帧 pose ZMQ publisher，接入 PN-Link SDK 生命周期与终端单键状态机；
9. 实现纯 Python 前台运行脚本，统一清理 source、bridge 和可选 viewer；
10. 完成录制回放、ZMQ、diagnostics 和无 viewer 回归测试；
11. 完成 Sim2Sim 和单机资源占用验收；
12. 更新部署/诊断脚本后执行悬挂实机验收。

第一版不得在上述步骤中顺带加入夹爪、手指重定向、自动标定、自动 live、跨电脑
PN-Link 数据传输或新的机器人故障状态跳转。这些内容应在全身链路通过验收后单独设计。
