PN-Link 驱动 SONIC Sim2Sim 需要三个终端。所有命令都在仓库根目录执行。

### 一次性构建

代码更新后先构建：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select bxi_example_py_elf3 remote_controller
```

### 终端1：启动仿真和 SONIC 控制器

```bash
cd /media/wp/新加卷/yuelk_project/ATEC/bxi_rl_controller_ros2_example_python

SONIC_TELEOP_SOURCE=pnlink \
bash script/run_sonic_bxi_sim2sim.sh
```

等待 MuJoCo ELF3 仿真窗口出现。`SONIC_TELEOP_SOURCE=pnlink` 很重要，它防止 launch 自动启动 PICO manager。

### 终端2：启动状态机遥控器

```bash
cd /media/wp/新加卷/yuelk_project/ATEC/bxi_rl_controller_ros2_example_python

bash script/run_sonic_sim2sim_controller.sh
```

保持该终端焦点，依次按：

```text
!  → pd_brake
1  → normal
```

进入 `normal` 后，MuJoCo 中的机器人应解除悬挂并进入正常站立状态。暂时不要按 `6`。

### 终端3：启动 PN-Link 数据链

```bash
cd /media/wp/新加卷/yuelk_project/ATEC/bxi_rl_controller_ros2_example_python

SONIC_TELEOP_SOURCE=pnlink \
bash script/run_sonic_pnlink_sources.sh
```

等待出现：

```text
state:"CAPTURING"
sdk_connected:true
source_hz:...
```

如果没有自动开始采集，按一次 `N`。

不做厂商标定时：

```text
保持标准 T 姿态
→ 轻按一次 T
→ 等待 READY_PAUSED
→ 按 L
```

标定成功应看到：

```text
PN-Link neutral calibration complete; bone_length_baselines=20
state:"READY_PAUSED"
neutral_calibrated:true
```

按 `L` 后应看到：

```text
state:"LIVE"
live_enabled:true
```

bridge 随后应从：

```text
waiting for calibrated, fresh POSE frames received=0
```

变成：

```text
[pnlink->smpl_ref] sent ... received=... skipped=0
```

### 进入 SONIC

回到终端2，按：

```text
6  → sonic_teleop
```

终端1应出现：

```text
[SONIC] reference status: live_reference
```

此时操作者缓慢移动，MuJoCo 中的 ELF3 应跟随 PN-Link 动作。

完整按键顺序是：

```text
终端2：! → 1
终端3：T → 等待完成 → L
终端2：6
```

### 状态确认

新开终端可检查仿真状态：

```bash
source script/setup_sonic_local_env.sh
ros2 topic echo --once /simulation/state_machine_info std_msgs/msg/String
```

应包含：

```text
sonic_teleop
```

正常运行标准：

- PN-Link：`state=LIVE`
- bridge：`sent` 和 `received` 持续增加，`skipped=0`
- SONIC：`reference status: live_reference`
- 没有持续的 `ROTATION_JUMP`、`SOURCE_STALE` 或骨长 `ERROR`

### 停止顺序

先在终端2按：

```text
1  → normal
```

然后：

```text
终端3：P → ESC
终端2：Ctrl+C
终端1：Ctrl+C
```

如果机器性能不足、两个 MuJoCo 窗口负载较高，可关闭 PN-Link 骨架 viewer：

```bash
SONIC_PNLINK_MUJOCO_VIEWER=0 \
SONIC_TELEOP_SOURCE=pnlink \
bash script/run_sonic_pnlink_sources.sh
```