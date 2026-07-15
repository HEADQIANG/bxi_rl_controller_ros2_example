# SONIC teleoperation on ELF3

## Scope

This branch adds SONIC as an ELF3 state-machine action. The existing official
back-flip, forward-flip, ballet, walking, recovery and gesture actions remain
unchanged. The SONIC controller still publishes the official 29-joint
`ActuatorCmds` interface.

The source branch is not the internal offline deployment archive. It excludes
XRT binaries, offline wheels, build/install/log output, internal reports and
experimental gripper CAD. `bxi_example_bms` is optional battery telemetry and
is not required by T1, T2 or T3.

## Dependencies

Build the official workspace dependencies first, then install:

```bash
python3 -m pip install -r script/sonic_runtime_requirements.txt
colcon build --packages-select bxi_example_py_elf3 remote_controller
```

Real PICO operation also requires the robot vendor's
`xrobotoolkit_sdk` binding and `/opt/apps/roboticsservice/RoboticsServiceProcess`.
Those files are deliberately not stored in GitHub. PyVista/VTK and the G1 CAD
meshes are not needed by the default headless T3 path; `PICO_ENABLE_VIS=1` is
not supported by this source-only dependency subset.

## Sim2Sim

Open three terminals from the repository root after sourcing ROS and the
workspace install.

T1 — MuJoCo and the BXI controller:

```bash
bash script/run_sonic_bxi_sim2sim.sh
```

T2 — keyboard/remote controller:

```bash
bash script/run_sonic_sim2sim_controller.sh
```

T3 — PICO manager and PICO-to-SMPL bridge:

```bash
bash script/run_sonic_pico_sources.sh
```

Keyboard state flow is `!` (PD brake), `1` (normal), then `6` (SONIC).
Back-flip moves from keyboard key `6` to `0`; its gamepad mapping is unchanged.
The SONIC gamepad mapping is `RT + X`.

T3 owns both child process groups and escalates shutdown from SIGINT to SIGTERM
and SIGKILL if necessary. It uses port 5556 for PICO pose, port 5557 for
`smpl_ref`, and the external XRT service normally uses port 60061.

## Runtime policy

- SONIC requires a fresh live `smpl_ref` by default. Until one arrives it holds
  the policy default pose and reports `waiting_for_live_smpl_ref`.
- SONIC intentionally does not use the common approximately 60-degree
  roll/pitch transition to `zero_torque`; other states retain that protection.
- Gripper CAN control is present as an optional integration and is disabled by
  default. Enable it only after validating bus IDs and PICO trigger topics with
  `BXI_SONIC_GRIPPER_ENABLE=1` and `PICO_ENABLE_ROS_BUTTONS=1`.
- The PICO manager retains the validated G1 legacy FK calibration. The ELF3 FK
  helper is included for A/B evaluation but is not the default calibration.

## Model and references

Default files are installed below the ROS package share directory:

```text
data/sonic_model/elf3_step28800_smpl/model_step_028800_smpl.onnx
data/sonic_reference/elf3_step28800_idle_left_001_A019/stream_reference.npz
```

They can be overridden with `BXI_SONIC_MODEL_ONNX` and
`BXI_SONIC_STREAM_REFERENCE_NPZ`. See `THIRD_PARTY_NOTICES.md` for model
license, attribution, cleanup provenance and SHA256.

## Validation status

The local Sim2Sim chain, clean source build/install, cleaned ONNX
inference-equivalence check, and T3 SIGINT cleanup while waiting for body data
have passed. That cleanup left no manager/bridge/XRT process or
5556/5557/60061 listener behind. Real-robot cleanup during normal POSE still
requires validation before declaring the true-hardware deployment closed.
