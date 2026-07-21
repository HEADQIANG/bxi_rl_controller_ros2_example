"""PN-Link global rotations to normalized SMPL and ELF3 wrist references."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import sys
from typing import Mapping

import numpy as np
from scipy.spatial.transform import Rotation

from .skeleton import PNLINK_TO_ROBOT, SMPL_PARENTS, SMPL_TO_PNLINK, rotation_from_wxyz


@dataclass(frozen=True)
class RetargetResult:
    smpl_global: tuple[Rotation, ...]
    smpl_local_axis_angle: np.ndarray
    smpl_joints_local: np.ndarray
    root_quaternion_wxyz: np.ndarray
    wrist: np.ndarray


def _smpl_global_rotations(
    current: Mapping[str, Rotation], neutral: Mapping[str, Rotation]
) -> tuple[Rotation, ...]:
    c = PNLINK_TO_ROBOT
    b_yz = Rotation.from_rotvec(np.array([np.pi / 2.0, 0.0, 0.0]))
    b_smpl = rotation_from_wxyz(np.array([0.5, 0.5, 0.5, 0.5]))
    result: list[Rotation] = []
    for index, source_name in enumerate(SMPL_TO_PNLINK):
        if source_name in ("LeftTiptoe", "RightTiptoe") and (
            source_name not in current or source_name not in neutral
        ):
            source_name = "LeftFoot" if source_name == "LeftTiptoe" else "RightFoot"
        if source_name is None:
            result.append(result[int(SMPL_PARENTS[index])])
            continue
        if source_name not in current or source_name not in neutral:
            raise ValueError(f"missing rotation for SMPL mapping source {source_name}")
        delta_pnlink = current[source_name] * neutral[source_name].inv()
        delta_robot = Rotation.from_matrix(c @ delta_pnlink.as_matrix() @ c.T)
        result.append(b_yz.inv() * delta_robot * b_smpl)
    return tuple(result)


def _smpl_local_axis_angle(global_rotations: tuple[Rotation, ...]) -> np.ndarray:
    if len(global_rotations) != 24:
        raise ValueError(f"expected 24 SMPL global rotations, got {len(global_rotations)}")
    local: list[np.ndarray] = []
    for index, rotation in enumerate(global_rotations):
        parent = int(SMPL_PARENTS[index])
        relative = rotation if parent < 0 else global_rotations[parent].inv() * rotation
        local.append(relative.as_rotvec())
    result = np.asarray(local, dtype=np.float32)
    if not np.all(np.isfinite(result)):
        raise ValueError("SMPL local axis-angle contains non-finite values")
    if np.any(np.linalg.norm(result, axis=1) > np.pi + 1.0e-4):
        raise ValueError("SMPL local axis-angle is outside the principal range")
    return result


def wrist_reference(body_pose_axis_angle: np.ndarray) -> np.ndarray:
    """Map SMPL elbow/wrist rotations to ELF3 native wrist joint angles."""
    pose = np.asarray(body_pose_axis_angle, dtype=np.float64).reshape(21, 3)
    l_elbow, r_elbow = pose[17], pose[18]
    l_wrist, r_wrist = pose[19], pose[20]
    y_axis = np.array([0.0, 1.0, 0.0])

    def swing_euler(elbow: np.ndarray) -> np.ndarray:
        rotation = Rotation.from_rotvec(elbow)
        quat_xyzw = rotation.as_quat()
        quat = quat_xyzw[[3, 0, 1, 2]]
        projected = np.dot(quat[1:], y_axis) * y_axis
        twist_quat = np.concatenate(([quat[0]], projected))
        norm = np.linalg.norm(twist_quat)
        twist = Rotation.identity() if norm < 1.0e-8 else rotation_from_wxyz(
            twist_quat / norm
        )
        return (twist.inv() * rotation).as_euler("XYZ", degrees=False)

    l_swing = swing_euler(l_elbow)
    r_swing = swing_euler(r_elbow)
    l_wrist_euler = Rotation.from_rotvec(l_wrist).as_euler("XYZ", degrees=False)
    r_wrist_euler = Rotation.from_rotvec(r_wrist).as_euler("XYZ", degrees=False)
    return np.asarray(
        [
            l_swing[0] + l_wrist_euler[0],
            l_wrist_euler[1],
            l_swing[2] + l_wrist_euler[2],
            -(r_swing[0] + r_wrist_euler[0]),
            -r_wrist_euler[1],
            r_swing[2] + r_wrist_euler[2],
        ],
        dtype=np.float32,
    )


def _process_smpl(local_axis_angle: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    try:
        import torch
    except ImportError as exc:
        raise RuntimeError("PN-Link retargeting requires torch") from exc

    vendor_root = Path(__file__).resolve().parents[1] / "sonic_pico" / "vendor"
    if str(vendor_root) not in sys.path:
        sys.path.insert(0, str(vendor_root))
    try:
        from gear_sonic.scripts.pico_manager_thread_server import process_smpl_joints
    except ImportError as exc:
        raise RuntimeError(f"cannot import SONIC SMPL processor from {vendor_root}") from exc

    body_pose = torch.from_numpy(local_axis_angle[1:22].reshape(1, 63)).float()
    global_orient = torch.from_numpy(local_axis_angle[0].reshape(1, 3)).float()
    transl = torch.zeros((1, 3), dtype=torch.float32)
    with torch.no_grad():
        output = process_smpl_joints(body_pose, global_orient, transl)
    joints = output["smpl_joints_local"].detach().cpu().numpy().reshape(24, 3)
    root_quat = output["global_orient_quat"].detach().cpu().numpy().reshape(4)
    return np.asarray(joints, dtype=np.float32), np.asarray(root_quat, dtype=np.float32)


def retarget_frame(
    current_global_rotations: Mapping[str, Rotation],
    neutral_global_rotations: Mapping[str, Rotation],
) -> RetargetResult:
    smpl_global, local_axis_angle = retarget_rotations(
        current_global_rotations, neutral_global_rotations
    )
    return process_smpl_frame(smpl_global, local_axis_angle)


def retarget_rotations(
    current_global_rotations: Mapping[str, Rotation],
    neutral_global_rotations: Mapping[str, Rotation],
) -> tuple[tuple[Rotation, ...], np.ndarray]:
    smpl_global = _smpl_global_rotations(current_global_rotations, neutral_global_rotations)
    local_axis_angle = _smpl_local_axis_angle(smpl_global)
    return smpl_global, local_axis_angle


def process_smpl_frame(
    smpl_global: tuple[Rotation, ...],
    local_axis_angle: np.ndarray,
) -> RetargetResult:
    joints, root_quat = _process_smpl(local_axis_angle)
    wrist = wrist_reference(local_axis_angle[1:22])
    if joints.shape != (24, 3) or root_quat.shape != (4,) or wrist.shape != (6,):
        raise ValueError("SONIC SMPL processor returned an invalid shape")
    if not all(np.all(np.isfinite(value)) for value in (joints, root_quat, wrist)):
        raise ValueError("SONIC SMPL output contains non-finite values")
    norm = float(np.linalg.norm(root_quat))
    if norm < 1.0e-6:
        raise ValueError("SONIC SMPL root quaternion has near-zero norm")
    root_quat = root_quat / norm
    return RetargetResult(smpl_global, local_axis_angle[:22], joints, root_quat, wrist)
