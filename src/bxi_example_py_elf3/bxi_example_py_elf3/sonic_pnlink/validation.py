"""Stage-specific validation for PN-Link diagnostic provenance."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np

from .diagnostics import DiagnosticIssue, rotation_matrix_is_valid
from .skeleton import (
    OPTIONAL_PNLINK_JOINTS,
    PNLINK_JOINT_NAMES,
    PNLINK_PARENTS,
    SMPL_JOINT_NAMES,
    SMPL_PARENTS,
    SMPL_TO_PNLINK,
    JointLocalPose,
    SkeletonWorldPose,
)


@dataclass(frozen=True)
class RawValidation:
    present: np.ndarray
    position_valid: np.ndarray
    rotation_valid: np.ndarray
    issues: tuple[DiagnosticIssue, ...]

    @property
    def has_error(self) -> bool:
        return any(issue.severity == "ERROR" for issue in self.issues)


def _mapping_for_source(name: str) -> tuple[int, str]:
    try:
        index = SMPL_TO_PNLINK.index(name)
    except ValueError:
        return -1, ""
    return index, SMPL_JOINT_NAMES[index]


def validate_raw_poses(
    poses: Mapping[str, JointLocalPose],
    *,
    frame_index: int,
    previous: Mapping[str, JointLocalPose] | None = None,
    neutral_bone_lengths_cm: dict[str, float] | None = None,
) -> RawValidation:
    present = np.zeros(len(PNLINK_JOINT_NAMES), dtype=bool)
    position_valid = np.zeros_like(present)
    rotation_valid = np.zeros_like(present)
    issues: list[DiagnosticIssue] = []
    for index, name in enumerate(PNLINK_JOINT_NAMES):
        parent = PNLINK_PARENTS[name] or ""
        smpl_index, smpl_joint = _mapping_for_source(name)
        pose = poses.get(name)
        if pose is None:
            if name not in OPTIONAL_PNLINK_JOINTS:
                issues.append(
                    DiagnosticIssue(
                        "SDK_RAW", "JOINT_MISSING", "state", "FRAME_DROPPED",
                        frame_index=frame_index,
                        source_joint=name,
                        source_parent=parent,
                        smpl_index=smpl_index,
                        smpl_joint=smpl_joint,
                        observed="missing",
                        expected="joint present",
                    )
                )
            continue
        present[index] = True
        try:
            position = np.asarray(pose.position_cm, dtype=np.float64).reshape(3)
            position_valid[index] = np.all(np.isfinite(position))
        except (TypeError, ValueError):
            position = np.full(3, np.nan)
        if not position_valid[index]:
            issues.append(
                DiagnosticIssue(
                    "SDK_RAW", "POSITION_NONFINITE", "position", "FRAME_DROPPED",
                    frame_index=frame_index,
                    source_joint=name,
                    source_parent=parent,
                    smpl_index=smpl_index,
                    smpl_joint=smpl_joint,
                    observed=np.array2string(position),
                    expected="finite float[3] centimeters",
                )
            )

        try:
            quat = np.asarray(pose.quaternion_wxyz, dtype=np.float64).reshape(4)
            norm = float(np.linalg.norm(quat))
            finite_quat = np.all(np.isfinite(quat))
        except (TypeError, ValueError):
            quat = np.full(4, np.nan)
            norm = float("nan")
            finite_quat = False
        rotation_valid[index] = finite_quat and 0.5 <= norm <= 1.5
        if not finite_quat:
            code = "ROTATION_NONFINITE"
        elif norm < 0.5 or norm > 1.5:
            code = "QUATERNION_NORM"
        else:
            code = ""
        if code:
            issues.append(
                DiagnosticIssue(
                    "SDK_RAW", code, "rotation", "FRAME_DROPPED",
                    frame_index=frame_index,
                    source_joint=name,
                    source_parent=parent,
                    smpl_index=smpl_index,
                    smpl_joint=smpl_joint,
                    observed=f"norm={norm}",
                    expected="finite quaternion norm in [0.5,1.5]",
                )
            )
        elif abs(norm - 1.0) > 0.02:
            issues.append(
                DiagnosticIssue(
                    "SDK_RAW", "QUATERNION_NORMALIZED", "rotation", "NORMALIZED",
                    frame_index=frame_index,
                    source_joint=name,
                    source_parent=parent,
                    smpl_index=smpl_index,
                    smpl_joint=smpl_joint,
                    observed=f"norm={norm:.6f}",
                    expected="abs(norm-1) <= 0.02",
                    severity="WARN",
                )
            )

        if previous is not None and name in previous and rotation_valid[index]:
            previous_quat = np.asarray(
                previous[name].quaternion_wxyz, dtype=np.float64
            ).reshape(4)
            previous_norm = np.linalg.norm(previous_quat)
            if np.isfinite(previous_norm) and previous_norm >= 0.5:
                q1 = quat / norm
                q0 = previous_quat / previous_norm
                angle = 2.0 * np.arccos(np.clip(abs(np.dot(q0, q1)), 0.0, 1.0))
                if angle > np.pi / 2.0:
                    issues.append(
                        DiagnosticIssue(
                            "SDK_RAW", "ROTATION_JUMP", "rotation", "FRAME_DROPPED",
                            frame_index=frame_index,
                            source_joint=name,
                            source_parent=parent,
                            smpl_index=smpl_index,
                            smpl_joint=smpl_joint,
                            observed=f"{np.rad2deg(angle):.2f}deg",
                            expected="<=90deg between frames",
                        )
                    )

        if (
            neutral_bone_lengths_cm is not None
            and parent
            and position_valid[index]
        ):
            length = float(np.linalg.norm(position))
            baseline = neutral_bone_lengths_cm.get(name)
            if baseline is not None and baseline > 1.0e-6:
                deviation = abs(length - baseline) / baseline
                if deviation > 0.1:
                    severity = "ERROR" if deviation > 0.3 else "WARN"
                    issues.append(
                        DiagnosticIssue(
                            "PNLINK_FK", "BONE_LENGTH_JUMP", "position",
                            "FRAME_DROPPED" if severity == "ERROR" else "ACCEPTED",
                            frame_index=frame_index,
                            source_joint=name,
                            source_parent=parent,
                            smpl_index=smpl_index,
                            smpl_joint=smpl_joint,
                            observed=f"deviation={deviation:.3f}",
                            expected="<=10% from neutral",
                            severity=severity,
                        )
                    )

    if previous is not None and "Hips" in poses and "Hips" in previous:
        current_root = np.asarray(poses["Hips"].position_cm, dtype=np.float64)
        previous_root = np.asarray(previous["Hips"].position_cm, dtype=np.float64)
        if np.all(np.isfinite(current_root)) and np.all(np.isfinite(previous_root)):
            jump_m = float(np.linalg.norm(current_root - previous_root) / 100.0)
            if jump_m > 1.0:
                issues.append(
                    DiagnosticIssue(
                        "SDK_RAW", "ROOT_POSITION_JUMP", "position", "FRAME_DROPPED",
                        frame_index=frame_index,
                        source_joint="Hips",
                        smpl_index=0,
                        smpl_joint="pelvis",
                        observed=f"{jump_m:.3f}m",
                        expected="<=1m between frames",
                    )
                )
    return RawValidation(
        present, position_valid, rotation_valid, tuple(issues)
    )


def validate_world_pose(
    world: SkeletonWorldPose,
    *,
    frame_index: int,
) -> tuple[DiagnosticIssue, ...]:
    issues: list[DiagnosticIssue] = []
    for name in PNLINK_JOINT_NAMES:
        position = world.positions_robot_m[name]
        rotation = world.rotations_robot[name]
        if not np.all(np.isfinite(position)):
            issues.append(
                DiagnosticIssue(
                    "PNLINK_FK", "FK_NONFINITE", "position", "FRAME_DROPPED",
                    frame_index=frame_index,
                    source_joint=name,
                    source_parent=PNLINK_PARENTS[name] or "",
                )
            )
        if not rotation_matrix_is_valid(rotation.as_matrix()):
            issues.append(
                DiagnosticIssue(
                    "PNLINK_FK", "ROTATION_NOT_ORTHOGONAL", "rotation",
                    "FRAME_DROPPED",
                    frame_index=frame_index,
                    source_joint=name,
                    source_parent=PNLINK_PARENTS[name] or "",
                )
            )
    return tuple(issues)


def validate_retarget_output(
    result: object,
    *,
    frame_index: int,
    neutral_bone_lengths_m: dict[int, float],
) -> tuple[DiagnosticIssue, ...]:
    issues: list[DiagnosticIssue] = []
    local_axis_angle = np.asarray(result.smpl_local_axis_angle)
    if local_axis_angle.shape != (22, 3) or not np.all(
        np.isfinite(local_axis_angle)
    ):
        issues.append(
            DiagnosticIssue(
                "SMPL_RETARGET", "ROTATION_NONFINITE", "rotation",
                "FRAME_DROPPED", frame_index=frame_index,
                observed=f"shape={local_axis_angle.shape}",
                expected="finite [22,3] axis-angle",
            )
        )
    elif np.any(np.linalg.norm(local_axis_angle, axis=1) > np.pi + 1.0e-4):
        issues.append(
            DiagnosticIssue(
                "SMPL_RETARGET", "AXIS_ANGLE_RANGE", "rotation",
                "FRAME_DROPPED", frame_index=frame_index,
                observed="axis-angle norm exceeds pi",
                expected="norm <= pi + 1e-4",
            )
        )
    for index, rotation in enumerate(result.smpl_global):
        if not rotation_matrix_is_valid(rotation.as_matrix()):
            issues.append(
                DiagnosticIssue(
                    "SMPL_RETARGET", "ROTATION_NOT_ORTHOGONAL", "rotation",
                    "FRAME_DROPPED", frame_index=frame_index,
                    smpl_index=index, smpl_joint=SMPL_JOINT_NAMES[index],
                )
            )

    joints = np.asarray(result.smpl_joints_local)
    root = np.asarray(result.root_quaternion_wxyz)
    wrist = np.asarray(result.wrist)
    if joints.shape != (24, 3) or not np.all(np.isfinite(joints)):
        issues.append(
            DiagnosticIssue(
                "SMPL_FK", "SMPL_FK_NONFINITE", "position", "FRAME_DROPPED",
                frame_index=frame_index,
                observed=f"shape={joints.shape}", expected="finite [24,3]",
            )
        )
        return tuple(issues)
    root_norm = float(np.linalg.norm(root)) if root.shape == (4,) else 0.0
    if not np.all(np.isfinite(root)) or root_norm <= 1.0e-6:
        issues.append(
            DiagnosticIssue(
                "SMPL_FK", "ROOT_QUATERNION_INVALID", "rotation",
                "FRAME_DROPPED", frame_index=frame_index,
                observed=f"shape={root.shape}, norm={root_norm}",
                expected="finite [4], norm > 1e-6",
            )
        )
    if wrist.shape != (6,) or not np.all(np.isfinite(wrist)):
        issues.append(
            DiagnosticIssue(
                "SMPL_FK", "WRIST_NONFINITE", "rotation", "FRAME_DROPPED",
                frame_index=frame_index,
                observed=f"shape={wrist.shape}", expected="finite [6]",
            )
        )
    for index, parent in enumerate(SMPL_PARENTS):
        if parent < 0:
            continue
        length = float(np.linalg.norm(joints[index] - joints[parent]))
        baseline = neutral_bone_lengths_m.setdefault(index, length)
        if abs(length - baseline) > 1.0e-4:
            issues.append(
                DiagnosticIssue(
                    "SMPL_FK", "SMPL_BONE_LENGTH_CHANGED", "position",
                    "FRAME_DROPPED", frame_index=frame_index,
                    smpl_index=index, smpl_joint=SMPL_JOINT_NAMES[index],
                    observed=f"{length:.8f}m", expected=f"{baseline:.8f}m +/-1e-4",
                )
            )
    return tuple(issues)
