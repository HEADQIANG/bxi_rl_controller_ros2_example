"""PN-Link skeleton constants and coordinate/forward-kinematics helpers."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np
from scipy.spatial.transform import Rotation


PNLINK_PARENTS = {
    "Hips": None,
    "Spine": "Hips",
    "Spine1": "Spine",
    "Spine2": "Spine1",
    "Neck": "Spine2",
    "Neck1": "Neck",
    "Head": "Neck1",
    "LeftShoulder": "Spine2",
    "LeftArm": "LeftShoulder",
    "LeftForeArm": "LeftArm",
    "LeftHand": "LeftForeArm",
    "RightShoulder": "Spine2",
    "RightArm": "RightShoulder",
    "RightForeArm": "RightArm",
    "RightHand": "RightForeArm",
    "LeftUpLeg": "Hips",
    "LeftLeg": "LeftUpLeg",
    "LeftFoot": "LeftLeg",
    "LeftTiptoe": "LeftFoot",
    "RightUpLeg": "Hips",
    "RightLeg": "RightUpLeg",
    "RightFoot": "RightLeg",
    "RightTiptoe": "RightFoot",
}

PNLINK_JOINT_NAMES = tuple(PNLINK_PARENTS)
OPTIONAL_PNLINK_JOINTS = frozenset(("LeftTiptoe", "RightTiptoe"))
REQUIRED_PNLINK_JOINTS = frozenset(PNLINK_PARENTS) - OPTIONAL_PNLINK_JOINTS

SMPL_JOINT_NAMES = (
    "pelvis", "left_hip", "right_hip", "spine1", "left_knee",
    "right_knee", "spine2", "left_ankle", "right_ankle", "spine3",
    "left_foot", "right_foot", "neck", "left_collar", "right_collar",
    "head", "left_shoulder", "right_shoulder", "left_elbow",
    "right_elbow", "left_wrist", "right_wrist", "left_hand", "right_hand",
)
SMPL_PARENTS = np.asarray(
    [-1, 0, 0, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21],
    dtype=np.int64,
)
SMPL_TO_PNLINK = (
    "Hips", "LeftUpLeg", "RightUpLeg", "Spine", "LeftLeg", "RightLeg",
    "Spine1", "LeftFoot", "RightFoot", "Spine2", "LeftTiptoe",
    "RightTiptoe", "Neck", "LeftShoulder", "RightShoulder", "Head",
    "LeftArm", "RightArm", "LeftForeArm", "RightForeArm", "LeftHand",
    "RightHand", None, None,
)

PNLINK_TO_ROBOT = np.asarray(
    [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
    dtype=np.float64,
)


@dataclass(frozen=True)
class JointLocalPose:
    position_cm: np.ndarray
    quaternion_wxyz: np.ndarray


@dataclass(frozen=True)
class SkeletonWorldPose:
    positions_pnlink_cm: dict[str, np.ndarray]
    rotations_pnlink: dict[str, Rotation]
    positions_robot_m: dict[str, np.ndarray]
    rotations_robot: dict[str, Rotation]


def normalize_quaternion_wxyz(value: np.ndarray, *, min_norm: float = 1.0e-6) -> np.ndarray:
    quat = np.asarray(value, dtype=np.float64).reshape(4)
    if not np.all(np.isfinite(quat)):
        raise ValueError("quaternion contains non-finite values")
    norm = float(np.linalg.norm(quat))
    if norm < min_norm:
        raise ValueError(f"quaternion norm {norm:.3g} is below {min_norm:.3g}")
    return quat / norm


def rotation_from_wxyz(value: np.ndarray) -> Rotation:
    quat = normalize_quaternion_wxyz(value)
    return Rotation.from_quat(quat[[1, 2, 3, 0]])


def rotation_to_wxyz(value: Rotation) -> np.ndarray:
    quat_xyzw = value.as_quat()
    return np.asarray(quat_xyzw[[3, 0, 1, 2]], dtype=np.float64)


def validate_local_poses(poses: Mapping[str, JointLocalPose]) -> None:
    missing = sorted(REQUIRED_PNLINK_JOINTS - poses.keys())
    if missing:
        raise ValueError(f"missing required PN-Link joints: {missing}")
    for name, pose in poses.items():
        position = np.asarray(pose.position_cm, dtype=np.float64).reshape(3)
        if not np.all(np.isfinite(position)):
            raise ValueError(f"{name} position contains non-finite values")
        normalize_quaternion_wxyz(pose.quaternion_wxyz)


def forward_kinematics(poses: Mapping[str, JointLocalPose]) -> SkeletonWorldPose:
    """Evaluate the PN-Link local skeleton, then change basis to robot Z-up."""
    validate_local_poses(poses)
    local = dict(poses)
    for toe, foot in (("LeftTiptoe", "LeftFoot"), ("RightTiptoe", "RightFoot")):
        if toe not in local:
            local[toe] = JointLocalPose(
                position_cm=np.zeros(3, dtype=np.float64),
                quaternion_wxyz=np.array([1.0, 0.0, 0.0, 0.0]),
            )

    world_pos: dict[str, np.ndarray] = {}
    world_rot: dict[str, Rotation] = {}
    for name, parent in PNLINK_PARENTS.items():
        pose = local[name]
        local_pos = np.asarray(pose.position_cm, dtype=np.float64).reshape(3)
        local_rot = rotation_from_wxyz(pose.quaternion_wxyz)
        if parent is None:
            world_pos[name] = local_pos
            world_rot[name] = local_rot
        else:
            world_pos[name] = world_pos[parent] + world_rot[parent].apply(local_pos)
            world_rot[name] = world_rot[parent] * local_rot

    c = PNLINK_TO_ROBOT
    robot_pos = {name: c @ value / 100.0 for name, value in world_pos.items()}
    robot_rot = {
        name: Rotation.from_matrix(c @ value.as_matrix() @ c.T)
        for name, value in world_rot.items()
    }
    return SkeletonWorldPose(world_pos, world_rot, robot_pos, robot_rot)
