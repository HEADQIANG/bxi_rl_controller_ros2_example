"""ELF3 FK reference poses for SONIC PICO 3-point calibration."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as Rotation

try:
    import pinocchio as pin
except Exception:  # pragma: no cover - import depends on deploy environment
    pin = None

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:  # pragma: no cover - allows source-tree checks without ROS setup
    get_package_share_directory = None


ELF3_CONTROLLED_JOINTS: tuple[str, ...] = (
    "waist_y_joint",
    "waist_x_joint",
    "waist_z_joint",
    "l_hip_y_joint",
    "l_hip_x_joint",
    "l_hip_z_joint",
    "l_knee_y_joint",
    "l_ankle_y_joint",
    "l_ankle_x_joint",
    "r_hip_y_joint",
    "r_hip_x_joint",
    "r_hip_z_joint",
    "r_knee_y_joint",
    "r_ankle_y_joint",
    "r_ankle_x_joint",
    "l_shoulder_y_joint",
    "l_shoulder_x_joint",
    "l_shoulder_z_joint",
    "l_elbow_y_joint",
    "l_wrist_x_joint",
    "l_wrist_y_joint",
    "l_wrist_z_joint",
    "r_shoulder_y_joint",
    "r_shoulder_x_joint",
    "r_shoulder_z_joint",
    "r_elbow_y_joint",
    "r_wrist_x_joint",
    "r_wrist_y_joint",
    "r_wrist_z_joint",
)

ELF3_FRAME_MAPPING: dict[str, str] = {
    "left_wrist": "l_wrist_z_link",
    "right_wrist": "r_wrist_z_link",
    "torso": "torso_link",
    "anchor": "waist_z_link",
}

ELF3_KEY_FRAME_OFFSETS: dict[str, np.ndarray] = {
    "left_wrist": np.array([0.203, 0.0, 0.0], dtype=np.float64),
    "right_wrist": np.array([0.203, 0.0, 0.0], dtype=np.float64),
    "torso": np.array([0.0, 0.0, 0.35], dtype=np.float64),
    "anchor": np.array([0.0, 0.0, 0.0], dtype=np.float64),
}


def _package_data_root() -> Optional[Path]:
    if get_package_share_directory is None:
        return None
    try:
        return Path(get_package_share_directory("bxi_example_py_elf3")) / "data"
    except Exception:
        return None


def resolve_default_elf3_urdf() -> Path:
    relative = Path("sonic_robot_model/elf3_dof29_hand/urdf/elf3.urdf")
    data_root = _package_data_root()
    if data_root is not None:
        installed = data_root / relative
        if installed.exists():
            return installed

    source_file = Path(__file__).resolve()
    for parent in source_file.parents:
        source_tree = parent / "data" / relative
        if source_tree.exists():
            return source_tree
        workspace_resource = parent / "resources" / "elf3_dof29_hand" / "urdf" / "elf3.urdf"
        if workspace_resource.exists():
            return workspace_resource

    raise FileNotFoundError(f"Cannot resolve default ELF3 URDF: {relative}")


@dataclass
class Elf3FkCalibration:
    urdf_path: Path
    asset_dir: Path

    def __post_init__(self) -> None:
        if pin is None:
            raise ImportError("pinocchio is required for ELF3 FK calibration")

        self.model = pin.buildModelFromUrdf(str(self.urdf_path))
        self.data = self.model.createData()
        self.q0 = pin.neutral(self.model)
        self.joint_to_q_index = self._build_joint_to_q_index()

    @classmethod
    def from_default_urdf(cls) -> "Elf3FkCalibration":
        urdf_path = resolve_default_elf3_urdf()
        return cls(urdf_path=urdf_path, asset_dir=urdf_path.parent)

    def _build_joint_to_q_index(self) -> dict[str, int]:
        mapping: dict[str, int] = {}
        for joint_name in self.model.names[1:]:
            joint_id = self.model.getJointId(joint_name)
            joint_model = self.model.joints[joint_id]
            if joint_model.nq == 1:
                mapping[joint_name] = joint_model.idx_q
        return mapping

    def configuration_from_29(self, body_q: np.ndarray | None = None) -> np.ndarray:
        q = self.q0.copy()
        if body_q is None:
            return q

        body_q = np.asarray(body_q, dtype=np.float64).reshape(-1)
        if body_q.shape[0] < len(ELF3_CONTROLLED_JOINTS):
            raise ValueError(
                f"ELF3 body_q must have at least {len(ELF3_CONTROLLED_JOINTS)} values, "
                f"got {body_q.shape[0]}"
            )

        for value, joint_name in zip(body_q, ELF3_CONTROLLED_JOINTS):
            try:
                q[self.joint_to_q_index[joint_name]] = value
            except KeyError as exc:
                raise RuntimeError(f"ELF3 URDF is missing controlled joint {joint_name}") from exc
        return q

    def key_frame_poses(
        self,
        body_q: np.ndarray | None = None,
        root_position: np.ndarray | None = None,
        apply_offset: bool = True,
    ) -> dict[str, dict[str, np.ndarray]]:
        q = self.configuration_from_29(body_q)
        if root_position is None:
            root_position = np.zeros(3, dtype=np.float64)
        else:
            root_position = np.asarray(root_position, dtype=np.float64).reshape(3)

        pin.framesForwardKinematics(self.model, self.data, q)

        result: dict[str, dict[str, np.ndarray]] = {}
        for key, frame_name in ELF3_FRAME_MAPPING.items():
            frame_id = self.model.getFrameId(frame_name)
            if frame_id < 0 or frame_id >= len(self.model.frames):
                raise RuntimeError(f"ELF3 URDF is missing frame {frame_name}")

            placement = self.data.oMf[frame_id]
            rotation = placement.rotation
            position = placement.translation.copy()

            if apply_offset:
                position = position + rotation @ ELF3_KEY_FRAME_OFFSETS[key]
            position = position + root_position

            quat_xyzw = Rotation.from_matrix(rotation).as_quat()
            quat_wxyz = np.array(
                [quat_xyzw[3], quat_xyzw[0], quat_xyzw[1], quat_xyzw[2]],
                dtype=np.float64,
            )
            result[key] = {
                "position": position.astype(np.float64, copy=True),
                "orientation_xyzw": quat_xyzw.astype(np.float64, copy=True),
                "orientation_wxyz": quat_wxyz,
            }

        return result


def get_elf3_key_frame_poses(
    body_q: np.ndarray | None = None,
    root_position: np.ndarray | None = None,
    apply_offset: bool = True,
    fk: Elf3FkCalibration | None = None,
) -> dict[str, dict[str, np.ndarray]]:
    fk = fk or Elf3FkCalibration.from_default_urdf()
    return fk.key_frame_poses(
        body_q=body_q,
        root_position=root_position,
        apply_offset=apply_offset,
    )
