"""Neutral-pose rotation averaging and stability checks."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Sequence

import numpy as np
from scipy.spatial.transform import Rotation

from .skeleton import (
    PNLINK_PARENTS,
    REQUIRED_PNLINK_JOINTS,
    rotation_from_wxyz,
    rotation_to_wxyz,
)


def markley_mean_wxyz(quaternions: np.ndarray) -> np.ndarray:
    values = np.asarray(quaternions, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 4 or values.shape[0] == 0:
        raise ValueError(f"expected quaternion array [N,4], got {values.shape}")
    if not np.all(np.isfinite(values)):
        raise ValueError("quaternion samples contain non-finite values")
    norms = np.linalg.norm(values, axis=1)
    if np.any(norms < 1.0e-6):
        raise ValueError("quaternion sample has near-zero norm")
    values = values / norms[:, None]
    reference = values[0]
    values[np.sum(values * reference, axis=1) < 0.0] *= -1.0
    eigenvalues, eigenvectors = np.linalg.eigh(values.T @ values)
    result = eigenvectors[:, int(np.argmax(eigenvalues))]
    if np.dot(result, reference) < 0.0:
        result *= -1.0
    return result / np.linalg.norm(result)


@dataclass(frozen=True)
class NeutralCalibration:
    rotations: dict[str, Rotation]
    sample_count: int
    duration_seconds: float
    max_angle_degrees: float


def median_bone_lengths_cm(
    samples: Sequence[Mapping[str, float]],
    *,
    required_samples: int = 25,
) -> dict[str, float]:
    """Build a stable local bone-length baseline from neutral-pose samples."""
    if len(samples) != required_samples:
        raise ValueError(
            f"bone length calibration requires exactly {required_samples} frames"
        )

    required_bones = {
        name for name in REQUIRED_PNLINK_JOINTS if PNLINK_PARENTS[name] is not None
    }
    common_bones = set(samples[0])
    for sample in samples[1:]:
        common_bones.intersection_update(sample)
    missing = sorted(required_bones - common_bones)
    if missing:
        raise ValueError(
            "bone length calibration missing joints: " + ", ".join(missing)
        )

    result: dict[str, float] = {}
    for name in sorted(common_bones):
        values = np.asarray([sample[name] for sample in samples], dtype=np.float64)
        if not np.all(np.isfinite(values)) or np.any(values <= 1.0e-6):
            raise ValueError(f"bone length calibration has invalid values for {name}")
        result[name] = float(np.median(values))
    return result


def calibrate_neutral(
    samples: Sequence[tuple[int, Mapping[str, Rotation]]],
    *,
    required_samples: int = 25,
    max_duration_seconds: float = 1.0,
    max_angle_degrees: float = 5.0,
) -> NeutralCalibration:
    if len(samples) != required_samples:
        raise ValueError(f"neutral calibration requires exactly {required_samples} frames")
    duration = (samples[-1][0] - samples[0][0]) / 1.0e9
    if duration < 0.0 or duration > max_duration_seconds:
        raise ValueError(f"neutral calibration duration {duration:.3f}s exceeds limit")

    rotations: dict[str, Rotation] = {}
    observed_max = 0.0
    for name in sorted(REQUIRED_PNLINK_JOINTS):
        try:
            quats = np.stack(
                [rotation_to_wxyz(frame[name]) for _, frame in samples]
            )
        except KeyError as exc:
            raise ValueError(f"neutral calibration missing joint {name}") from exc
        mean = rotation_from_wxyz(markley_mean_wxyz(quats))
        errors = (
            mean.inv() * Rotation.from_quat(quats[:, [1, 2, 3, 0]])
        ).magnitude()
        joint_max = float(np.rad2deg(np.max(errors)))
        observed_max = max(observed_max, joint_max)
        if joint_max > max_angle_degrees:
            raise ValueError(
                f"neutral joint {name} moved {joint_max:.2f}deg; "
                f"limit is {max_angle_degrees:.2f}deg"
            )
        rotations[name] = mean

    return NeutralCalibration(rotations, len(samples), duration, observed_max)
