"""Timestamp-based 50 Hz resampling for normalized PN-Link frames."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


TARGET_PERIOD_NS = 20_000_000
SOURCE_STALE_NS = 100_000_000


@dataclass(frozen=True)
class StandardFrame:
    timestamp_ns: int
    timestamp_realtime: float
    smpl_joints: np.ndarray
    root_quat: np.ndarray
    wrist: np.ndarray


class StandardFrameResampler:
    def __init__(self, period_ns: int = TARGET_PERIOD_NS) -> None:
        self.period_ns = int(period_ns)
        self.previous: StandardFrame | None = None
        self.next_timestamp_ns: int | None = None

    def reset(self) -> None:
        self.previous = None
        self.next_timestamp_ns = None

    def add(self, current: StandardFrame) -> list[StandardFrame]:
        if self.previous is None:
            self.previous = current
            self.next_timestamp_ns = current.timestamp_ns
            return []
        previous = self.previous
        if current.timestamp_ns <= previous.timestamp_ns:
            self.reset()
            self.previous = current
            self.next_timestamp_ns = current.timestamp_ns
            return []
        if current.timestamp_ns - previous.timestamp_ns > SOURCE_STALE_NS:
            self.reset()
            self.previous = current
            self.next_timestamp_ns = current.timestamp_ns
            return []

        output: list[StandardFrame] = []
        assert self.next_timestamp_ns is not None
        while self.next_timestamp_ns <= current.timestamp_ns:
            if self.next_timestamp_ns < previous.timestamp_ns:
                self.next_timestamp_ns += self.period_ns
                continue
            alpha = (self.next_timestamp_ns - previous.timestamp_ns) / (
                current.timestamp_ns - previous.timestamp_ns
            )
            alpha = float(np.clip(alpha, 0.0, 1.0))
            quats_wxyz = np.stack((previous.root_quat, current.root_quat))
            rotations = Rotation.from_quat(quats_wxyz[:, [1, 2, 3, 0]])
            root_xyzw = Slerp([0.0, 1.0], rotations)([alpha]).as_quat()[0]
            root = root_xyzw[[3, 0, 1, 2]]
            realtime = (1.0 - alpha) * previous.timestamp_realtime
            realtime += alpha * current.timestamp_realtime
            joints = (1.0 - alpha) * previous.smpl_joints
            joints += alpha * current.smpl_joints
            joints = joints.astype(np.float32)
            output.append(
                StandardFrame(
                    self.next_timestamp_ns,
                    realtime,
                    joints,
                    np.asarray(root, dtype=np.float32),
                    ((1.0 - alpha) * previous.wrist + alpha * current.wrist).astype(np.float32),
                )
            )
            self.next_timestamp_ns += self.period_ns
        self.previous = current
        return output


def build_pose_window(entries: list[tuple[int, StandardFrame]]) -> dict[str, np.ndarray]:
    if len(entries) != 10:
        raise ValueError(f"pose window must contain exactly 10 frames, got {len(entries)}")
    frame_indices = np.asarray([item[0] for item in entries], dtype=np.int64)
    if np.any(np.diff(frame_indices) != 1):
        raise ValueError("pose window frame indices must be consecutive")
    return {
        "frame_index": frame_indices,
        "smpl_joints": np.stack([item[1].smpl_joints for item in entries]).astype(np.float32),
        "body_quat_w": np.stack([item[1].root_quat for item in entries]).astype(np.float32),
        "wrist": np.stack([item[1].wrist for item in entries]).astype(np.float32),
        "stream_mode": np.array([1], dtype=np.int32),
        "calibration_ready": np.array([True], dtype=bool),
        "timestamp_realtime": np.array([entries[-1][1].timestamp_realtime], dtype=np.float64),
        "timestamp_monotonic": np.array(
            [entries[-1][1].timestamp_ns / 1.0e9], dtype=np.float64
        ),
    }
