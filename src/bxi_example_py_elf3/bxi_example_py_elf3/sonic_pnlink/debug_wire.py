"""Non-blocking PN-Link debug-frame publisher."""

from __future__ import annotations

import json
from typing import Any

import numpy as np
try:
    import zmq
except ImportError:  # pragma: no cover - wire tests do not need a socket
    zmq = None

from bxi_example_py_elf3.sonic_pico.zmq_messages import pack_pose_message

from .skeleton import PNLINK_JOINT_NAMES, rotation_to_wxyz


DEBUG_SCHEMA_VERSION = 1


def pack_debug_frame(fields: dict[str, np.ndarray]) -> bytes:
    payload = {"schema_version": np.array([DEBUG_SCHEMA_VERSION], dtype=np.int32)}
    payload.update(fields)
    return pack_pose_message(payload, topic="pnlink_debug", version=DEBUG_SCHEMA_VERSION)


def decode_debug_frame(message: bytes) -> dict[str, np.ndarray]:
    topic = b"pnlink_debug"
    if not message.startswith(topic):
        raise ValueError("debug message topic is not pnlink_debug")
    payload = message[len(topic):]
    header_size = 1280
    if len(payload) < header_size:
        raise ValueError("debug message is shorter than its header")
    raw_header = payload[:header_size].split(b"\x00", 1)[0]
    header = json.loads(raw_header.decode("utf-8"))
    dtype_map = {
        "f32": np.dtype("<f4"),
        "f64": np.dtype("<f8"),
        "i32": np.dtype("<i4"),
        "i64": np.dtype("<i8"),
        "u8": np.dtype("u1"),
        "bool": np.dtype("?"),
    }
    data = memoryview(payload[header_size:])
    fields: dict[str, np.ndarray] = {}
    offset = 0
    for descriptor in header.get("fields", []):
        dtype = dtype_map.get(descriptor.get("dtype"))
        if dtype is None:
            raise ValueError(f"unsupported debug dtype: {descriptor.get('dtype')}")
        shape = tuple(int(value) for value in descriptor.get("shape", []))
        count = int(np.prod(shape)) if shape else 1
        size = count * dtype.itemsize
        if offset + size > len(data):
            raise ValueError(f"debug field {descriptor.get('name')} exceeds payload")
        fields[descriptor["name"]] = np.frombuffer(
            data[offset:offset + size], dtype=dtype, count=count
        ).reshape(shape).copy()
        offset += size
    version = int(np.asarray(fields.get("schema_version", [-1])).reshape(-1)[0])
    if version != DEBUG_SCHEMA_VERSION:
        raise ValueError(f"unsupported debug schema version: {version}")
    return fields


def build_debug_snapshot(
    *,
    frame_index: int,
    timestamp_monotonic_ns: int,
    joints: dict[str, Any],
    world: Any = None,
    result: Any = None,
    raw_present: np.ndarray | None = None,
    raw_position_valid: np.ndarray | None = None,
    raw_rotation_valid: np.ndarray | None = None,
    source_stage_valid: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    joint_count = len(PNLINK_JOINT_NAMES)
    local_pos = np.full((joint_count, 3), np.nan, dtype=np.float32)
    local_quat = np.full((joint_count, 4), np.nan, dtype=np.float32)
    for index, name in enumerate(PNLINK_JOINT_NAMES):
        if name not in joints:
            continue
        try:
            local_pos[index] = np.asarray(
                joints[name].position_cm, dtype=np.float32
            ).reshape(3)
        except (TypeError, ValueError):
            pass
        try:
            local_quat[index] = np.asarray(
                joints[name].quaternion_wxyz, dtype=np.float32
            ).reshape(4)
        except (TypeError, ValueError):
            pass
    raw_world_pos = np.full((joint_count, 3), np.nan, dtype=np.float32)
    raw_world_quat = np.full((joint_count, 4), np.nan, dtype=np.float32)
    if world is not None:
        for index, name in enumerate(PNLINK_JOINT_NAMES):
            if name not in world.positions_robot_m:
                continue
            raw_world_pos[index] = world.positions_robot_m[name]
            raw_world_quat[index] = rotation_to_wxyz(
                world.rotations_robot[name]
            )

    retarget_quat = np.full((24, 4), np.nan, dtype=np.float32)
    local_axis_angle = np.full((22, 3), np.nan, dtype=np.float32)
    smpl_joints = np.full((24, 3), np.nan, dtype=np.float32)
    root_quat = np.full(4, np.nan, dtype=np.float32)
    wrist = np.full(6, np.nan, dtype=np.float32)
    if result is not None:
        retarget_quat = np.stack(
            [rotation_to_wxyz(value) for value in result.smpl_global]
        ).astype(np.float32)
        local_axis_angle = np.asarray(
            result.smpl_local_axis_angle, dtype=np.float32
        ).reshape(22, 3)
        smpl_joints = np.asarray(
            result.smpl_joints_local, dtype=np.float32
        ).reshape(24, 3)
        root_quat = np.asarray(
            result.root_quaternion_wxyz, dtype=np.float32
        ).reshape(4)
        wrist = np.asarray(result.wrist, dtype=np.float32).reshape(6)

    present = (
        np.asarray(raw_present, dtype=bool).reshape(joint_count)
        if raw_present is not None
        else np.asarray([name in joints for name in PNLINK_JOINT_NAMES], dtype=bool)
    )
    position_valid = (
        np.asarray(raw_position_valid, dtype=bool).reshape(joint_count)
        if raw_position_valid is not None
        else np.all(np.isfinite(local_pos), axis=1)
    )
    rotation_valid = (
        np.asarray(raw_rotation_valid, dtype=bool).reshape(joint_count)
        if raw_rotation_valid is not None
        else np.all(np.isfinite(local_quat), axis=1)
    )
    return {
        "frame_index": np.array([frame_index], dtype=np.int64),
        "timestamp_monotonic": np.array(
            [timestamp_monotonic_ns / 1.0e9], dtype=np.float64
        ),
        "raw_local_pos_cm": local_pos,
        "raw_local_quat": local_quat,
        "raw_world_pos": raw_world_pos,
        "raw_world_quat": raw_world_quat,
        "retarget_global_quat": retarget_quat,
        "smpl_local_axis_angle": local_axis_angle,
        "smpl_joints": smpl_joints,
        "smpl_root_quat": root_quat,
        "wrist": wrist,
        "raw_present": present,
        "raw_position_valid": position_valid,
        "raw_rotation_valid": rotation_valid,
        "smpl_mapping_valid": np.all(np.isfinite(retarget_quat), axis=1),
        "smpl_position_valid": np.all(np.isfinite(smpl_joints), axis=1),
        "smpl_rotation_valid": np.all(np.isfinite(retarget_quat), axis=1),
        "source_stage_valid": (
            np.asarray(source_stage_valid, dtype=bool).reshape(7)
            if source_stage_valid is not None
            else np.ones(7, dtype=bool)
        ),
    }


def sanitize_debug_frame(
    fields: dict[str, np.ndarray],
    last_valid: dict[str, np.ndarray] | None = None,
) -> tuple[dict[str, np.ndarray], dict[str, np.ndarray]]:
    """Replace non-renderable values without mutating the received snapshot."""
    previous = dict(last_valid or {})
    safe = {name: np.array(value, copy=True) for name, value in fields.items()}
    defaults = {
        "raw_world_pos": np.zeros((len(PNLINK_JOINT_NAMES), 3), dtype=np.float32),
        "raw_world_quat": np.tile(
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
            (len(PNLINK_JOINT_NAMES), 1),
        ),
        "smpl_joints": np.zeros((24, 3), dtype=np.float32),
        "retarget_global_quat": np.tile(
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32), (24, 1)
        ),
    }
    validity_fields = {
        "raw_world_pos": "raw_position_valid",
        "raw_world_quat": "raw_rotation_valid",
        "smpl_joints": "smpl_position_valid",
        "retarget_global_quat": "smpl_rotation_valid",
    }
    for name, fallback in defaults.items():
        value = np.asarray(safe.get(name, fallback), dtype=np.float32)
        replacement = np.asarray(previous.get(name, fallback), dtype=np.float32)
        received = np.asarray(fields.get(name, fallback), dtype=np.float32)
        valid_rows = np.all(np.isfinite(received), axis=1)
        validity_name = validity_fields[name]
        if validity_name in fields:
            valid_rows &= np.asarray(fields[validity_name], dtype=bool).reshape(-1)
        value[~valid_rows] = replacement[~valid_rows]
        if not np.all(np.isfinite(value)):
            value[~np.isfinite(value)] = fallback[~np.isfinite(value)]
        safe[name] = value
        updated = replacement.copy()
        updated[valid_rows] = value[valid_rows]
        previous[name] = updated
    return safe, previous


class DebugPublisher:
    def __init__(self, context: Any, endpoint: str) -> None:
        if zmq is None:
            raise RuntimeError("PN-Link debug publisher requires pyzmq")
        self.socket = context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.setsockopt(zmq.LINGER, 0)
        self.socket.bind(endpoint)
        self.dropped = 0

    def send(self, fields: dict[str, np.ndarray]) -> None:
        try:
            self.socket.send(pack_debug_frame(fields), flags=zmq.NOBLOCK)
        except zmq.Again:
            self.dropped += 1

    def close(self) -> None:
        self.socket.close(linger=0)
