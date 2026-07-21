"""Small, dependency-free diagnostic primitives used by the PN-Link source."""

from __future__ import annotations

from collections import Counter, deque
from dataclasses import dataclass, field as dataclass_field
import json
from pathlib import Path
import queue
import shutil
import threading
import time
from typing import Any

import numpy as np


PIPELINE_STAGES = (
    "SDK_RAW", "PNLINK_FK", "CALIBRATION", "SMPL_RETARGET", "SMPL_FK",
    "TEMPORAL_BUFFER", "POSE_WIRE", "SMPL_REF_BRIDGE", "POLICY_INPUT",
)
SOURCE_STAGES = PIPELINE_STAGES[:7]


@dataclass(frozen=True)
class DiagnosticIssue:
    stage: str
    code: str
    field: str
    action: str
    frame_index: int = -1
    source_joint: str = ""
    source_parent: str = ""
    smpl_index: int = -1
    smpl_joint: str = ""
    origin: bool = True
    caused_by: str = ""
    observed: str = ""
    expected: str = ""
    severity: str = "ERROR"
    timestamp_ns: int = dataclass_field(default_factory=time.monotonic_ns)

    def __post_init__(self) -> None:
        if self.stage not in PIPELINE_STAGES:
            raise ValueError(f"unknown pipeline diagnostic stage: {self.stage}")
        if self.severity not in ("OK", "WARN", "ERROR", "STALE"):
            raise ValueError(f"unknown diagnostic severity: {self.severity}")

    @property
    def key(self) -> tuple[str, str, str, str]:
        return (self.stage, self.code, self.source_joint, self.field)


class DiagnosticReporter:
    def __init__(self, history_size: int = 100) -> None:
        self.history: deque[DiagnosticIssue] = deque(maxlen=history_size)
        self.occurrences: Counter[tuple[str, str, str, str]] = Counter()
        self.last_published_ns: dict[tuple[str, str, str, str], int] = {}
        self.first_seen_ns: dict[tuple[str, str, str, str], int] = {}
        self.last_seen_ns: dict[tuple[str, str, str, str], int] = {}
        self._lock = threading.Lock()

    def report(self, issue: DiagnosticIssue) -> bool:
        with self._lock:
            self.history.append(issue)
            key = issue.key
            self.occurrences[key] += 1
            self.first_seen_ns.setdefault(key, issue.timestamp_ns)
            self.last_seen_ns[key] = issue.timestamp_ns
            previous = self.last_published_ns.get(key, 0)
            if issue.timestamp_ns - previous < 1_000_000_000:
                return False
            self.last_published_ns[key] = issue.timestamp_ns
            return True

    def latest(self) -> DiagnosticIssue | None:
        with self._lock:
            return self.history[-1] if self.history else None

    def as_dict(self, issue: DiagnosticIssue) -> dict[str, Any]:
        with self._lock:
            key = issue.key
            result = dict(issue.__dict__)
            result["first_seen_ns"] = self.first_seen_ns.get(
                key, issue.timestamp_ns
            )
            result["last_seen_ns"] = self.last_seen_ns.get(
                key, issue.timestamp_ns
            )
            result["occurrences"] = self.occurrences[key]
            return result


def rotation_matrix_is_valid(matrix: np.ndarray, tolerance: float = 1.0e-3) -> bool:
    value = np.asarray(matrix, dtype=np.float64).reshape(3, 3)
    if not np.all(np.isfinite(value)):
        return False
    determinant_error = abs(float(np.linalg.det(value)) - 1.0)
    orthogonal_error = float(np.linalg.norm(value.T @ value - np.eye(3)))
    return determinant_error <= tolerance and orthogonal_error <= tolerance


def copy_frame_arrays(frame: dict[str, np.ndarray]) -> dict[str, np.ndarray]:
    return {
        name: np.array(value, copy=True)
        for name, value in frame.items()
        if isinstance(value, np.ndarray)
    }


@dataclass
class _BundleCapture:
    issue: DiagnosticIssue
    frames: list[dict[str, np.ndarray]]
    remaining_post_frames: int
    manifest: dict[str, Any]


class DiagnosticBundleRecorder:
    """Capture bounded diagnostic history and write bundles off-thread."""

    def __init__(
        self,
        directory: str | Path,
        *,
        pre_frames: int = 100,
        post_frames: int = 50,
        queue_size: int = 2,
        max_bundles: int = 20,
        dedupe_seconds: float = 5.0,
        session_name: str | None = None,
    ) -> None:
        self.root = Path(directory)
        self.session = self.root / (
            session_name or time.strftime("%Y%m%d_%H%M%S")
        )
        self.pre_frames = max(1, int(pre_frames))
        self.post_frames = max(0, int(post_frames))
        self.max_bundles = max(1, int(max_bundles))
        self.dedupe_ns = int(max(0.0, dedupe_seconds) * 1.0e9)
        self.history: deque[dict[str, np.ndarray]] = deque(
            maxlen=self.pre_frames
        )
        self.active: _BundleCapture | None = None
        self.last_trigger_ns: dict[tuple[str, str, str, str], int] = {}
        self.queue: queue.Queue[_BundleCapture | None] = queue.Queue(
            maxsize=max(1, int(queue_size))
        )
        self.dropped_requests = 0
        self.write_failures = 0
        self.last_write_error = ""
        self._lock = threading.Lock()
        self._worker = threading.Thread(
            target=self._worker_main,
            name="sonic-pnlink-diagnostic-writer",
            daemon=True,
        )
        self._worker.start()

    def add_frame(self, frame: dict[str, np.ndarray]) -> None:
        snapshot = copy_frame_arrays(frame)
        completed: _BundleCapture | None = None
        with self._lock:
            self.history.append(snapshot)
            if self.active is not None:
                self.active.frames.append(snapshot)
                self.active.remaining_post_frames -= 1
                if self.active.remaining_post_frames <= 0:
                    completed = self.active
                    self.active = None
        if completed is not None:
            self._enqueue(completed)

    def trigger(
        self,
        issue: DiagnosticIssue,
        manifest: dict[str, Any] | None = None,
    ) -> bool:
        with self._lock:
            previous = self.last_trigger_ns.get(issue.key, 0)
            if issue.timestamp_ns - previous < self.dedupe_ns:
                return False
            if self.active is not None:
                return False
            self.last_trigger_ns[issue.key] = issue.timestamp_ns
            self.active = _BundleCapture(
                issue=issue,
                frames=[copy_frame_arrays(frame) for frame in self.history],
                remaining_post_frames=self.post_frames,
                manifest=dict(manifest or {}),
            )
            if self.post_frames > 0:
                return True
            capture = self.active
            self.active = None
        return self._enqueue(capture)

    def dump_now(
        self,
        issue: DiagnosticIssue,
        manifest: dict[str, Any] | None = None,
    ) -> bool:
        with self._lock:
            capture = _BundleCapture(
                issue,
                [copy_frame_arrays(frame) for frame in self.history],
                0,
                dict(manifest or {}),
            )
        return self._enqueue(capture)

    def _enqueue(self, capture: _BundleCapture) -> bool:
        try:
            self.queue.put_nowait(capture)
            return True
        except queue.Full:
            self.dropped_requests += 1
            return False

    def _worker_main(self) -> None:
        while True:
            capture = self.queue.get()
            try:
                if capture is None:
                    return
                try:
                    self._write_bundle(capture)
                except Exception as exc:
                    self.write_failures += 1
                    self.last_write_error = str(exc)
            finally:
                self.queue.task_done()

    @staticmethod
    def _stack_group(
        frames: list[dict[str, np.ndarray]], prefixes: tuple[str, ...]
    ) -> dict[str, np.ndarray]:
        keys = sorted(
            {
                key
                for frame in frames
                for key in frame
                if key.startswith(prefixes)
            }
        )
        result: dict[str, np.ndarray] = {}
        for key in keys:
            values = [frame[key] for frame in frames if key in frame]
            if len(values) != len(frames):
                continue
            try:
                result[key] = np.stack(values)
            except ValueError:
                continue
        return result

    def _write_bundle(self, capture: _BundleCapture) -> Path:
        self.session.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        suffix = f"{time.time_ns() % 1_000_000_000:09d}"
        bundle = self.session / (
            f"{stamp}_{suffix}_{capture.issue.stage}_{capture.issue.code}"
        )
        bundle.mkdir(parents=False, exist_ok=False)
        groups = {
            "raw_frames.npz": self._stack_group(capture.frames, ("raw_",)),
            "retarget_frames.npz": self._stack_group(
                capture.frames, ("retarget_", "smpl_local_")
            ),
            "output_frames.npz": self._stack_group(
                capture.frames,
                (
                    "frame_index", "timestamp_", "smpl_joints",
                    "smpl_root_", "smpl_hand_", "wrist",
                    "source_stage_",
                ),
            ),
        }
        arrays = {
            key: {name: list(value.shape) for name, value in values.items()}
            for key, values in groups.items()
        }
        manifest = dict(capture.manifest)
        manifest.update(
            {
                "schema_version": 1,
                "trigger_issue": dict(capture.issue.__dict__),
                "frame_count": len(capture.frames),
                "post_frames_incomplete": capture.remaining_post_frames > 0,
                "arrays": arrays,
            }
        )
        (bundle / "manifest.json").write_text(
            json.dumps(manifest, indent=2, sort_keys=True), encoding="utf-8"
        )
        (bundle / "issues.jsonl").write_text(
            json.dumps(dict(capture.issue.__dict__), sort_keys=True) + "\n",
            encoding="utf-8",
        )
        for filename, values in groups.items():
            np.savez_compressed(bundle / filename, **values)
        self._rotate_bundles()
        return bundle

    def _rotate_bundles(self) -> None:
        bundles = sorted(path for path in self.session.iterdir() if path.is_dir())
        for old in bundles[:-self.max_bundles]:
            shutil.rmtree(old)

    def wait(self) -> None:
        self.queue.join()

    def close(self) -> None:
        partial: _BundleCapture | None = None
        with self._lock:
            if self.active is not None:
                partial = self.active
                self.active = None
        if partial is not None:
            self._enqueue(partial)
        self.queue.join()
        self.queue.put(None)
        self.queue.join()
        self._worker.join(timeout=2.0)
