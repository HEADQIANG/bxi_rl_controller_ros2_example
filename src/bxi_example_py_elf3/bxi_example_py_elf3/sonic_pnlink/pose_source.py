"""Pure-Python PN-Link source that publishes normalized SONIC ``pose`` windows."""

from __future__ import annotations

from collections import deque
from enum import Enum
import json
import os
from pathlib import Path
import select
import sys
import termios
import threading
import time
from typing import Any
import tty

import numpy as np
from scipy.spatial.transform import Rotation
import zmq

from bxi_example_py_elf3.sonic_pico.zmq_messages import pack_pose_message

from .calibration import (
    NeutralCalibration,
    calibrate_neutral,
    median_bone_lengths_cm,
)
from .debug_wire import DebugPublisher, build_debug_snapshot
from .diagnostics import (
    DiagnosticBundleRecorder,
    DiagnosticIssue,
    DiagnosticReporter,
)
from .retarget import (
    DEFAULT_HAND_FLOOR_THRESHOLD_M,
    RetargetResult,
    process_smpl_frame,
    retarget_rotations,
    smpl_hand_floor_contact,
)
from .sdk_adapter import (
    CALIBRATE_MOTION,
    CLEAR_ZERO_DRIFT,
    RESUME_BODY,
    RESUME_HANDS,
    START_CAPTURE,
    STOP_CAPTURE,
    ZERO_POSITION,
    PnLinkSdkAdapter,
    RawAvatarFrame,
    SdkCommandResult,
    SdkConfig,
)
from .skeleton import (
    PNLINK_JOINT_NAMES,
    PNLINK_PARENTS,
    REQUIRED_PNLINK_JOINTS,
    forward_kinematics,
)
from .temporal import (
    SOURCE_STALE_NS,
    StandardFrame,
    StandardFrameResampler,
    build_pose_window,
)
from .validation import (
    validate_raw_poses,
    validate_retarget_output,
    validate_world_pose,
)

WINDOW = 10
NEUTRAL_SAMPLE_COUNT = 25
SOURCE_FRAME_RATE_HZ = 50.0


class SourceState(str, Enum):
    DISCONNECTED = "DISCONNECTED"
    CAPTURING = "CAPTURING"
    VENDOR_CALIBRATING = "VENDOR_CALIBRATING"
    NEEDS_NEUTRAL = "NEEDS_NEUTRAL"
    READY_PAUSED = "READY_PAUSED"
    LIVE = "LIVE"


class ConsoleLogger:
    def _write(self, level: str, message: str) -> None:
        print(f"[pnlink-source] {level} {message}", flush=True)

    def info(self, message: str) -> None:
        self._write("INFO", message)

    def warning(self, message: str) -> None:
        self._write("WARN", message)

    def error(self, message: str) -> None:
        self._write("ERROR", message)


def _format_issue_message(issue: DiagnosticIssue) -> str:
    location: list[str] = []
    if issue.source_joint:
        location.append(f"joint={issue.source_joint}")
    if issue.source_parent:
        location.append(f"parent={issue.source_parent}")
    if issue.smpl_joint:
        smpl = issue.smpl_joint
        if issue.smpl_index >= 0:
            smpl = f"{smpl}[{issue.smpl_index}]"
        location.append(f"smpl={smpl}")
    context = f" [{', '.join(location)}]" if location else ""
    return f"{issue.stage}/{issue.code}{context}: {issue.observed}"


class TerminalKeyInput:
    """Non-blocking single-key input with terminal restoration on shutdown."""

    def __init__(self, stream: Any = None) -> None:
        self.stream = stream if stream is not None else sys.stdin
        self.fd: int | None = None
        self.old_settings: list[Any] | None = None

    def __enter__(self) -> "TerminalKeyInput":
        if not hasattr(self.stream, "fileno"):
            return self
        try:
            self.fd = self.stream.fileno()
        except (OSError, ValueError):
            self.fd = None
            return self
        if self.stream.isatty():
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def read(self) -> str | None:
        if self.fd is None:
            return None
        readable, _, _ = select.select([self.stream], [], [], 0.0)
        if not readable:
            return None
        value = self.stream.read(1)
        return value or None

    def __exit__(self, *_args: Any) -> None:
        if self.fd is not None and self.old_settings is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


class SonicPnLinkPoseSource:
    def __init__(self) -> None:
        self.logger = ConsoleLogger()
        self._lock = threading.RLock()
        self.state = SourceState.DISCONNECTED
        self.vendor_calibrated = False
        self.capture_requested = False
        self.neutral: NeutralCalibration | None = None
        self.live_enabled = False
        self.last_frame_ns = 0
        self.last_realtime = 0.0
        self.frame_count = 0
        self.source_started_ns = time.monotonic_ns()
        self.neutral_samples: list[tuple[int, dict[str, Rotation]]] | None = None
        self.neutral_bone_length_samples_cm: list[dict[str, float]] | None = None
        self.reporter = DiagnosticReporter()
        self.failed_stages: set[str] = set()
        self.resampler = StandardFrameResampler()
        self.window: deque[tuple[int, StandardFrame]] = deque(maxlen=WINDOW)
        self.pending_pose: deque[dict[str, np.ndarray]] = deque(maxlen=4)
        self.pending_debug: dict[str, np.ndarray] | None = None
        self.next_frame_index = 0
        self.previous_root_quat: np.ndarray | None = None
        self.previous_raw_poses = None
        self.previous_source_timestamp_ns = 0
        self.neutral_bone_lengths_cm: dict[str, float] = {}
        self.smpl_neutral_lengths_m: dict[int, float] = {}
        self.missing_joints: list[str] = []
        self.capture_active = False
        self.stop_requested = False
        self._last_status_json = ""
        self._last_status_time = 0.0
        self.hand_floor_threshold_m = float(
            os.environ.get(
                "SONIC_PNLINK_HAND_FLOOR_THRESHOLD_M",
                str(DEFAULT_HAND_FLOOR_THRESHOLD_M),
            )
        )
        if (
            not np.isfinite(self.hand_floor_threshold_m)
            or self.hand_floor_threshold_m < 0.0
        ):
            raise RuntimeError(
                "SONIC_PNLINK_HAND_FLOOR_THRESHOLD_M must be finite and non-negative"
            )
        self.smpl_foot_plane_z_m = float("nan")
        self.smpl_hand_floor_gap_m = np.full(2, np.nan, dtype=np.float32)
        self.smpl_hand_floor_contact = np.zeros(2, dtype=bool)
        self._previous_hand_floor_contact: np.ndarray | None = None

        bind_host = os.environ.get("SONIC_PNLINK_BIND_HOST", "127.0.0.1")
        if bind_host not in ("127.0.0.1", "localhost"):
            raise RuntimeError("SONIC_PNLINK_BIND_HOST must be loopback in local topology")
        port = int(os.environ.get("SONIC_PNLINK_PORT", "5556"))
        self.zmq_context = zmq.Context()
        self.pose_socket = self.zmq_context.socket(zmq.PUB)
        self.pose_socket.setsockopt(zmq.SNDHWM, 4)
        self.pose_socket.setsockopt(zmq.LINGER, 0)
        self.pose_socket.bind(f"tcp://{bind_host}:{port}")

        self.debug_publisher: DebugPublisher | None = None
        self.bundle_recorder: DiagnosticBundleRecorder | None = None
        self.bundle_drops_reported = 0
        self.bundle_failures_reported = 0
        if _env_flag("SONIC_PNLINK_DIAGNOSTICS", False):
            debug_host = os.environ.get("SONIC_PNLINK_DEBUG_HOST", "127.0.0.1")
            if debug_host not in ("127.0.0.1", "localhost"):
                raise RuntimeError("SONIC_PNLINK_DEBUG_HOST must be loopback")
            debug_port = int(os.environ.get("SONIC_PNLINK_DEBUG_PORT", "5558"))
            self.debug_publisher = DebugPublisher(
                self.zmq_context, f"tcp://{debug_host}:{debug_port}"
            )
            self.bundle_recorder = DiagnosticBundleRecorder(
                os.environ.get(
                    "SONIC_PNLINK_DIAG_DIR",
                    "/tmp/sonic_pnlink_diagnostics",
                )
            )

        try:
            import torch

            torch.set_num_threads(1)
            torch.set_num_interop_threads(1)
        except (ImportError, RuntimeError) as exc:
            self.logger.warning(f"unable to configure torch thread limits: {exc}")

        sdk_path_value = os.environ.get("SONIC_PNLINK_SDK_PATH", "").strip()
        if not sdk_path_value:
            raise RuntimeError("SONIC_PNLINK_SDK_PATH is required")
        sdk_path = Path(sdk_path_value)
        self.sdk = PnLinkSdkAdapter(
            SdkConfig(
                sdk_path=sdk_path,
                local_ip=os.environ.get("PNLINK_LOCAL_IP", "10.42.0.101"),
                local_port=int(os.environ.get("PNLINK_LOCAL_PORT", "8002")),
                server_ip=os.environ.get("PNLINK_SERVER_IP", "10.42.0.202"),
                server_port=int(os.environ.get("PNLINK_SERVER_PORT", "8080")),
            ),
            self._on_avatar_frame,
            self._on_sdk_command_result,
            self._on_sdk_command_progress,
        )
        self.sdk.start()
        self.logger.info(f"PN-Link source listening; pose PUB tcp://{bind_host}:{port}")

    def _clear_live_buffers(self) -> None:
        self.resampler.reset()
        self.window.clear()
        self.pending_pose.clear()
        self.next_frame_index = 0
        self.previous_root_quat = None

    def _clear_hand_floor_contact(self) -> None:
        self.smpl_foot_plane_z_m = float("nan")
        self.smpl_hand_floor_gap_m = np.full(2, np.nan, dtype=np.float32)
        self.smpl_hand_floor_contact = np.zeros(2, dtype=bool)
        self._previous_hand_floor_contact = None

    def start_capture(self) -> tuple[bool, str]:
        with self._lock:
            if self.capture_active:
                return True, "capture already active"
            if self.capture_requested:
                return False, "start capture command is already pending"
            self.capture_requested = True
        try:
            self.sdk.start_capture()
        except Exception as exc:
            with self._lock:
                self.capture_requested = False
            return False, str(exc)
        return True, "start capture command accepted; waiting for AvatarUpdated"

    def stop_capture(self) -> tuple[bool, str]:
        with self._lock:
            if not self.capture_active:
                return False, "capture is not active"
        try:
            self.sdk.stop_capture()
        except Exception as exc:
            return False, str(exc)
        return True, "stop capture command accepted"

    def start_vendor_calibration(self) -> tuple[bool, str]:
        with self._lock:
            if not self.capture_active or self.state == SourceState.DISCONNECTED:
                return False, "start capture before vendor calibration"
            self.state = SourceState.VENDOR_CALIBRATING
            self.vendor_calibrated = False
            self.neutral = None
            self.neutral_samples = None
            self.neutral_bone_length_samples_cm = None
            self.live_enabled = False
            self.previous_raw_poses = None
            self.neutral_bone_lengths_cm.clear()
            self.smpl_neutral_lengths_m.clear()
            self._clear_live_buffers()
            self._clear_hand_floor_contact()
        try:
            self.sdk.calibrate_motion()
        except Exception as exc:
            with self._lock:
                self.state = SourceState.CAPTURING
            return False, str(exc)
        self._publish_status(force=True)
        return True, "vendor calibration command accepted"

    def run_vendor_command(self, command: str) -> tuple[bool, str]:
        with self._lock:
            if not self.capture_active or self.state == SourceState.DISCONNECTED:
                return False, "start capture before issuing PN-Link commands"
        methods = {
            RESUME_HANDS: self.sdk.resume_hands,
            CLEAR_ZERO_DRIFT: self.sdk.clear_zero_drift,
            RESUME_BODY: self.sdk.resume_body,
            ZERO_POSITION: self.sdk.zero_position,
        }
        try:
            methods[command]()
        except Exception as exc:
            return False, str(exc)
        return True, f"{command} command accepted"

    def _on_sdk_command_result(self, result: SdkCommandResult) -> None:
        with self._lock:
            if result.command == START_CAPTURE:
                self.capture_requested = False
                message = f"PN-Link start capture: {result.message}"
                if result.success:
                    self.capture_active = True
                    self.logger.info(message)
                else:
                    self.logger.error(message)
            elif result.command == STOP_CAPTURE:
                if result.success:
                    self.capture_active = False
                    self.live_enabled = False
                    self.state = SourceState.DISCONNECTED
                    self._clear_live_buffers()
                    self._clear_hand_floor_contact()
                    self.logger.info(f"PN-Link stop capture: {result.message}")
                else:
                    self.logger.error(f"PN-Link stop capture failed: {result.message}")
            elif result.command == CALIBRATE_MOTION:
                if result.success:
                    self.vendor_calibrated = True
                    self.state = SourceState.NEEDS_NEUTRAL
                    # The vendor changes joint reference frames when calibration
                    # completes. Do not compare the first calibrated frame with
                    # a frame captured under the old reference coordinates.
                    self.previous_raw_poses = None
                    self.logger.info(
                        "PN-Link vendor calibration complete; capture neutral T-pose next"
                    )
                else:
                    self.vendor_calibrated = False
                    self.state = SourceState.CAPTURING
                    self.logger.error(
                        f"PN-Link vendor calibration failed: {result.message}"
                    )
            elif result.success:
                self.logger.info(f"PN-Link {result.command}: {result.message}")
            else:
                self.logger.error(f"PN-Link {result.command} failed: {result.message}")
        self._publish_status(force=True)

    def _on_sdk_command_progress(self, message: str) -> None:
        self.logger.info(message)

    def capture_neutral(self) -> tuple[bool, str]:
        with self._lock:
            allowed_states = (
                SourceState.CAPTURING,
                SourceState.NEEDS_NEUTRAL,
                SourceState.READY_PAUSED,
            )
            if self.state not in allowed_states:
                hint = "; press P before T" if self.state == SourceState.LIVE else ""
                return False, (
                    "neutral capture requires active paused capture, "
                    f"current={self.state.value}{hint}"
                )
            self.state = SourceState.NEEDS_NEUTRAL
            self.neutral = None
            self.live_enabled = False
            self.neutral_bone_lengths_cm.clear()
            self.smpl_neutral_lengths_m.clear()
            self._clear_live_buffers()
            self._clear_hand_floor_contact()
            self.neutral_samples = []
            self.neutral_bone_length_samples_cm = []
        self._publish_status(force=True)
        return True, f"collecting {NEUTRAL_SAMPLE_COUNT} neutral frames"

    def set_live(self, enabled: bool) -> tuple[bool, str]:
        with self._lock:
            if not enabled:
                self.live_enabled = False
                self._clear_live_buffers()
                if self.neutral is not None:
                    self.state = SourceState.READY_PAUSED
                result = True, "live disabled"
            elif self.state != SourceState.READY_PAUSED or self.neutral is None:
                result = (
                    False,
                    f"live requires READY_PAUSED, current={self.state.value}",
                )
            else:
                self._clear_live_buffers()
                self.live_enabled = True
                self.state = SourceState.LIVE
                result = True, "live enabled; filling 10-frame window"
        self._publish_status(force=True)
        return result

    def _handle_issue(self, issue: DiagnosticIssue) -> None:
        if issue.severity in ("ERROR", "STALE"):
            self.failed_stages.add(issue.stage)
        if self.reporter.report(issue):
            message = _format_issue_message(issue)
            if issue.severity == "OK":
                self.logger.info(message)
            elif issue.severity == "WARN":
                self.logger.warning(message)
            else:
                self.logger.error(message)
        if issue.severity == "ERROR" and self.bundle_recorder is not None:
            accepted = self.bundle_recorder.trigger(
                issue,
                manifest={
                    "source_state": self.state.value,
                    "pnlink_joint_names": list(PNLINK_JOINT_NAMES),
                    "environment": {
                        name: os.environ.get(name, "")
                        for name in (
                            "SONIC_TELEOP_SOURCE",
                            "SONIC_PNLINK_TOPOLOGY",
                            "SONIC_PNLINK_PORT",
                            "SONIC_PNLINK_DIAGNOSTICS",
                        )
                    },
                },
            )
            if not accepted and self.bundle_recorder.dropped_requests:
                self.logger.warning(
                    "diagnostic bundle request dropped because writer queue is full"
                )

    def _recover_stage(self, stage: str, frame_index: int) -> None:
        if stage not in self.failed_stages:
            return
        self.failed_stages.remove(stage)
        self._handle_issue(
            DiagnosticIssue(
                stage, "OK", "state", "RECOVERED",
                frame_index=frame_index, observed="stage recovered", severity="OK"
            )
        )

    def _queue_debug_snapshot(self, snapshot: dict[str, np.ndarray]) -> None:
        if self.debug_publisher is not None:
            with self._lock:
                self.pending_debug = snapshot
        if self.bundle_recorder is not None:
            self.bundle_recorder.add_frame(snapshot)

    def _on_avatar_frame(self, frame: RawAvatarFrame) -> None:
        with self._lock:
            self.frame_count += 1
            frame_index = self.frame_count
        timestamp_regressed = (
            self.previous_source_timestamp_ns > 0
            and frame.timestamp_monotonic_ns <= self.previous_source_timestamp_ns
        )
        if timestamp_regressed:
            self._handle_issue(
                DiagnosticIssue(
                    "TEMPORAL_BUFFER", "TIMESTAMP_REGRESSION", "timestamp",
                    "FRAME_DROPPED", frame_index=frame_index,
                    observed=str(frame.timestamp_monotonic_ns),
                    expected=f"> {self.previous_source_timestamp_ns}",
                )
            )
        else:
            self.previous_source_timestamp_ns = frame.timestamp_monotonic_ns
        raw_validation = validate_raw_poses(
            frame.joints,
            frame_index=frame_index,
            previous=self.previous_raw_poses,
            neutral_bone_lengths_cm=(
                self.neutral_bone_lengths_cm if self.neutral is not None else None
            ),
        )
        for issue in raw_validation.issues:
            self._handle_issue(issue)
        self.missing_joints = [
            PNLINK_JOINT_NAMES[index]
            for index, present in enumerate(raw_validation.present)
            if not present and PNLINK_JOINT_NAMES[index] in REQUIRED_PNLINK_JOINTS
        ]
        if raw_validation.has_error or timestamp_regressed:
            snapshot = build_debug_snapshot(
                frame_index=frame_index,
                timestamp_monotonic_ns=frame.timestamp_monotonic_ns,
                joints=frame.joints,
                raw_present=raw_validation.present,
                raw_position_valid=raw_validation.position_valid,
                raw_rotation_valid=raw_validation.rotation_valid,
                source_stage_valid=np.zeros(7, dtype=bool),
            )
            self._queue_debug_snapshot(snapshot)
            return
        self._recover_stage("SDK_RAW", frame_index)
        self._recover_stage("TEMPORAL_BUFFER", frame_index)

        try:
            world = forward_kinematics(frame.joints)
        except Exception as exc:
            self._handle_issue(
                DiagnosticIssue(
                    "PNLINK_FK", "FK_NONFINITE", "rotation", "FRAME_DROPPED",
                    frame_index=frame_index,
                    observed=str(exc),
                )
            )
            self._queue_debug_snapshot(
                build_debug_snapshot(
                    frame_index=frame_index,
                    timestamp_monotonic_ns=frame.timestamp_monotonic_ns,
                    joints=frame.joints,
                    raw_present=raw_validation.present,
                    raw_position_valid=raw_validation.position_valid,
                    raw_rotation_valid=raw_validation.rotation_valid,
                    source_stage_valid=np.array(
                        [True, False, False, False, False, False, False]
                    ),
                )
            )
            return
        world_issues = validate_world_pose(world, frame_index=frame_index)
        for issue in world_issues:
            self._handle_issue(issue)
        if any(issue.severity == "ERROR" for issue in world_issues):
            self._queue_debug_snapshot(
                build_debug_snapshot(
                    frame_index=frame_index,
                    timestamp_monotonic_ns=frame.timestamp_monotonic_ns,
                    joints=frame.joints,
                    world=world,
                    raw_present=raw_validation.present,
                    raw_position_valid=raw_validation.position_valid,
                    raw_rotation_valid=raw_validation.rotation_valid,
                    source_stage_valid=np.array(
                        [True, False, False, False, False, False, False]
                    ),
                )
            )
            return
        self._recover_stage("PNLINK_FK", frame_index)

        with self._lock:
            self.last_frame_ns = frame.timestamp_monotonic_ns
            self.last_realtime = frame.timestamp_realtime
            self.missing_joints = []
            became_connected = self.state == SourceState.DISCONNECTED
            if self.state == SourceState.DISCONNECTED:
                self.capture_active = True
                self.state = SourceState.CAPTURING
            if self.neutral_samples is not None:
                bone_lengths_cm = {
                    name: float(np.linalg.norm(frame.joints[name].position_cm))
                    for name, parent in PNLINK_PARENTS.items()
                    if parent is not None and name in frame.joints
                }
                self.neutral_samples.append(
                    (frame.timestamp_monotonic_ns, world.rotations_pnlink)
                )
                if self.neutral_bone_length_samples_cm is None:
                    self.neutral_bone_length_samples_cm = []
                self.neutral_bone_length_samples_cm.append(bone_lengths_cm)
                if len(self.neutral_samples) == NEUTRAL_SAMPLE_COUNT:
                    samples = self.neutral_samples
                    bone_length_samples = self.neutral_bone_length_samples_cm
                    self.neutral_samples = None
                    self.neutral_bone_length_samples_cm = None
                    try:
                        neutral = calibrate_neutral(samples)
                        if bone_length_samples is None:
                            raise RuntimeError("neutral bone-length samples are missing")
                        bone_lengths = median_bone_lengths_cm(bone_length_samples)
                    except Exception as exc:
                        self._handle_issue(
                            DiagnosticIssue(
                                "CALIBRATION", "NEUTRAL_UNSTABLE", "rotation",
                                "FRAME_DROPPED", frame_index=frame_index,
                                observed=str(exc),
                            )
                        )
                    else:
                        self.neutral = neutral
                        self.neutral_bone_lengths_cm = bone_lengths
                        self.state = SourceState.READY_PAUSED
                        self.logger.info(
                            "PN-Link neutral calibration complete; "
                            f"bone_length_baselines={len(bone_lengths)}"
                        )
                        self._recover_stage("CALIBRATION", frame_index)
                        self._publish_status(force=True)

            neutral = self.neutral
            should_retarget = neutral is not None and self.state in (
                SourceState.READY_PAUSED, SourceState.LIVE
            )
        if became_connected:
            self._publish_status(force=True)
        if not should_retarget:
            self.previous_raw_poses = dict(frame.joints)
            self._queue_debug_snapshot(
                build_debug_snapshot(
                    frame_index=frame_index,
                    timestamp_monotonic_ns=frame.timestamp_monotonic_ns,
                    joints=frame.joints,
                    world=world,
                    raw_present=raw_validation.present,
                    raw_position_valid=raw_validation.position_valid,
                    raw_rotation_valid=raw_validation.rotation_valid,
                    source_stage_valid=np.array(
                        [True, True, False, False, False, False, False]
                    ),
                )
            )
            return

        try:
            smpl_global, local_axis_angle = retarget_rotations(
                world.rotations_pnlink, neutral.rotations
            )
        except Exception as exc:
            self._handle_issue(
                DiagnosticIssue(
                    "SMPL_RETARGET", "ROTATION_NONFINITE", "rotation",
                    "FRAME_DROPPED", frame_index=frame_index, observed=str(exc)
                )
            )
            self._queue_debug_snapshot(
                build_debug_snapshot(
                    frame_index=frame_index,
                    timestamp_monotonic_ns=frame.timestamp_monotonic_ns,
                    joints=frame.joints,
                    world=world,
                    raw_present=raw_validation.present,
                    raw_position_valid=raw_validation.position_valid,
                    raw_rotation_valid=raw_validation.rotation_valid,
                    source_stage_valid=np.array(
                        [True, True, True, False, False, False, False]
                    ),
                )
            )
            return
        try:
            result = process_smpl_frame(smpl_global, local_axis_angle)
        except Exception as exc:
            self._handle_issue(
                DiagnosticIssue(
                    "SMPL_FK", "SMPL_FK_NONFINITE", "position",
                    "FRAME_DROPPED", frame_index=frame_index, observed=str(exc)
                )
            )
            self._queue_debug_snapshot(
                build_debug_snapshot(
                    frame_index=frame_index,
                    timestamp_monotonic_ns=frame.timestamp_monotonic_ns,
                    joints=frame.joints,
                    world=world,
                    raw_present=raw_validation.present,
                    raw_position_valid=raw_validation.position_valid,
                    raw_rotation_valid=raw_validation.rotation_valid,
                    source_stage_valid=np.array(
                        [True, True, True, True, False, False, False]
                    ),
                )
            )
            return
        output_issues = validate_retarget_output(
            result,
            frame_index=frame_index,
            neutral_bone_lengths_m=self.smpl_neutral_lengths_m,
        )
        for issue in output_issues:
            self._handle_issue(issue)
        if any(issue.severity == "ERROR" for issue in output_issues):
            snapshot = build_debug_snapshot(
                frame_index=frame_index,
                timestamp_monotonic_ns=frame.timestamp_monotonic_ns,
                joints=frame.joints,
                world=world,
                result=result,
                raw_present=raw_validation.present,
                raw_position_valid=raw_validation.position_valid,
                raw_rotation_valid=raw_validation.rotation_valid,
                source_stage_valid=np.array(
                    [True, True, True, True, False, False, False]
                ),
            )
            if any(issue.stage == "SMPL_FK" for issue in output_issues):
                snapshot["smpl_position_valid"][:] = False
            if any(issue.stage == "SMPL_RETARGET" for issue in output_issues):
                snapshot["smpl_rotation_valid"][:] = False
            self._queue_debug_snapshot(snapshot)
            return
        self._recover_stage("SMPL_RETARGET", frame_index)
        self._recover_stage("SMPL_FK", frame_index)
        self._update_hand_floor_contact(result)
        self.previous_raw_poses = dict(frame.joints)
        self._queue_debug(
            frame, world, result, raw_validation, frame_index
        )

        with self._lock:
            if self.state != SourceState.LIVE or not self.live_enabled:
                return
            root = result.root_quaternion_wxyz.copy()
            if self.previous_root_quat is not None and np.dot(root, self.previous_root_quat) < 0.0:
                root *= -1.0
            self.previous_root_quat = root
            standardized = StandardFrame(
                frame.timestamp_monotonic_ns,
                frame.timestamp_realtime,
                result.smpl_joints_local,
                root,
                result.wrist,
            )
            for sampled in self.resampler.add(standardized):
                frame_index = self.next_frame_index
                self.next_frame_index += 1
                self.window.append((frame_index, sampled))
                if len(self.window) == WINDOW:
                    self.pending_pose.append(self._build_pose_message())

    def _build_pose_message(self) -> dict[str, np.ndarray]:
        return build_pose_window(list(self.window))

    def _queue_debug(
        self,
        frame: RawAvatarFrame,
        world: Any,
        result: RetargetResult,
        raw_validation: Any,
        frame_index: int,
    ) -> None:
        self._queue_debug_snapshot(
            build_debug_snapshot(
                frame_index=frame_index,
                timestamp_monotonic_ns=frame.timestamp_monotonic_ns,
                joints=frame.joints,
                world=world,
                result=result,
                raw_present=raw_validation.present,
                raw_position_valid=raw_validation.position_valid,
                raw_rotation_valid=raw_validation.rotation_valid,
                source_stage_valid=np.ones(7, dtype=bool),
                hand_floor_threshold_m=self.hand_floor_threshold_m,
            )
        )

    def _update_hand_floor_contact(self, result: RetargetResult) -> None:
        measurement = smpl_hand_floor_contact(
            result.smpl_joints_local,
            result.root_quaternion_wxyz,
            threshold_m=self.hand_floor_threshold_m,
        )
        with self._lock:
            previous = self._previous_hand_floor_contact
            self.smpl_foot_plane_z_m = measurement.foot_plane_z_m
            self.smpl_hand_floor_gap_m = measurement.hand_gap_m.copy()
            self.smpl_hand_floor_contact = measurement.contact.copy()
            changed = previous is None or not np.array_equal(
                previous, measurement.contact
            )
            self._previous_hand_floor_contact = measurement.contact.copy()
        if changed:
            self.logger.info(
                "SMPL hand-floor "
                f"left={bool(measurement.contact[0])} "
                f"gap={measurement.hand_gap_m[0]:+.3f}m, "
                f"right={bool(measurement.contact[1])} "
                f"gap={measurement.hand_gap_m[1]:+.3f}m, "
                f"threshold={self.hand_floor_threshold_m:.3f}m"
            )

    def _tick(self) -> None:
        self.sdk.poll()
        now = time.monotonic_ns()
        if (
            self.bundle_recorder is not None
            and self.bundle_recorder.dropped_requests > self.bundle_drops_reported
        ):
            self.bundle_drops_reported = self.bundle_recorder.dropped_requests
            self._handle_issue(
                DiagnosticIssue(
                    "POSE_WIRE", "DIAGNOSTIC_DUMP_DROPPED", "wire", "DROPPED",
                    frame_index=self.frame_count,
                    observed=f"dropped={self.bundle_drops_reported}",
                    expected="diagnostic writer queue capacity=2",
                    severity="WARN",
                )
            )
        if (
            self.bundle_recorder is not None
            and self.bundle_recorder.write_failures > self.bundle_failures_reported
        ):
            self.bundle_failures_reported = self.bundle_recorder.write_failures
            self._handle_issue(
                DiagnosticIssue(
                    "POSE_WIRE", "DIAGNOSTIC_DUMP_FAILED", "wire", "DROPPED",
                    frame_index=self.frame_count,
                    observed=self.bundle_recorder.last_write_error,
                    expected="diagnostic bundle written asynchronously",
                    severity="WARN",
                )
            )
        with self._lock:
            if self.state == SourceState.LIVE and now - self.last_frame_ns > SOURCE_STALE_NS:
                self.live_enabled = False
                self.state = SourceState.READY_PAUSED
                self._clear_live_buffers()
                self._handle_issue(
                    DiagnosticIssue(
                        "TEMPORAL_BUFFER", "SOURCE_STALE", "timestamp",
                        "LIVE_REVOKED", frame_index=self.frame_count,
                        observed="no AvatarUpdated frame for 0.1 seconds",
                        severity="STALE",
                    )
                )
            pose = self.pending_pose.popleft() if self.pending_pose else None
            debug = self.pending_debug
            self.pending_debug = None
        if pose is not None:
            try:
                message = pack_pose_message(pose, topic="pose", version=4)
                self.pose_socket.send(message)
                self._recover_stage("POSE_WIRE", self.frame_count)
            except Exception as exc:
                self._handle_issue(
                    DiagnosticIssue(
                        "POSE_WIRE", "POSE_SCHEMA_INVALID", "wire",
                        "FRAME_DROPPED", frame_index=self.frame_count,
                        observed=str(exc),
                    )
                )
        if debug is not None and self.debug_publisher is not None:
            try:
                self.debug_publisher.send(debug)
            except Exception as exc:
                self.logger.warning(
                    f"debug frame dropped without affecting control: {exc}"
                )

    def _status_payload(self) -> dict[str, Any]:
        now = time.monotonic_ns()
        age_ms = (now - self.last_frame_ns) / 1.0e6 if self.last_frame_ns else -1.0
        elapsed = max((now - self.source_started_ns) / 1.0e9, 1.0e-6)
        neutral_samples_collected = (
            len(self.neutral_samples) if self.neutral_samples is not None else 0
        )
        instruction, countdown_seconds = _calibration_guidance(
            self.state, neutral_samples_collected
        )
        return {
            "state": self.state.value,
            "sdk_connected": self.last_frame_ns > 0 and age_ms <= 100.0,
            "capture_requested": self.capture_requested,
            "capture_active": self.capture_active,
            "vendor_calibrated": self.vendor_calibrated,
            "neutral_calibrated": self.neutral is not None,
            "live_enabled": self.live_enabled,
            "source_hz": round(self.frame_count / elapsed, 2),
            "last_frame_age_ms": round(age_ms, 2),
            "missing_joints": self.missing_joints,
            "instruction": instruction,
            "countdown_seconds": countdown_seconds,
            "neutral_samples_collected": neutral_samples_collected,
            "neutral_samples_required": NEUTRAL_SAMPLE_COUNT,
            "smpl_foot_plane_z_m": (
                round(self.smpl_foot_plane_z_m, 4)
                if np.isfinite(self.smpl_foot_plane_z_m)
                else None
            ),
            "smpl_hand_floor_gap_m": [
                round(float(value), 4) if np.isfinite(value) else None
                for value in self.smpl_hand_floor_gap_m
            ],
            "smpl_hand_floor_contact": self.smpl_hand_floor_contact.tolist(),
            "smpl_hand_floor_threshold_m": self.hand_floor_threshold_m,
        }

    def _publish_status(self, *, force: bool = False) -> None:
        now = time.monotonic()
        if not force:
            if self.state == SourceState.VENDOR_CALIBRATING:
                return
            if now - self._last_status_time < 2.0:
                return
        encoded = json.dumps(
            self._status_payload(),
            ensure_ascii=False,
            separators=(",", ":"),
        )
        print(f"[pnlink-source] STATUS {encoded}", flush=True)
        self._last_status_json = encoded
        self._last_status_time = now

    def handle_key(self, key: str) -> bool:
        key = key.lower()
        if key == "\x1b":
            self.logger.info("ESC pressed; exiting")
            return False
        actions = {
            "n": self.start_capture,
            "f": self.stop_capture,
            "c": self.start_vendor_calibration,
            "t": self.capture_neutral,
            "l": lambda: self.set_live(True),
            "p": lambda: self.set_live(False),
            "r": lambda: self.run_vendor_command(RESUME_HANDS),
            "0": lambda: self.run_vendor_command(CLEAR_ZERO_DRIFT),
            "o": lambda: self.run_vendor_command(RESUME_BODY),
            "z": lambda: self.run_vendor_command(ZERO_POSITION),
        }
        if key in ("h", "?"):
            self.print_key_help()
            return True
        action = actions.get(key)
        if action is None:
            return True
        success, message = action()
        (self.logger.info if success else self.logger.warning)(message)
        return True

    def print_key_help(self) -> None:
        print(
            "[pnlink-source] keys: N start capture | F stop capture | "
            "C vendor calibration | T capture/recalibrate SONIC T-pose (C optional) | "
            "L live | P pause | R resume hands | 0 clear drift | "
            "O resume body | Z zero position | H help | ESC exit",
            flush=True,
        )

    def run(self) -> None:
        self.print_key_help()
        if not sys.stdin.isatty():
            self.logger.warning(
                "stdin is not a TTY; run this source in a foreground terminal "
                "for single-key control"
            )
        self._publish_status(force=True)
        period = 1.0 / SOURCE_FRAME_RATE_HZ
        next_tick = time.monotonic()
        with TerminalKeyInput() as keyboard:
            while not self.stop_requested:
                key = keyboard.read()
                if key is not None and not self.handle_key(key):
                    break
                self._tick()
                self._publish_status()
                next_tick += period
                now = time.monotonic()
                if next_tick < now - period:
                    next_tick = now + period
                time.sleep(max(0.0, next_tick - now))

    def close(self) -> None:
        try:
            self.sdk.close()
        finally:
            if self.debug_publisher is not None:
                self.debug_publisher.close()
            if self.bundle_recorder is not None:
                self.bundle_recorder.close()
            self.pose_socket.close(linger=0)
            self.zmq_context.term()


def _env_flag(name: str, default: bool) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


def _calibration_guidance(
    state: SourceState, neutral_samples_collected: int
) -> tuple[str, float | None]:
    """Return operator guidance without claiming a duration for vendor calibration."""
    if state == SourceState.DISCONNECTED:
        return "等待 PN-Link 动作数据，请确认设备连接和网络配置。", None
    if state == SourceState.CAPTURING:
        return "已收到动作数据；可执行厂商标定，或保持标准 T 姿态直接开始中立标定。", None
    if state == SourceState.VENDOR_CALIBRATING:
        return "正在进行厂商标定，请保持设备要求的标定姿态并等待完成。", None
    if state == SourceState.NEEDS_NEUTRAL:
        if neutral_samples_collected > 0:
            remaining = max(0, NEUTRAL_SAMPLE_COUNT - neutral_samples_collected)
            return (
                "正在采集 T 姿态，请保持不动。",
                round(remaining / SOURCE_FRAME_RATE_HZ, 2),
            )
        return "请保持标准 T 姿态，然后开始中立姿态采集。", None
    if state == SourceState.READY_PAUSED:
        return "SMPL 标定完成，可查看原始与 SMPL 骨架；当前未接入 SONIC。", None
    return "实时输出已开启。", None


def main(args: list[str] | None = None) -> int:
    del args
    source: SonicPnLinkPoseSource | None = None
    try:
        source = SonicPnLinkPoseSource()
        source.run()
    except KeyboardInterrupt:
        pass
    finally:
        if source is not None:
            source.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
