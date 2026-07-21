"""MuJoCo viewer for raw PN-Link and retargeted SMPL-24 skeletons."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import os
from pathlib import Path
import threading
import time
from typing import Any

import numpy as np
try:
    import zmq
except ImportError:  # pragma: no cover - pure viewer tests need no socket
    zmq = None

from .debug_wire import decode_debug_frame, sanitize_debug_frame
from .diagnostics import (
    DiagnosticBundleRecorder,
    DiagnosticIssue,
    PIPELINE_STAGES,
)
from .skeleton import (
    PNLINK_JOINT_NAMES,
    PNLINK_PARENTS,
    SMPL_JOINT_NAMES,
    SMPL_PARENTS,
    SMPL_TO_PNLINK,
    rotation_from_wxyz,
)

RAW_COLOR = np.array([0.0, 0.8, 0.9, 1.0], dtype=np.float32)
SMPL_COLOR = np.array([0.15, 0.85, 0.25, 1.0], dtype=np.float32)
WARNING_COLOR = np.array([1.0, 0.85, 0.0, 1.0], dtype=np.float32)
POSITION_ERROR_COLOR = np.array([1.0, 0.05, 0.05, 1.0], dtype=np.float32)
ROTATION_ERROR_COLOR = np.array([1.0, 0.35, 0.0, 1.0], dtype=np.float32)
BOTH_ERROR_COLOR = np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32)
AFFECTED_COLOR = np.array([0.25, 0.05, 0.05, 0.7], dtype=np.float32)
FROZEN_COLOR = np.array([0.5, 0.5, 0.5, 0.5], dtype=np.float32)
MAPPING_COLOR = np.array([1.0, 0.9, 0.0, 0.7], dtype=np.float32)
MAGENTA_COLOR = np.array([1.0, 0.0, 1.0, 0.9], dtype=np.float32)


def build_mujoco_xml() -> str:
    return """
<mujoco model="sonic_pnlink_diagnostics">
  <option gravity="0 0 -9.81" timestep="0.02"/>
  <visual>
    <global offwidth="1280" offheight="720"/>
    <rgba haze="0.05 0.05 0.05 1"/>
  </visual>
  <worldbody>
    <light pos="0 0 4" dir="0 0 -1" diffuse="0.8 0.8 0.8"/>
    <geom name="floor" type="plane" size="4 4 0.1"
          rgba="0.08 0.09 0.1 1" contype="0" conaffinity="0"/>
  </worldbody>
</mujoco>
""".strip()


def layout_skeletons(
    raw_positions: np.ndarray,
    smpl_positions: np.ndarray,
    separation: float = 0.8,
) -> tuple[np.ndarray, np.ndarray]:
    raw = np.asarray(raw_positions, dtype=np.float32).reshape(-1, 3).copy()
    smpl = np.asarray(smpl_positions, dtype=np.float32).reshape(24, 3).copy()
    raw_root = raw[PNLINK_JOINT_NAMES.index("Hips")].copy()
    smpl_root = smpl[0].copy()
    raw[:, :2] -= raw_root[:2]
    smpl[:, :2] -= smpl_root[:2]
    raw[:, 1] += separation
    smpl[:, 1] -= separation
    return raw, smpl


def raw_display_states(fields: dict[str, np.ndarray]) -> list[str]:
    present = np.asarray(fields["raw_present"], dtype=bool)
    position_valid = np.asarray(fields["raw_position_valid"], dtype=bool)
    rotation_valid = np.asarray(fields["raw_rotation_valid"], dtype=bool)
    states: list[str] = []
    invalid_ancestor: dict[str, bool] = {}
    for index, name in enumerate(PNLINK_JOINT_NAMES):
        parent = PNLINK_PARENTS[name]
        affected = bool(parent and invalid_ancestor[parent])
        local_invalid = (
            not present[index]
            or not position_valid[index]
            or not rotation_valid[index]
        )
        invalid_ancestor[name] = local_invalid or affected
        if affected and not local_invalid:
            states.append("affected")
        elif not present[index]:
            states.append("missing")
        elif not position_valid[index] and not rotation_valid[index]:
            states.append("both_error")
        elif not position_valid[index]:
            states.append("position_error")
        elif not rotation_valid[index]:
            states.append("rotation_error")
        else:
            states.append("valid")
    return states


def raw_affected_causes(fields: dict[str, np.ndarray]) -> list[str]:
    present = np.asarray(fields["raw_present"], dtype=bool)
    position_valid = np.asarray(fields["raw_position_valid"], dtype=bool)
    rotation_valid = np.asarray(fields["raw_rotation_valid"], dtype=bool)
    causes: list[str] = []
    inherited: dict[str, str] = {}
    for index, name in enumerate(PNLINK_JOINT_NAMES):
        parent = PNLINK_PARENTS[name]
        cause = inherited.get(parent or "", "")
        if not present[index]:
            cause = f"SDK_RAW/{name}/state"
        elif not position_valid[index]:
            cause = f"SDK_RAW/{name}/position"
        elif not rotation_valid[index]:
            cause = f"SDK_RAW/{name}/rotation"
        inherited[name] = cause
        causes.append(cause)
    return causes


def _color_for_raw_state(state: str) -> np.ndarray:
    return {
        "valid": RAW_COLOR,
        "missing": FROZEN_COLOR,
        "position_error": POSITION_ERROR_COLOR,
        "rotation_error": ROTATION_ERROR_COLOR,
        "both_error": BOTH_ERROR_COLOR,
        "affected": AFFECTED_COLOR,
    }[state]


@dataclass
class DiagnosticInbox:
    issues: deque[dict[str, str]] = field(default_factory=lambda: deque(maxlen=10))
    stages: dict[str, str] = field(
        default_factory=lambda: {stage: "UNKNOWN" for stage in PIPELINE_STAGES}
    )
    source_status: dict[str, Any] = field(default_factory=dict)

    def start(self) -> None:
        pass

    def spin_once(self) -> None:
        pass

    def close(self) -> None:
        pass


@dataclass
class ViewerState:
    history: deque[dict[str, np.ndarray]] = field(
        default_factory=lambda: deque(maxlen=100)
    )
    paused: bool = False
    history_offset: int = 0
    selected_smpl_joint: int = 0
    request_dump: bool = False
    request_focus: bool = False
    request_latest_error: bool = False
    click_position: tuple[float, float, int, int] | None = None
    lock: threading.Lock = field(default_factory=threading.Lock)

    def key_callback(self, keycode: int) -> None:
        with self.lock:
            if keycode == 32:  # Space
                self.paused = not self.paused
            elif keycode == 263:  # Left
                self.paused = True
                self.history_offset = min(
                    self.history_offset + 1, max(0, len(self.history) - 1)
                )
            elif keycode == 262:  # Right
                self.history_offset = max(0, self.history_offset - 1)
            elif keycode == 91:  # [
                self.selected_smpl_joint = (self.selected_smpl_joint - 1) % 24
            elif keycode == 93:  # ]
                self.selected_smpl_joint = (self.selected_smpl_joint + 1) % 24
            elif keycode in (68, 100):  # D
                self.request_dump = True
            elif keycode in (70, 102):  # F
                self.request_focus = True
            elif keycode in (69, 101):  # E
                self.request_latest_error = True

    def add(self, frame: dict[str, np.ndarray]) -> None:
        with self.lock:
            self.history.append(frame)
            if not self.paused:
                self.history_offset = 0

    def current(self) -> dict[str, np.ndarray] | None:
        with self.lock:
            if not self.history:
                return None
            index = max(0, len(self.history) - 1 - self.history_offset)
            return self.history[index]


def install_mouse_selection(viewer: Any, state: ViewerState) -> None:
    try:
        import glfw

        window = viewer._window
        previous = glfw.set_mouse_button_callback(window, None)

        def callback(window_handle, button, action, modifiers):
            if callable(previous):
                previous(window_handle, button, action, modifiers)
            if button == glfw.MOUSE_BUTTON_LEFT and action == glfw.PRESS:
                x, y = glfw.get_cursor_pos(window_handle)
                width, height = glfw.get_window_size(window_handle)
                with state.lock:
                    state.click_position = (x, y, width, height)

        glfw.set_mouse_button_callback(window, callback)
    except Exception:
        pass


def select_nearest_joint(
    viewer: Any,
    model: Any,
    raw: np.ndarray,
    smpl: np.ndarray,
    click: tuple[float, float, int, int],
) -> int:
    x, y, width, height = click
    if width <= 0 or height <= 0:
        return 0
    azimuth = np.deg2rad(float(viewer.cam.azimuth))
    elevation = np.deg2rad(float(viewer.cam.elevation))
    forward = np.array(
        [
            np.cos(elevation) * np.cos(azimuth),
            np.cos(elevation) * np.sin(azimuth),
            np.sin(elevation),
        ]
    )
    right = np.cross(forward, np.array([0.0, 0.0, 1.0]))
    right_norm = np.linalg.norm(right)
    if right_norm < 1.0e-8:
        return 0
    right /= right_norm
    up = np.cross(right, forward)
    camera_position = np.asarray(viewer.cam.lookat) - viewer.cam.distance * forward
    points = np.concatenate((raw, smpl), axis=0)
    relative = points - camera_position
    depth = relative @ forward
    fovy = np.deg2rad(float(model.vis.global_.fovy))
    scale_y = np.maximum(depth * np.tan(fovy / 2.0), 1.0e-6)
    scale_x = scale_y * (width / height)
    screen_x = width * (0.5 + 0.5 * (relative @ right) / scale_x)
    screen_y = height * (0.5 - 0.5 * (relative @ up) / scale_y)
    distance = np.hypot(screen_x - x, screen_y - y)
    distance[depth <= 0.0] = np.inf
    nearest = int(np.argmin(distance))
    if nearest >= len(raw):
        return nearest - len(raw)
    source_name = PNLINK_JOINT_NAMES[nearest]
    matches = [
        index for index, mapped in enumerate(SMPL_TO_PNLINK)
        if mapped == source_name
    ]
    return matches[0] if matches else 0


def _add_sphere(mujoco: Any, scene: Any, position: np.ndarray, color: np.ndarray) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_SPHERE,
        np.array([0.025, 0.0, 0.0]),
        np.asarray(position, dtype=np.float64),
        np.eye(3).reshape(-1),
        np.asarray(color, dtype=np.float32),
    )
    scene.ngeom += 1


def _add_bone(
    mujoco: Any,
    scene: Any,
    start: np.ndarray,
    end: np.ndarray,
    color: np.ndarray,
    radius: float = 0.009,
) -> None:
    if scene.ngeom >= scene.maxgeom:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        np.zeros(3),
        np.zeros(3),
        np.eye(3).reshape(-1),
        np.asarray(color, dtype=np.float32),
    )
    mujoco.mjv_connector(
        geom,
        mujoco.mjtGeom.mjGEOM_CAPSULE,
        radius,
        np.asarray(start, dtype=np.float64),
        np.asarray(end, dtype=np.float64),
    )
    scene.ngeom += 1


def _add_axes(
    mujoco: Any,
    scene: Any,
    position: np.ndarray,
    quaternion_wxyz: np.ndarray,
    *,
    error: bool = False,
) -> None:
    try:
        matrix = rotation_from_wxyz(quaternion_wxyz).as_matrix()
    except (TypeError, ValueError):
        return
    colors = (
        (MAGENTA_COLOR, MAGENTA_COLOR, MAGENTA_COLOR)
        if error
        else (POSITION_ERROR_COLOR, SMPL_COLOR, RAW_COLOR)
    )
    for axis in range(3):
        endpoint = position + matrix[:, axis] * 0.12
        _add_bone(
            mujoco, scene, position, endpoint, colors[axis], radius=0.0025
        )


def render_debug_frame(
    mujoco: Any,
    scene: Any,
    fields: dict[str, np.ndarray],
    selected_smpl_joint: int,
) -> tuple[np.ndarray, np.ndarray]:
    scene.ngeom = 0
    raw, smpl = layout_skeletons(fields["raw_world_pos"], fields["smpl_joints"])
    raw_states = raw_display_states(fields)
    for index, name in enumerate(PNLINK_JOINT_NAMES):
        color = _color_for_raw_state(raw_states[index])
        _add_sphere(mujoco, scene, raw[index], color)
        if raw_states[index] in ("rotation_error", "both_error"):
            _add_axes(
                mujoco,
                scene,
                raw[index],
                fields["raw_world_quat"][index],
                error=True,
            )
        parent = PNLINK_PARENTS[name]
        if parent is not None:
            parent_index = PNLINK_JOINT_NAMES.index(parent)
            bone_color = color
            if name in ("LeftTiptoe", "RightTiptoe") and not fields["raw_present"][index]:
                bone_color = FROZEN_COLOR
            _add_bone(mujoco, scene, raw[parent_index], raw[index], bone_color)

    stage_valid = np.asarray(fields["source_stage_valid"], dtype=bool).reshape(-1)
    smpl_visible = stage_valid.size > 2 and bool(stage_valid[2])
    if smpl_visible:
        smpl_valid = np.asarray(fields["smpl_position_valid"], dtype=bool)
        for index, parent in enumerate(SMPL_PARENTS):
            color = SMPL_COLOR if smpl_valid[index] else POSITION_ERROR_COLOR
            _add_sphere(mujoco, scene, smpl[index], color)
            if parent >= 0:
                bone_color = color if smpl_valid[parent] else AFFECTED_COLOR
                _add_bone(mujoco, scene, smpl[parent], smpl[index], bone_color)

        source_name = SMPL_TO_PNLINK[selected_smpl_joint]
        if source_name in PNLINK_JOINT_NAMES:
            source_index = PNLINK_JOINT_NAMES.index(source_name)
            _add_bone(
                mujoco,
                scene,
                raw[source_index],
                smpl[selected_smpl_joint],
                MAPPING_COLOR,
                radius=0.003,
            )
            _add_axes(
                mujoco,
                scene,
                raw[source_index],
                fields["raw_world_quat"][source_index],
                error=not bool(fields["raw_rotation_valid"][source_index]),
            )
        _add_axes(
            mujoco,
            scene,
            smpl[selected_smpl_joint],
            fields["retarget_global_quat"][selected_smpl_joint],
            error=not bool(fields["smpl_rotation_valid"][selected_smpl_joint]),
        )
    _add_bone(mujoco, scene, np.zeros(3), np.array([0.25, 0.0, 0.0]), POSITION_ERROR_COLOR, 0.004)
    _add_bone(mujoco, scene, np.zeros(3), np.array([0.0, 0.25, 0.0]), SMPL_COLOR, 0.004)
    _add_bone(mujoco, scene, np.zeros(3), np.array([0.0, 0.0, 0.25]), RAW_COLOR, 0.004)
    return raw, smpl


def _overlay(
    viewer: Any,
    inbox: DiagnosticInbox,
    state: ViewerState,
    frame: dict[str, np.ndarray],
) -> None:
    overlay = getattr(viewer, "_overlay", None)
    if overlay is None:
        return
    try:
        import mujoco

        location = mujoco.mjtGridPos.mjGRID_TOPLEFT
        rows = overlay.setdefault(location, [])
        rows.clear()
        frame_index = int(np.asarray(frame["frame_index"]).reshape(-1)[0])
        status = inbox.source_status
        rows.append(("PN-Link / SONIC", f"frame {frame_index}"))
        hips = frame["raw_world_pos"][PNLINK_JOINT_NAMES.index("Hips")]
        rows.append(("Hips world", np.array2string(hips, precision=3)))
        rows.append(
            (
                "source",
                f"{status.get('state', 'unknown')} "
                f"hz={status.get('source_hz', '?')} live={status.get('live_enabled', '?')}",
            )
        )
        source_name = SMPL_TO_PNLINK[state.selected_smpl_joint]
        if source_name in PNLINK_JOINT_NAMES:
            source_index = PNLINK_JOINT_NAMES.index(source_name)
            cause = raw_affected_causes(frame)[source_index]
            if cause:
                rows.append(("caused_by", cause))
        rows.append(
            (
                "selected",
                f"SMPL[{state.selected_smpl_joint}] "
                f"{SMPL_JOINT_NAMES[state.selected_smpl_joint]} <- "
                f"{SMPL_TO_PNLINK[state.selected_smpl_joint] or 'SMPL FK'}",
            )
        )
        rows.append(("source stages", " | ".join(
            f"{name}:{inbox.stages[name]}" for name in PIPELINE_STAGES[:5]
        )))
        rows.append(("pipeline", " | ".join(
            f"{name}:{inbox.stages[name]}" for name in PIPELINE_STAGES[5:]
        )))
        for issue in list(inbox.issues)[:5]:
            rows.append(
                (
                    issue.get("stage", "issue"),
                    f"{issue.get('code')} {issue.get('source_joint')} "
                    f"{issue.get('field')} {issue.get('action')}",
                )
            )
    except Exception:
        pass


def main() -> int:
    if zmq is None:
        raise RuntimeError("sonic_pnlink_mujoco_viewer requires pyzmq")
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        raise RuntimeError("MuJoCo viewer requires DISPLAY or WAYLAND_DISPLAY")
    try:
        import mujoco
        import mujoco.viewer
    except ImportError as exc:
        raise RuntimeError("sonic_pnlink_mujoco_viewer requires mujoco") from exc

    host = os.environ.get("SONIC_PNLINK_DEBUG_HOST", "127.0.0.1")
    port = int(os.environ.get("SONIC_PNLINK_DEBUG_PORT", "5558"))
    render_hz = max(
        1.0, float(os.environ.get("SONIC_PNLINK_VIEWER_RENDER_HZ", "30"))
    )
    context = zmq.Context()
    socket = context.socket(zmq.SUB)
    socket.setsockopt(zmq.RCVHWM, 1)
    socket.setsockopt(zmq.LINGER, 0)
    socket.setsockopt_string(zmq.SUBSCRIBE, "pnlink_debug")
    socket.connect(f"tcp://{host}:{port}")
    poller = zmq.Poller()
    poller.register(socket, zmq.POLLIN)

    inbox = DiagnosticInbox()
    inbox.start()
    state = ViewerState()
    last_valid: dict[str, np.ndarray] = {}
    recorder = DiagnosticBundleRecorder(
        Path(os.environ.get("SONIC_PNLINK_DIAG_DIR", "/tmp/sonic_pnlink_diagnostics")),
        post_frames=0,
        session_name="viewer_manual",
    )
    model = mujoco.MjModel.from_xml_string(build_mujoco_xml())
    data = mujoco.MjData(model)
    try:
        with mujoco.viewer.launch_passive(
            model,
            data,
            key_callback=state.key_callback,
            show_left_ui=False,
            show_right_ui=False,
        ) as viewer:
            install_mouse_selection(viewer, state)
            viewer.cam.lookat[:] = [0.0, 0.0, 0.9]
            viewer.cam.distance = 3.2
            viewer.cam.azimuth = 135.0
            viewer.cam.elevation = -15.0
            period = 1.0 / render_hz
            while viewer.is_running():
                started = time.monotonic()
                while dict(poller.poll(timeout=0)).get(socket) == zmq.POLLIN:
                    try:
                        message = socket.recv(flags=zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    try:
                        decoded = decode_debug_frame(message)
                        safe, last_valid = sanitize_debug_frame(decoded, last_valid)
                    except Exception as exc:
                        print(f"[pnlink-viewer] invalid debug frame: {exc}", flush=True)
                        continue
                    state.add(safe)
                    recorder.add_frame(decoded)
                inbox.spin_once()
                frame = state.current()
                if state.request_latest_error and inbox.issues:
                    try:
                        error_frame = int(inbox.issues[0].get("frame_index", -1))
                        history_frames = [
                            int(item["frame_index"][0]) for item in state.history
                        ]
                        nearest = min(
                            range(len(history_frames)),
                            key=lambda index: abs(history_frames[index] - error_frame),
                        )
                        state.paused = True
                        state.history_offset = len(history_frames) - 1 - nearest
                    except (TypeError, ValueError):
                        pass
                    state.request_latest_error = False
                if frame is not None:
                    for stage, valid in zip(
                        PIPELINE_STAGES[:7], frame["source_stage_valid"]
                    ):
                        inbox.stages[stage] = "OK" if bool(valid) else "ERROR"
                    with viewer.lock():
                        raw, smpl = render_debug_frame(
                            mujoco,
                            viewer.user_scn,
                            frame,
                            state.selected_smpl_joint,
                        )
                        with state.lock:
                            click = state.click_position
                            state.click_position = None
                        if click is not None:
                            state.selected_smpl_joint = select_nearest_joint(
                                viewer, model, raw, smpl, click
                            )
                        if state.request_focus:
                            viewer.cam.lookat[:] = smpl[state.selected_smpl_joint]
                            state.request_focus = False
                        _overlay(viewer, inbox, state, frame)
                    if state.request_dump:
                        recorder.dump_now(
                            DiagnosticIssue(
                                "SDK_RAW", "MANUAL_DUMP", "state", "RECORDED",
                                frame_index=int(frame["frame_index"][0]),
                                severity="WARN",
                            ),
                            manifest={"requested_by": "mujoco_viewer"},
                        )
                        state.request_dump = False
                viewer.sync()
                remaining = period - (time.monotonic() - started)
                if remaining > 0.0:
                    time.sleep(remaining)
    finally:
        recorder.close()
        inbox.close()
        poller.unregister(socket)
        socket.close(linger=0)
        context.term()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
