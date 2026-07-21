"""Runtime-only adapter around the vendor-provided ``mocap_api`` module."""

from __future__ import annotations

from dataclasses import dataclass
import importlib
from pathlib import Path
import platform
import sys
import time
from typing import Any, Callable, Mapping

import numpy as np

from .skeleton import JointLocalPose


@dataclass(frozen=True)
class SdkConfig:
    sdk_path: Path
    local_ip: str
    local_port: int
    server_ip: str
    server_port: int


@dataclass(frozen=True)
class RawAvatarFrame:
    timestamp_monotonic_ns: int
    timestamp_realtime: float
    joints: dict[str, JointLocalPose]


@dataclass(frozen=True)
class SdkCommandResult:
    command: str
    success: bool
    message: str


START_CAPTURE = "start_capture"
STOP_CAPTURE = "stop_capture"
CALIBRATE_MOTION = "calibrate_motion"
RESUME_HANDS = "resume_hands"
CLEAR_ZERO_DRIFT = "clear_zero_drift"
RESUME_BODY = "resume_body"
ZERO_POSITION = "zero_position"


_NATIVE_COMMAND_NAMES = {
    START_CAPTURE: "CommandStartCapture",
    STOP_CAPTURE: "CommandStopCapture",
    CALIBRATE_MOTION: "CommandCalibrateMotion",
    RESUME_HANDS: "CommandResumeOriginalHandsPosture",
    CLEAR_ZERO_DRIFT: "CommandClearZeroMotionDrift",
    RESUME_BODY: "CommandResumeOriginalPosture",
    ZERO_POSITION: "CommandZeroPosition",
}

_CLIENT_COMMAND_NAMES = {
    START_CAPTURE: ("start_capture", "StartCapture"),
    STOP_CAPTURE: ("stop_capture", "StopCapture"),
    CALIBRATE_MOTION: ("command_calibrate_motion", "CommandCalibrateMotion"),
    RESUME_HANDS: (
        "resume_original_hands_posture",
        "ResumeOriginalHandsPosture",
    ),
    CLEAR_ZERO_DRIFT: ("clear_zero_motion_drift", "ClearZeroMotionDrift"),
    RESUME_BODY: ("resume_original_posture", "ResumeOriginalPosture"),
    ZERO_POSITION: ("zero_position", "ZeroPosition"),
}


def validate_sdk_installation(path: Path) -> None:
    if not path.is_dir():
        raise FileNotFoundError(f"PN-Link SDK directory does not exist: {path}")
    if not (path / "mocap_api.py").is_file():
        raise FileNotFoundError(f"PN-Link Python binding is missing: {path / 'mocap_api.py'}")
    native = (
        list(path.glob("librobotapi_*.so"))
        + list(path.glob("librobotapi*.so"))
        + list((path / "lib").glob("librobotapi_*.so"))
        + list((path / "lib").glob("librobotapi*.so"))
    )
    if not native:
        raise FileNotFoundError(f"PN-Link native library is missing under {path}")
    machine = platform.machine().lower()
    architecture_aliases = {
        "x86_64": ("x86_64", "x86-64", "amd64", "x64"),
        "aarch64": ("aarch64", "arm64"),
    }.get(machine, (machine,))

    def matches_architecture(item: Path) -> bool:
        aliases = (alias in item.name.lower() for alias in architecture_aliases)
        return item.name == "librobotapi.so" or any(aliases)

    matching = [item for item in native if matches_architecture(item)]
    if not matching and machine in ("x86_64", "aarch64"):
        names = ", ".join(item.name for item in native)
        raise RuntimeError(
            f"PN-Link native libraries do not match architecture {machine}: {names}"
        )


def _value(obj: Any, names: tuple[str, ...]) -> Any:
    for name in names:
        if isinstance(obj, Mapping) and name in obj:
            return obj[name]
        if hasattr(obj, name):
            value = getattr(obj, name)
            return value() if callable(value) else value
    raise AttributeError(f"{type(obj).__name__} has none of {names}")


def _xyz(value: Any) -> np.ndarray:
    if isinstance(value, Mapping):
        return np.asarray([value[k] for k in ("x", "y", "z")], dtype=np.float64)
    if all(hasattr(value, key) for key in ("x", "y", "z")):
        return np.asarray([value.x, value.y, value.z], dtype=np.float64)
    return np.asarray(value, dtype=np.float64).reshape(3)


def _wxyz(value: Any) -> np.ndarray:
    if isinstance(value, Mapping):
        return np.asarray([value[k] for k in ("w", "x", "y", "z")], dtype=np.float64)
    if all(hasattr(value, key) for key in ("w", "x", "y", "z")):
        return np.asarray([value.w, value.x, value.y, value.z], dtype=np.float64)
    return np.asarray(value, dtype=np.float64).reshape(4)


def avatar_to_frame(avatar: Any) -> RawAvatarFrame:
    joints_value = _value(avatar, ("get_joints", "GetJoints", "joints"))
    if isinstance(joints_value, Mapping):
        iterable = joints_value.items()
    else:
        iterable = ((None, joint) for joint in joints_value)
    joints: dict[str, JointLocalPose] = {}
    for provided_name, joint in iterable:
        name = str(provided_name) if provided_name is not None else str(
            _value(joint, ("get_name", "GetName", "name"))
        )
        position = _value(
            joint,
            ("get_local_position", "GetLocalPosition", "local_position", "position"),
        )
        rotation = _value(
            joint,
            ("get_local_rotation", "GetLocalRotation", "local_rotation", "rotation"),
        )
        joints[name] = JointLocalPose(_xyz(position), _wxyz(rotation))
    return RawAvatarFrame(time.monotonic_ns(), time.time(), joints)


class PnLinkSdkAdapter:
    """Normalize SDK lifecycle and callback naming into one small interface."""

    def __init__(
        self,
        config: SdkConfig,
        callback: Callable[[RawAvatarFrame], None],
        command_callback: Callable[[SdkCommandResult], None] | None = None,
        progress_callback: Callable[[str], None] | None = None,
    ) -> None:
        validate_sdk_installation(config.sdk_path)
        sys.path.insert(0, str(config.sdk_path))
        try:
            self.module = importlib.import_module("mocap_api")
        except Exception as exc:
            raise RuntimeError(
                f"cannot import PN-Link mocap_api from {config.sdk_path}: {exc}"
            ) from exc
        self.callback = callback
        self.command_callback = command_callback
        self.progress_callback = progress_callback
        self.settings = None
        self._native_mcp = False
        self._pending_command: str | None = None
        self.client = self._create_client(config)
        if not self._native_mcp:
            self._register_callback()

    def _create_client(self, config: SdkConfig) -> Any:
        application_cls = getattr(self.module, "MCPApplication", None)
        settings_cls = getattr(self.module, "MCPSettings", None)
        if application_cls is not None and settings_cls is not None:
            self._native_mcp = True
            self.settings = settings_cls()
            self.settings.set_bvh_data(self.module.MCPBvhData.Binary)
            self.settings.set_bvh_transformation(
                self.module.MCPBvhDisplacement.Enable
            )
            self.settings.set_bvh_rotation(self.module.MCPBvhRotation.YXZ)
            self.settings.SetSettingsUDPEx(config.local_ip, config.local_port)
            self.settings.SetSettingsUDPServer(config.server_ip, config.server_port)
            client = application_cls()
            client.set_settings(self.settings)
            return client

        factory = getattr(self.module, "create_sonic_pnlink_client", None)
        kwargs = {
            "local_ip": config.local_ip,
            "local_port": config.local_port,
            "server_ip": config.server_ip,
            "server_port": config.server_port,
        }
        if callable(factory):
            return factory(**kwargs)
        for class_name in ("MocapApi", "MocapAPI", "MocapClient"):
            cls = getattr(self.module, class_name, None)
            if cls is None:
                continue
            endpoint_args = (
                config.local_ip,
                config.local_port,
                config.server_ip,
                config.server_port,
            )
            for args in ((), endpoint_args):
                try:
                    client = cls(*args)
                    configure = getattr(client, "configure", None)
                    if callable(configure):
                        configure(**kwargs)
                    return client
                except TypeError:
                    continue
        raise RuntimeError(
            "unsupported mocap_api: provide MocapApi/MocapAPI/MocapClient or "
            "create_sonic_pnlink_client(local_ip, local_port, server_ip, server_port)"
        )

    def _register_callback(self) -> None:
        def on_avatar(*args: Any) -> None:
            if not args:
                raise RuntimeError("AvatarUpdated callback did not provide an avatar")
            payload = args[-1]
            try:
                frame = avatar_to_frame(payload)
            except AttributeError:
                avatar = _value(payload, ("get_avatar", "GetAvatar", "avatar"))
                frame = avatar_to_frame(avatar)
            self.callback(frame)

        for name in (
            "set_avatar_updated_callback", "add_avatar_updated_callback",
            "SetAvatarUpdatedCallback", "AddAvatarUpdatedCallback",
        ):
            method = getattr(self.client, name, None)
            if callable(method):
                method(on_avatar)
                return
        event = getattr(self.client, "AvatarUpdated", None)
        if event is not None:
            try:
                event += on_avatar
                return
            except TypeError:
                pass
        raise RuntimeError("mocap_api client does not expose an AvatarUpdated callback")

    def start(self) -> None:
        if self._native_mcp:
            opened, message = self.client.open()
            if not opened:
                raise RuntimeError(f"PN-Link SDK open failed: {message}")
            return
        for name in ("start", "open", "connect", "Start", "Open", "Connect"):
            method = getattr(self.client, name, None)
            if callable(method):
                result = method()
                if result is False:
                    raise RuntimeError(f"PN-Link SDK {name} returned failure")
                return
        raise RuntimeError("mocap_api client does not expose start/open/connect")

    def poll(self) -> None:
        if self._native_mcp:
            for event in self.client.poll_next_event():
                if event.event_type == self.module.MCPEventType.AvatarUpdated:
                    avatar = self.module.MCPAvatar(event.event_data.avatar_handle)
                    self.callback(avatar_to_frame(avatar))
                elif event.event_type == self.module.MCPEventType.CommandReply:
                    self._handle_command_reply(event.event_data.commandRespond)
            return
        for name in ("poll", "update", "Poll", "Update"):
            method = getattr(self.client, name, None)
            if callable(method):
                method()
                return

    def _emit_command_result(self, command: str, success: bool, message: str) -> None:
        if self.command_callback is not None:
            self.command_callback(SdkCommandResult(command, success, message))

    def _queue_native_command(self, command: str, command_type: int) -> None:
        if self._pending_command is not None:
            raise RuntimeError(
                f"PN-Link command already pending: {self._pending_command}"
            )
        self.client.queue_command(command_type)
        self._pending_command = command

    @property
    def pending_command(self) -> str | None:
        return self._pending_command

    def _run_command(self, command: str) -> None:
        if self._pending_command is not None:
            raise RuntimeError(
                f"PN-Link command already pending: {self._pending_command}"
            )
        if command == RESUME_HANDS and sys.platform != "win32":
            raise RuntimeError(
                "Resume Hands is not supported by the bundled Linux PN-Link SDK"
            )
        if self._native_mcp:
            enum_name = _NATIVE_COMMAND_NAMES[command]
            command_type = getattr(self.module.EMCPCommand, enum_name, None)
            if command_type is None:
                raise RuntimeError(f"PN-Link SDK does not expose {enum_name}")
            self._queue_native_command(command, command_type)
            return

        for owner in (self.client, self.module):
            for name in _CLIENT_COMMAND_NAMES[command]:
                method = getattr(owner, name, None)
                if not callable(method):
                    continue
                result = method()
                if result is False:
                    raise RuntimeError(f"PN-Link {command} returned failure")
                self._emit_command_result(command, True, "completed")
                return
        if command == START_CAPTURE:
            # Some callback-style clients begin capture during open/connect.
            self._emit_command_result(command, True, "capture managed by SDK")
            return
        raise RuntimeError(f"mocap_api does not expose {command}")

    def _handle_command_reply(self, response: Any) -> None:
        running_reply = getattr(self.module.MCPReplay, "MCPReplay_Running", None)
        if running_reply is not None and response._replay == running_reply:
            self._handle_calibration_progress(response)
            return
        if response._replay != self.module.MCPReplay.MCPReplay_Result:
            return
        command = self._pending_command or "unknown"
        result = self.module.MCPCommand()
        try:
            code = int(result.get_result_code(response._commandHandle))
            message = (
                "completed"
                if code == 0
                else str(result.get_result_message(response._commandHandle))
            )
        except Exception as exc:
            code = -1
            message = f"cannot read PN-Link command result: {exc}"
        destroy_error = None
        try:
            result.destroy_command(response._commandHandle)
        except Exception as exc:
            destroy_error = str(exc)
        finally:
            self._pending_command = None
        if destroy_error is not None:
            message = f"{message}; command cleanup failed: {destroy_error}"
        self._emit_command_result(command, code == 0, message)

    def _handle_calibration_progress(self, response: Any) -> None:
        if self._pending_command != CALIBRATE_MOTION or self.progress_callback is None:
            return
        try:
            progress_handle = self.module.MCPCommand().get_progress(
                response._commandHandle
            )
            progress = self.module.MCPCalibrateMotionProgress(progress_handle)
            step, pose_name = progress.get_step_current_pose()
            step_type = self.module.MCPCalibrateMotionProgressStep
            if step == step_type.CalibrateMotionProgressStep_Countdown:
                countdown, pose_name = progress.get_countdown_current_pose()
                message = f"calibration {pose_name}: countdown {countdown}"
            elif step == step_type.CalibrateMotionProgressStep_Progress:
                percent, pose_name = progress.get_progress_current_pose()
                message = f"calibration {pose_name}: progress {percent}%"
            elif step == step_type.CalibrateMotionProgressStep_Prepare:
                message = f"calibration {pose_name}: preparing"
            else:
                message = f"calibration {pose_name}: step {step}"
        except Exception as exc:
            message = f"cannot read calibration progress: {exc}"
        self.progress_callback(message)

    def start_capture(self) -> None:
        self._run_command(START_CAPTURE)

    def stop_capture(self) -> None:
        self._run_command(STOP_CAPTURE)

    def calibrate_motion(self) -> None:
        self._run_command(CALIBRATE_MOTION)

    def resume_hands(self) -> None:
        self._run_command(RESUME_HANDS)

    def clear_zero_drift(self) -> None:
        self._run_command(CLEAR_ZERO_DRIFT)

    def resume_body(self) -> None:
        self._run_command(RESUME_BODY)

    def zero_position(self) -> None:
        self._run_command(ZERO_POSITION)

    def close(self) -> None:
        if self._native_mcp:
            self.client.close()
            return
        for name in ("close", "disconnect", "stop", "Close", "Disconnect", "Stop"):
            method = getattr(self.client, name, None)
            if callable(method):
                method()
                return
