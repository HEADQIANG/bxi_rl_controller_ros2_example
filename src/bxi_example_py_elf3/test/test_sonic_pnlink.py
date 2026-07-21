import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
import threading
from types import SimpleNamespace

from bxi_example_py_elf3.sonic_pnlink.calibration import (
    calibrate_neutral,
    markley_mean_wxyz,
    median_bone_lengths_cm,
)
from bxi_example_py_elf3.sonic_pnlink.diagnostics import (
    DiagnosticBundleRecorder,
    DiagnosticIssue,
    DiagnosticReporter,
    rotation_matrix_is_valid,
)
from bxi_example_py_elf3.sonic_pnlink.debug_wire import (
    build_debug_snapshot,
    decode_debug_frame,
    pack_debug_frame,
    sanitize_debug_frame,
)
from bxi_example_py_elf3.sonic_pnlink.mujoco_viewer import (
    build_mujoco_xml,
    layout_skeletons,
    raw_affected_causes,
    raw_display_states,
    render_debug_frame,
    select_nearest_joint,
)
from bxi_example_py_elf3.sonic_pnlink.retarget import RetargetResult
from bxi_example_py_elf3.sonic_pnlink.temporal import (
    StandardFrame,
    StandardFrameResampler,
    build_pose_window,
)
from bxi_example_py_elf3.sonic_pnlink.retarget import (
    _smpl_global_rotations,
    _smpl_local_axis_angle,
    wrist_reference,
)
from bxi_example_py_elf3.sonic_pnlink import sdk_adapter
from bxi_example_py_elf3.sonic_pnlink.sdk_adapter import (
    CALIBRATE_MOTION,
    CLEAR_ZERO_DRIFT,
    RESUME_BODY,
    RESUME_HANDS,
    START_CAPTURE,
    ZERO_POSITION,
    PnLinkSdkAdapter,
    SdkConfig,
    avatar_to_frame,
    validate_sdk_installation,
)
from bxi_example_py_elf3.sonic_pnlink.skeleton import (
    JointLocalPose,
    PNLINK_JOINT_NAMES,
    PNLINK_PARENTS,
    REQUIRED_PNLINK_JOINTS,
    forward_kinematics,
)
from bxi_example_py_elf3.sonic_pnlink.pose_source import (
    NEUTRAL_SAMPLE_COUNT,
    SonicPnLinkPoseSource,
    SourceState,
    _calibration_guidance,
    _format_issue_message,
)
from bxi_example_py_elf3.sonic_pnlink.validation import (
    validate_raw_poses,
    validate_retarget_output,
)


def _identity_local_poses():
    return {
        name: JointLocalPose(
            np.array([100.0, 0.0, 0.0]) if parent is None else np.array([0.0, 1.0, 0.0]),
            np.array([1.0, 0.0, 0.0, 0.0]),
        )
        for name, parent in PNLINK_PARENTS.items()
    }


def test_pnlink_fk_converts_centimeters_and_coordinate_basis():
    world = forward_kinematics(_identity_local_poses())

    np.testing.assert_allclose(world.positions_robot_m["Hips"], [0.0, 1.0, 0.0])
    np.testing.assert_allclose(world.positions_robot_m["Spine"], [0.0, 1.0, 0.01])
    np.testing.assert_allclose(world.rotations_robot["Head"].as_matrix(), np.eye(3))


def test_markley_mean_handles_opposite_quaternion_signs():
    values = np.array([[1.0, 0.0, 0.0, 0.0], [-1.0, 0.0, 0.0, 0.0]])
    mean = markley_mean_wxyz(values)
    np.testing.assert_allclose(mean, [1.0, 0.0, 0.0, 0.0], atol=1.0e-7)


def test_neutral_calibration_accepts_25_stable_frames():
    identity = {name: Rotation.identity() for name in PNLINK_PARENTS}
    samples = [(index * 20_000_000, identity) for index in range(25)]

    result = calibrate_neutral(samples)

    assert result.sample_count == 25
    assert result.duration_seconds == 0.48
    assert result.max_angle_degrees == 0.0


def test_neutral_bone_lengths_use_median_of_25_frames():
    samples = []
    for index in range(25):
        sample = {
            name: 1.0
            for name in REQUIRED_PNLINK_JOINTS
            if PNLINK_PARENTS[name] is not None
        }
        if index == 0:
            sample["LeftUpLeg"] = 10.0
        samples.append(sample)

    result = median_bone_lengths_cm(samples)

    assert result["LeftUpLeg"] == 1.0
    assert result["RightUpLeg"] == 1.0


def test_status_guidance_reports_neutral_countdown():
    instruction, countdown = _calibration_guidance(SourceState.NEEDS_NEUTRAL, 10)

    assert instruction == "正在采集 T 姿态，请保持不动。"
    assert countdown == 0.3
    assert NEUTRAL_SAMPLE_COUNT == 25


def test_status_guidance_does_not_invent_vendor_calibration_countdown():
    instruction, countdown = _calibration_guidance(SourceState.VENDOR_CALIBRATING, 0)

    assert instruction == "正在进行厂商标定，请保持设备要求的标定姿态并等待完成。"
    assert countdown is None


def test_neutral_capture_can_start_without_vendor_calibration():
    source = SonicPnLinkPoseSource.__new__(SonicPnLinkPoseSource)
    source._lock = threading.RLock()
    source.state = SourceState.CAPTURING
    source.vendor_calibrated = False
    source.neutral = object()
    source.live_enabled = False
    source.neutral_bone_lengths_cm = {"LeftUpLeg": 10.0}
    source.smpl_neutral_lengths_m = {1: 0.1}
    source.neutral_samples = None
    source.neutral_bone_length_samples_cm = None
    clear_calls = []
    source._clear_live_buffers = lambda: clear_calls.append(True)
    source._publish_status = lambda **_kwargs: None

    success, message = source.capture_neutral()

    assert success is True
    assert message == "collecting 25 neutral frames"
    assert source.vendor_calibrated is False
    assert source.state == SourceState.NEEDS_NEUTRAL
    assert source.neutral is None
    assert source.neutral_samples == []
    assert source.neutral_bone_length_samples_cm == []
    assert source.neutral_bone_lengths_cm == {}
    assert source.smpl_neutral_lengths_m == {}
    assert clear_calls == [True]


def test_neutral_capture_rejects_live_state_until_paused():
    source = SonicPnLinkPoseSource.__new__(SonicPnLinkPoseSource)
    source._lock = threading.RLock()
    source.state = SourceState.LIVE

    success, message = source.capture_neutral()

    assert success is False
    assert message == "neutral capture requires active paused capture, current=LIVE; press P before T"


def test_vendor_calibration_completion_resets_rotation_continuity():
    messages = []
    source = SonicPnLinkPoseSource.__new__(SonicPnLinkPoseSource)
    source._lock = threading.RLock()
    source.vendor_calibrated = False
    source.state = SourceState.VENDOR_CALIBRATING
    source.previous_raw_poses = _identity_local_poses()
    source.logger = SimpleNamespace(
        info=lambda message: messages.append(("info", message)),
        error=lambda message: messages.append(("error", message)),
    )
    source._publish_status = lambda **_kwargs: None

    source._on_sdk_command_result(
        sdk_adapter.SdkCommandResult(CALIBRATE_MOTION, True, "completed")
    )

    assert source.vendor_calibrated is True
    assert source.state == SourceState.NEEDS_NEUTRAL
    assert source.previous_raw_poses is None
    assert messages == [
        ("info", "PN-Link vendor calibration complete; capture neutral T-pose next")
    ]


def test_diagnostic_log_message_includes_joint_context():
    issue = DiagnosticIssue(
        "PNLINK_FK",
        "BONE_LENGTH_JUMP",
        "position",
        "ACCEPTED",
        source_joint="LeftLeg",
        source_parent="LeftUpLeg",
        smpl_index=4,
        smpl_joint="left_knee",
        observed="deviation=0.162",
        severity="WARN",
    )

    assert _format_issue_message(issue) == (
        "PNLINK_FK/BONE_LENGTH_JUMP "
        "[joint=LeftLeg, parent=LeftUpLeg, smpl=left_knee[4]]: deviation=0.162"
    )


def test_diagnostic_log_message_without_joint_context_stays_compact():
    issue = DiagnosticIssue(
        "TEMPORAL_BUFFER",
        "SOURCE_STALE",
        "timestamp",
        "LIVE_REVOKED",
        observed="no AvatarUpdated frame",
        severity="STALE",
    )

    assert _format_issue_message(issue) == (
        "TEMPORAL_BUFFER/SOURCE_STALE: no AvatarUpdated frame"
    )


def test_vendor_calibration_suppresses_periodic_status(capsys):
    source = SonicPnLinkPoseSource.__new__(SonicPnLinkPoseSource)
    source.state = SourceState.VENDOR_CALIBRATING
    source._last_status_json = ""
    source._last_status_time = 0.0
    source._status_payload = lambda: {"state": source.state.value}

    source._publish_status()

    assert capsys.readouterr().out == ""

    source._publish_status(force=True)

    assert '"state":"VENDOR_CALIBRATING"' in capsys.readouterr().out


def test_status_metrics_are_rate_limited(monkeypatch, capsys):
    source = SonicPnLinkPoseSource.__new__(SonicPnLinkPoseSource)
    source.state = SourceState.CAPTURING
    source._last_status_json = ""
    source._last_status_time = 10.0
    source._status_payload = lambda: {"source_hz": 49.99}
    monkeypatch.setattr(
        "bxi_example_py_elf3.sonic_pnlink.pose_source.time.monotonic",
        lambda: 10.1,
    )

    source._publish_status()

    assert capsys.readouterr().out == ""


def test_pnlink_source_has_no_ros_runtime_dependency():
    source = Path(__file__).parents[1] / (
        "bxi_example_py_elf3/sonic_pnlink/pose_source.py"
    )
    text = source.read_text(encoding="utf-8")

    assert "import rclpy" not in text
    assert "create_service" not in text
    assert "create_publisher" not in text


def test_terminal_keys_dispatch_vendor_and_sonic_controls():
    calls = []
    source = SonicPnLinkPoseSource.__new__(SonicPnLinkPoseSource)
    source.logger = SimpleNamespace(
        info=lambda message: calls.append(("info", message)),
        warning=lambda message: calls.append(("warning", message)),
    )
    source.start_capture = lambda: (calls.append("start_capture") or (True, "ok"))
    source.stop_capture = lambda: (calls.append("stop_capture") or (True, "ok"))
    source.start_vendor_calibration = lambda: (calls.append("calibrate_motion") or (True, "ok"))
    source.capture_neutral = lambda: (calls.append("capture_neutral") or (True, "ok"))
    source.set_live = lambda enabled: (calls.append(("live", enabled)) or (True, "ok"))
    source.run_vendor_command = lambda command: (calls.append(command) or (True, "ok"))

    for key in ("n", "f", "c", "t", "l", "p", "r", "0", "o", "z"):
        assert source.handle_key(key)

    assert calls[::2] == [
        "start_capture",
        "stop_capture",
        "calibrate_motion",
        "capture_neutral",
        ("live", True),
        ("live", False),
        RESUME_HANDS,
        CLEAR_ZERO_DRIFT,
        RESUME_BODY,
        ZERO_POSITION,
    ]
    assert source.handle_key("\x1b") is False


def test_neutral_pose_retargets_to_identity_non_root_rotations():
    identity = {name: Rotation.identity() for name in PNLINK_PARENTS}
    global_rotations = _smpl_global_rotations(identity, identity)
    local = _smpl_local_axis_angle(global_rotations)

    np.testing.assert_allclose(local[1:], 0.0, atol=1.0e-6)


def test_wrist_reference_moves_elbow_swing_into_wrist():
    body_pose = np.zeros((21, 3), dtype=np.float64)
    body_pose[17, 0] = 0.1
    body_pose[19, 0] = 0.2
    body_pose[18, 2] = 0.15
    body_pose[20, 1] = 0.25

    wrist = wrist_reference(body_pose)

    np.testing.assert_allclose(wrist[[0, 4, 5]], [0.3, -0.25, 0.15], atol=1.0e-6)


def test_resampler_interpolates_positions_and_slerps_root_rotation():
    resampler = StandardFrameResampler(period_ns=20_000_000)
    first = StandardFrame(
        0,
        100.0,
        np.zeros((24, 3), dtype=np.float32),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        np.zeros(6, dtype=np.float32),
    )
    final_quat_xyzw = Rotation.from_euler("z", 90.0, degrees=True).as_quat()
    second = StandardFrame(
        40_000_000,
        100.04,
        np.full((24, 3), 2.0, dtype=np.float32),
        final_quat_xyzw[[3, 0, 1, 2]].astype(np.float32),
        np.full(6, 2.0, dtype=np.float32),
    )

    assert resampler.add(first) == []
    output = resampler.add(second)

    assert [frame.timestamp_ns for frame in output] == [0, 20_000_000, 40_000_000]
    np.testing.assert_allclose(output[1].smpl_joints, 1.0)
    middle_xyzw = output[1].root_quat[[1, 2, 3, 0]]
    assert np.isclose(Rotation.from_quat(middle_xyzw).magnitude(), np.pi / 4.0)


def test_pose_window_matches_pnlink_wire_contract():
    entries = [
        (
            index,
            StandardFrame(
                index * 20_000_000,
                100.0 + index * 0.02,
                np.zeros((24, 3), dtype=np.float32),
                np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
                np.zeros(6, dtype=np.float32),
            ),
        )
        for index in range(10)
    ]

    message = build_pose_window(entries)

    assert message["smpl_joints"].shape == (10, 24, 3)
    assert message["body_quat_w"].shape == (10, 4)
    assert message["wrist"].shape == (10, 6)
    assert message["frame_index"].dtype == np.int64


def test_sdk_avatar_mapping_is_normalized_to_raw_frame():
    avatar = {
        "joints": {
            "Hips": {
                "local_position": {"x": 1.0, "y": 2.0, "z": 3.0},
                "local_rotation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
            }
        }
    }

    frame = avatar_to_frame(avatar)

    np.testing.assert_array_equal(frame.joints["Hips"].position_cm, [1.0, 2.0, 3.0])
    np.testing.assert_array_equal(
        frame.joints["Hips"].quaternion_wxyz, [1.0, 0.0, 0.0, 0.0]
    )


def _fake_native_mcp_module():
    module = SimpleNamespace()
    module.MCPBvhData = SimpleNamespace(Binary=1)
    module.MCPBvhDisplacement = SimpleNamespace(Enable=2)
    module.MCPBvhRotation = SimpleNamespace(YXZ=3)
    module.MCPEventType = SimpleNamespace(AvatarUpdated=256, CommandReply=1536)
    module.MCPReplay = SimpleNamespace(MCPReplay_Result=2)
    module.EMCPCommand = SimpleNamespace(
        CommandStartCapture=0,
        CommandStopCapture=1,
        CommandCalibrateMotion=3,
        CommandResumeOriginalHandsPosture=4,
        CommandClearZeroMotionDrift=5,
        CommandResumeOriginalPosture=6,
        CommandZeroPosition=7,
    )

    class Settings:
        def __init__(self):
            self.calls = []

        def set_bvh_data(self, value):
            self.calls.append(("data", value))

        def set_bvh_transformation(self, value):
            self.calls.append(("transformation", value))

        def set_bvh_rotation(self, value):
            self.calls.append(("rotation", value))

        def SetSettingsUDPEx(self, host, port):
            self.calls.append(("local", host, port))

        def SetSettingsUDPServer(self, host, port):
            self.calls.append(("server", host, port))

    class Application:
        def __init__(self):
            self.settings = None
            self.commands = []
            self.events = []
            self.closed = False

        def set_settings(self, settings):
            self.settings = settings

        def open(self):
            return True, "NoError"

        def queue_command(self, command):
            self.commands.append(command)

        def poll_next_event(self):
            events, self.events = self.events, []
            return events

        def close(self):
            self.closed = True

    class Command:
        destroyed = []

        def get_result_code(self, _handle):
            return 0

        def get_result_message(self, _handle):
            return "unexpected"

        def destroy_command(self, handle):
            self.destroyed.append(handle)

    module.MCPSettings = Settings
    module.MCPApplication = Application
    module.MCPCommand = Command
    module.MCPAvatar = lambda handle: handle
    return module


def test_sdk_installation_accepts_vendor_lib_subdirectory(monkeypatch, tmp_path):
    (tmp_path / "mocap_api.py").write_text("# fake binding\n")
    (tmp_path / "lib").mkdir()
    (tmp_path / "lib" / "librobotapi_x86-64.so").write_bytes(b"fake")
    monkeypatch.setattr(sdk_adapter.platform, "machine", lambda: "x86_64")

    validate_sdk_installation(tmp_path)


def test_native_mcp_application_poll_and_async_commands(monkeypatch, tmp_path):
    (tmp_path / "mocap_api.py").write_text("# imported through test double\n")
    (tmp_path / "lib").mkdir()
    (tmp_path / "lib" / "librobotapi_x86-64.so").write_bytes(b"fake")
    module = _fake_native_mcp_module()
    monkeypatch.setattr(sdk_adapter.platform, "machine", lambda: "x86_64")
    monkeypatch.setattr(sdk_adapter.importlib, "import_module", lambda _name: module)
    frames = []
    results = []
    adapter = PnLinkSdkAdapter(
        SdkConfig(tmp_path, "10.42.0.101", 8002, "10.42.0.202", 8080),
        frames.append,
        results.append,
    )

    adapter.start()
    adapter.start_capture()
    assert adapter.client.commands == [0]
    avatar = {
        "joints": {
            "Hips": {
                "local_position": (1.0, 2.0, 3.0),
                "local_rotation": (1.0, 0.0, 0.0, 0.0),
            }
        }
    }
    adapter.client.events.extend(
        [
            SimpleNamespace(
                event_type=module.MCPEventType.AvatarUpdated,
                event_data=SimpleNamespace(avatar_handle=avatar),
            ),
            SimpleNamespace(
                event_type=module.MCPEventType.CommandReply,
                event_data=SimpleNamespace(
                    commandRespond=SimpleNamespace(_replay=2, _commandHandle=41)
                ),
            ),
        ]
    )

    adapter.poll()

    assert len(frames) == 1
    assert results[0].command == START_CAPTURE
    assert results[0].success is True
    adapter.calibrate_motion()
    assert adapter.client.commands == [0, 3]
    adapter.client.events.append(
        SimpleNamespace(
            event_type=module.MCPEventType.CommandReply,
            event_data=SimpleNamespace(
                commandRespond=SimpleNamespace(_replay=2, _commandHandle=42)
            ),
        )
    )
    adapter.poll()

    assert results[-1].command == CALIBRATE_MOTION
    assert module.MCPCommand.destroyed == [41, 42]
    adapter.close()
    assert adapter.client.closed is True


def test_native_mcp_vendor_keys_use_reference_command_enums(monkeypatch, tmp_path):
    (tmp_path / "mocap_api.py").write_text("# imported through test double\n")
    (tmp_path / "lib").mkdir()
    (tmp_path / "lib" / "librobotapi_x86-64.so").write_bytes(b"fake")
    module = _fake_native_mcp_module()
    monkeypatch.setattr(sdk_adapter.platform, "machine", lambda: "x86_64")
    monkeypatch.setattr(sdk_adapter.sys, "platform", "win32")
    monkeypatch.setattr(sdk_adapter.importlib, "import_module", lambda _name: module)
    adapter = PnLinkSdkAdapter(
        SdkConfig(tmp_path, "10.42.0.101", 8002, "10.42.0.202", 8080),
        lambda _frame: None,
    )

    commands = (
        (adapter.stop_capture, 1),
        (adapter.resume_hands, 4),
        (adapter.clear_zero_drift, 5),
        (adapter.resume_body, 6),
        (adapter.zero_position, 7),
    )
    for index, (invoke, expected) in enumerate(commands):
        invoke()
        assert adapter.client.commands[-1] == expected
        adapter.client.events.append(
            SimpleNamespace(
                event_type=module.MCPEventType.CommandReply,
                event_data=SimpleNamespace(
                    commandRespond=SimpleNamespace(
                        _replay=2,
                        _commandHandle=100 + index,
                    )
                ),
            )
        )
        adapter.poll()

    assert adapter.client.commands == [1, 4, 5, 6, 7]


def test_repeated_diagnostic_is_rate_limited_but_counted():
    reporter = DiagnosticReporter()
    first = DiagnosticIssue(
        "SDK_RAW", "JOINT_MISSING", "rotation", "FRAME_DROPPED",
        timestamp_ns=2_000_000_000,
    )
    repeated = DiagnosticIssue(
        "SDK_RAW", "JOINT_MISSING", "rotation", "FRAME_DROPPED",
        timestamp_ns=2_100_000_000,
    )

    assert reporter.report(first)
    assert not reporter.report(repeated)
    assert reporter.as_dict(repeated)["occurrences"] == 2


def test_raw_validation_localizes_position_and_rotation_independently():
    poses = _identity_local_poses()
    poses["LeftForeArm"] = JointLocalPose(
        np.array([np.nan, 1.0, 2.0]),
        np.array([1.0, 0.0, 0.0, 0.0]),
    )

    result = validate_raw_poses(poses, frame_index=42)

    issues = [issue for issue in result.issues if issue.source_joint == "LeftForeArm"]
    assert [(issue.code, issue.field) for issue in issues] == [
        ("POSITION_NONFINITE", "position")
    ]
    issue = issues[0]
    assert issue.smpl_index == 18
    assert issue.smpl_joint == "left_elbow"
    assert issue.origin is True


def test_raw_validation_does_not_create_bone_baseline_before_neutral():
    poses = _identity_local_poses()
    poses["LeftUpLeg"] = JointLocalPose(
        np.array([0.0, 50.0, 0.0]),
        np.array([1.0, 0.0, 0.0, 0.0]),
    )
    empty_baseline = {}

    result = validate_raw_poses(
        poses,
        frame_index=42,
        neutral_bone_lengths_cm=empty_baseline,
    )

    assert empty_baseline == {}
    assert not any(issue.code == "BONE_LENGTH_JUMP" for issue in result.issues)


def test_rotation_origin_marks_descendants_as_affected_without_duplicate_issues():
    poses = _identity_local_poses()
    poses["RightArm"] = JointLocalPose(
        np.zeros(3), np.zeros(4)
    )
    validation = validate_raw_poses(poses, frame_index=43)
    snapshot = build_debug_snapshot(
        frame_index=43,
        timestamp_monotonic_ns=1_000_000_000,
        joints=poses,
        raw_present=validation.present,
        raw_position_valid=validation.position_valid,
        raw_rotation_valid=validation.rotation_valid,
    )

    states = raw_display_states(snapshot)

    assert states[list(PNLINK_PARENTS).index("RightArm")] == "rotation_error"
    assert states[list(PNLINK_PARENTS).index("RightForeArm")] == "affected"
    causes = raw_affected_causes(snapshot)
    assert causes[list(PNLINK_PARENTS).index("RightForeArm")] == (
        "SDK_RAW/RightArm/rotation"
    )
    origins = [issue for issue in validation.issues if issue.severity == "ERROR"]
    assert [issue.source_joint for issue in origins] == ["RightArm"]


def test_missing_spine_joint_reports_smpl_mapping():
    poses = _identity_local_poses()
    poses.pop("Spine1")

    result = validate_raw_poses(poses, frame_index=44)

    issue = next(issue for issue in result.issues if issue.code == "JOINT_MISSING")
    assert issue.source_joint == "Spine1"
    assert issue.smpl_joint == "spine2"


def test_debug_wire_preserves_invalid_values_but_viewer_sanitizes_them():
    poses = _identity_local_poses()
    poses["LeftForeArm"] = JointLocalPose(
        np.array([np.nan, 0.0, 0.0]),
        np.array([1.0, 0.0, 0.0, 0.0]),
    )
    validation = validate_raw_poses(poses, frame_index=45)
    snapshot = build_debug_snapshot(
        frame_index=45,
        timestamp_monotonic_ns=2_000_000_000,
        joints=poses,
        raw_present=validation.present,
        raw_position_valid=validation.position_valid,
        raw_rotation_valid=validation.rotation_valid,
    )

    decoded = decode_debug_frame(pack_debug_frame(snapshot))
    safe, last_valid = sanitize_debug_frame(decoded)

    assert np.isnan(decoded["raw_local_pos_cm"]).any()
    assert np.isfinite(safe["raw_world_pos"]).all()
    assert np.isfinite(safe["smpl_joints"]).all()
    assert np.isfinite(last_valid["raw_world_quat"]).all()


def _diagnostic_frame(index: int) -> dict[str, np.ndarray]:
    return {
        "frame_index": np.array([index], dtype=np.int64),
        "timestamp_monotonic": np.array([index / 50.0], dtype=np.float64),
        "raw_local_pos_cm": np.full((23, 3), index, dtype=np.float32),
        "raw_local_quat": np.full((23, 4), index, dtype=np.float32),
        "retarget_global_quat": np.full((24, 4), index, dtype=np.float32),
        "smpl_local_axis_angle": np.full((22, 3), index, dtype=np.float32),
        "smpl_joints": np.full((24, 3), index, dtype=np.float32),
        "smpl_root_quat": np.full(4, index, dtype=np.float32),
        "wrist": np.full(6, index, dtype=np.float32),
        "source_stage_valid": np.ones(7, dtype=bool),
    }


def test_bundle_contains_bounded_pre_and_post_error_frames(tmp_path):
    recorder = DiagnosticBundleRecorder(
        tmp_path,
        pre_frames=3,
        post_frames=2,
        session_name="test_session",
    )
    for index in range(3):
        recorder.add_frame(_diagnostic_frame(index))
    issue = DiagnosticIssue(
        "SDK_RAW", "POSITION_NONFINITE", "position", "FRAME_DROPPED",
        frame_index=3,
    )
    assert recorder.trigger(issue)
    recorder.add_frame(_diagnostic_frame(3))
    recorder.add_frame(_diagnostic_frame(4))
    recorder.wait()
    recorder.close()

    bundles = list((tmp_path / "test_session").iterdir())
    assert len(bundles) == 1
    manifest = __import__("json").loads(
        (bundles[0] / "manifest.json").read_text(encoding="utf-8")
    )
    assert manifest["frame_count"] == 5
    assert manifest["post_frames_incomplete"] is False
    with np.load(bundles[0] / "output_frames.npz") as output:
        np.testing.assert_array_equal(output["frame_index"].reshape(-1), range(5))


def test_bundle_rotation_keeps_only_configured_limit(tmp_path):
    recorder = DiagnosticBundleRecorder(
        tmp_path,
        pre_frames=1,
        post_frames=0,
        max_bundles=2,
        dedupe_seconds=0.0,
        session_name="rotation",
    )
    for index in range(3):
        recorder.add_frame(_diagnostic_frame(index))
        recorder.dump_now(
            DiagnosticIssue(
                "SDK_RAW", f"MANUAL_{index}", "state", "RECORDED",
                severity="WARN",
            )
        )
    recorder.wait()
    recorder.close()

    assert len(list((tmp_path / "rotation").iterdir())) == 2


def test_bundle_write_failure_is_contained_in_worker(tmp_path):
    recorder = DiagnosticBundleRecorder(
        tmp_path, pre_frames=1, post_frames=0, session_name="failure"
    )

    def fail_write(_capture):
        raise OSError("disk unavailable")

    recorder._write_bundle = fail_write
    recorder.add_frame(_diagnostic_frame(1))
    recorder.dump_now(
        DiagnosticIssue(
            "SDK_RAW", "MANUAL", "state", "RECORDED", severity="WARN"
        )
    )
    recorder.wait()

    assert recorder.write_failures == 1
    assert recorder.last_write_error == "disk unavailable"
    assert recorder._worker.is_alive()
    recorder.close()


def test_viewer_layout_preserves_height_and_separates_skeletons():
    raw = np.zeros((23, 3), dtype=np.float32)
    smpl = np.zeros((24, 3), dtype=np.float32)
    raw[:, 2] = np.linspace(0.0, 1.0, 23)
    smpl[:, 2] = np.linspace(0.0, 1.0, 24)

    raw_display, smpl_display = layout_skeletons(raw, smpl)

    assert np.allclose(raw_display[:, 1], 0.8)
    assert np.allclose(smpl_display[:, 1], -0.8)
    np.testing.assert_array_equal(raw_display[:, 2], raw[:, 2])
    assert "sonic_pnlink_diagnostics" in build_mujoco_xml()
    assert rotation_matrix_is_valid(np.eye(3))
    assert not rotation_matrix_is_valid(np.diag([1.0, 1.0, 2.0]))


def test_smpl_fk_nan_is_attributed_to_smpl_fk_stage():
    result = RetargetResult(
        tuple(Rotation.identity() for _ in range(24)),
        np.zeros((22, 3), dtype=np.float32),
        np.full((24, 3), np.nan, dtype=np.float32),
        np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32),
        np.zeros(6, dtype=np.float32),
    )

    issues = validate_retarget_output(
        result, frame_index=46, neutral_bone_lengths_m={}
    )

    assert any(
        issue.stage == "SMPL_FK" and issue.code == "SMPL_FK_NONFINITE"
        for issue in issues
    )


def test_viewer_render_path_builds_nonblank_finite_scene():
    poses = _identity_local_poses()
    validation = validate_raw_poses(poses, frame_index=47)
    snapshot = build_debug_snapshot(
        frame_index=47,
        timestamp_monotonic_ns=3_000_000_000,
        joints=poses,
        raw_present=validation.present,
        raw_position_valid=validation.position_valid,
        raw_rotation_valid=validation.rotation_valid,
    )
    safe, _ = sanitize_debug_frame(snapshot)
    safe["source_stage_valid"][:] = False
    safe["source_stage_valid"][:2] = True

    class GeomTypes:
        mjGEOM_SPHERE = 1
        mjGEOM_CAPSULE = 2

    class FakeMujoco:
        mjtGeom = GeomTypes

        def __init__(self):
            self.connector_points = []

        @staticmethod
        def mjv_initGeom(*_args):
            return None

        def mjv_connector(self, _geom, _kind, _radius, start, end):
            self.connector_points.extend((np.array(start), np.array(end)))

    class FakeScene:
        def __init__(self):
            self.ngeom = 0
            self.maxgeom = 500
            self.geoms = [object() for _ in range(self.maxgeom)]

    mujoco = FakeMujoco()
    scene = FakeScene()

    render_debug_frame(mujoco, scene, safe, selected_smpl_joint=18)

    raw_only_geom_count = scene.ngeom
    assert raw_only_geom_count > len(PNLINK_JOINT_NAMES)
    assert np.isfinite(np.stack(mujoco.connector_points)).all()

    safe["source_stage_valid"][2] = True
    safe["smpl_position_valid"][:] = True
    safe["smpl_rotation_valid"][:] = True
    render_debug_frame(mujoco, scene, safe, selected_smpl_joint=18)

    assert scene.ngeom > raw_only_geom_count + 24


def test_mouse_projection_selects_centered_pelvis():
    class Camera:
        azimuth = 0.0
        elevation = 0.0
        distance = 3.0
        lookat = np.zeros(3)

    class Viewer:
        cam = Camera()

    class GlobalVisual:
        fovy = 45.0

    class Visual:
        global_ = GlobalVisual()

    class Model:
        vis = Visual()

    raw = np.full((len(PNLINK_JOINT_NAMES), 3), 10.0)
    raw[PNLINK_JOINT_NAMES.index("Hips")] = 0.0
    smpl = np.full((24, 3), 20.0)

    selected = select_nearest_joint(
        Viewer(), Model(), raw, smpl, (400.0, 300.0, 800, 600)
    )

    assert selected == 0
