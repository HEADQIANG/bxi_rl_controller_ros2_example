import numpy as np

from bxi_example_py_elf3.sonic_pico.pico_pose_to_smpl_ref_bridge import (
    POSE_STREAM_MODE,
    PicoSourceReadinessGate,
    PoseSourceReadinessGate,
    StreamedSmplRefMerger,
    _build_argument_parser,
    _build_live_smpl_ref_if_ready,
    _decode_packed_message,
    _parse_incoming_chunk,
    _ros_diagnostic_level,
)
from bxi_example_py_elf3.sonic_pico.zmq_messages import pack_pose_message


STALE_SECONDS = 0.2


class _ByteDiagnosticStatus:
    OK = b"\x00"
    WARN = b"\x01"
    ERROR = b"\x02"
    STALE = b"\x03"


def test_ros_diagnostic_level_uses_message_constant_representation():
    assert _ros_diagnostic_level(None, _ByteDiagnosticStatus) == b"\x02"
    assert [
        _ros_diagnostic_level(level, _ByteDiagnosticStatus)
        for level in range(4)
    ] == [b"\x00", b"\x01", b"\x02", b"\x03"]


def _pose_fields(
    frame_start: int,
    *,
    stream_mode: int = POSE_STREAM_MODE,
    calibration_ready: bool = True,
) -> dict[str, np.ndarray]:
    frame_indices = np.arange(frame_start, frame_start + 10, dtype=np.int64)
    frame_count = frame_indices.size
    return {
        "stream_mode": np.array([stream_mode], dtype=np.int32),
        "calibration_ready": np.array([calibration_ready], dtype=bool),
        "frame_index": frame_indices,
        "smpl_joints": np.zeros((frame_count, 24, 3), dtype=np.float32),
        "body_quat_w": np.tile(
            np.array([[1.0, 0.0, 0.0, 0.0]], dtype=np.float32),
            (frame_count, 1),
        ),
        "joint_pos": np.zeros((frame_count, 29), dtype=np.float32),
    }


def _pnlink_pose_fields(frame_start: int) -> dict[str, np.ndarray]:
    fields = _pose_fields(frame_start)
    frame_count = fields["frame_index"].shape[0]
    fields.pop("joint_pos")
    fields["wrist"] = np.arange(frame_count * 6, dtype=np.float32).reshape(
        frame_count, 6
    )
    return fields


def test_gate_rejects_uncalibrated_and_non_pose_messages():
    gate = PicoSourceReadinessGate(required_consecutive=1)

    assert not gate.observe(
        _pose_fields(0, calibration_ready=False),
        now_mono=0.00,
        stale_seconds=STALE_SECONDS,
    )
    assert not gate.is_fresh(0.00, STALE_SECONDS)

    assert not gate.observe(
        _pose_fields(1, stream_mode=0),
        now_mono=0.01,
        stale_seconds=STALE_SECONDS,
    )
    assert not gate.is_fresh(0.01, STALE_SECONDS)


def test_gate_rejects_a_repeated_frame_and_revokes_ready_immediately():
    gate = PicoSourceReadinessGate(required_consecutive=1)
    fields = _pose_fields(10)

    assert gate.observe(fields, now_mono=0.00, stale_seconds=STALE_SECONDS)
    assert gate.is_fresh(0.00, STALE_SECONDS)
    assert not gate.observe(fields, now_mono=0.01, stale_seconds=STALE_SECONDS)
    assert not gate.is_fresh(0.01, STALE_SECONDS)


def test_gate_requires_three_consecutive_progressing_messages():
    gate = PicoSourceReadinessGate()

    assert not gate.observe(
        _pose_fields(20), now_mono=0.00, stale_seconds=STALE_SECONDS
    )
    assert not gate.observe(
        _pose_fields(21), now_mono=0.01, stale_seconds=STALE_SECONDS
    )
    assert gate.observe(
        _pose_fields(22), now_mono=0.02, stale_seconds=STALE_SECONDS
    )
    assert gate.is_fresh(0.02, STALE_SECONDS)


def test_gate_recovers_after_pose_session_frame_counter_resets():
    gate = PicoSourceReadinessGate()

    for index, frame_start in enumerate((1000, 1001, 1002)):
        gate.observe(
            _pose_fields(frame_start),
            now_mono=index * 0.01,
            stale_seconds=STALE_SECONDS,
        )
    assert gate.is_fresh(0.02, STALE_SECONDS)

    assert not gate.observe(
        _pose_fields(0), now_mono=0.03, stale_seconds=STALE_SECONDS
    )
    assert not gate.is_fresh(0.03, STALE_SECONDS)
    assert not gate.observe(
        _pose_fields(1), now_mono=0.04, stale_seconds=STALE_SECONDS
    )
    assert gate.observe(
        _pose_fields(2), now_mono=0.05, stale_seconds=STALE_SECONDS
    )
    assert gate.is_fresh(0.05, STALE_SECONDS)


def test_gate_is_not_fresh_after_stale_timeout():
    gate = PicoSourceReadinessGate()
    for index, frame_start in enumerate((30, 31, 32)):
        gate.observe(
            _pose_fields(frame_start),
            now_mono=index * 0.01,
            stale_seconds=STALE_SECONDS,
        )

    assert gate.is_fresh(0.02 + STALE_SECONDS, STALE_SECONDS)
    assert not gate.is_fresh(0.02 + STALE_SECONDS + 0.001, STALE_SECONDS)


def test_stale_gate_stops_live_output_and_clears_buffered_reference():
    gate = PicoSourceReadinessGate()
    latest_fields = None
    for index, frame_start in enumerate((40, 41, 42)):
        latest_fields = _pose_fields(frame_start)
        gate.observe(
            latest_fields,
            now_mono=index * 0.01,
            stale_seconds=STALE_SECONDS,
        )

    merger = StreamedSmplRefMerger()
    merger.merge(_parse_incoming_chunk(latest_fields, "pico_g1_legacy"))

    live_ref = _build_live_smpl_ref_if_ready(
        gate, merger, now_mono=0.02, stale_seconds=STALE_SECONDS
    )
    assert live_ref is not None
    assert bool(live_ref["source_ready"][0])
    assert int(live_ref["source_stream_mode"][0]) == POSE_STREAM_MODE

    stale_ref = _build_live_smpl_ref_if_ready(
        gate,
        merger,
        now_mono=0.02 + STALE_SECONDS + 0.001,
        stale_seconds=STALE_SECONDS,
    )
    assert stale_ref is None
    assert merger.timesteps == 0


def test_old_pico_gate_name_remains_a_compatible_alias():
    assert PicoSourceReadinessGate is PoseSourceReadinessGate


def test_pnlink_gate_accepts_direct_wrist_without_joint_pos():
    gate = PoseSourceReadinessGate(required_consecutive=1, source_kind="pnlink")
    fields = _pnlink_pose_fields(50)

    assert gate.observe(fields, now_mono=0.0, stale_seconds=STALE_SECONDS)
    assert gate.is_fresh(0.0, STALE_SECONDS)


def test_pnlink_chunk_uses_direct_wrist_field():
    fields = _pnlink_pose_fields(60)

    chunk = _parse_incoming_chunk(
        fields,
        wrist_source="pico_g1_legacy",
        source_kind="pnlink",
    )

    np.testing.assert_array_equal(chunk.wrist, fields["wrist"])
    assert chunk.term1_local.shape == (10, 72)
    assert chunk.root_quat.shape == (10, 4)


def test_pnlink_gate_rejects_nonfinite_direct_wrist():
    gate = PoseSourceReadinessGate(required_consecutive=1, source_kind="pnlink")
    fields = _pnlink_pose_fields(65)
    fields["wrist"][0, 0] = np.nan

    assert not gate.observe(fields, now_mono=0.0, stale_seconds=STALE_SECONDS)
    assert not gate.is_fresh(0.0, STALE_SECONDS)


def test_pico_chunk_keeps_legacy_joint_pos_wrist_mapping():
    fields = _pose_fields(66)
    fields["joint_pos"][:] = np.arange(29, dtype=np.float32)

    chunk = _parse_incoming_chunk(fields, wrist_source="pico_g1_legacy")

    expected = np.array([23, 25, 27, 24, 26, 28], dtype=np.float32)
    np.testing.assert_array_equal(chunk.wrist[0], expected)


def test_direct_wrist_takes_precedence_when_joint_pos_is_also_present():
    fields = _pose_fields(70)
    fields["wrist"] = np.full((10, 6), 3.0, dtype=np.float32)
    fields["joint_pos"][:] = 9.0

    chunk = _parse_incoming_chunk(fields, wrist_source="pico_g1_legacy")

    np.testing.assert_array_equal(chunk.wrist, fields["wrist"])


def test_pose_host_alias_and_source_kind_parse_without_breaking_pico_flag():
    parser = _build_argument_parser()

    pnlink = parser.parse_args(
        ["--source-kind", "pnlink", "--pose-host", "127.0.0.2"]
    )
    pico = parser.parse_args(["--pico-host", "127.0.0.3"])

    assert pnlink.source_kind == "pnlink"
    assert pnlink.pose_host == "127.0.0.2"
    assert pico.source_kind == "pico"
    assert pico.pose_host == "127.0.0.3"


def test_pnlink_pose_wire_decodes_and_passes_bridge_contract():
    original = _pnlink_pose_fields(80)
    message = pack_pose_message(original, topic="pose", version=4)

    decoded = _decode_packed_message(message, "pose")
    gate = PoseSourceReadinessGate(required_consecutive=1, source_kind="pnlink")
    assert gate.observe(decoded, now_mono=0.0, stale_seconds=STALE_SECONDS)
    chunk = _parse_incoming_chunk(
        decoded,
        wrist_source="pico_g1_legacy",
        source_kind="pnlink",
    )

    np.testing.assert_array_equal(chunk.frame_indices, original["frame_index"])
    np.testing.assert_array_equal(chunk.wrist, original["wrist"])
