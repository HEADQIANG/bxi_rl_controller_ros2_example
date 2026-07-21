import json
import signal
import subprocess

import pytest

from bxi_example_py_elf3.sonic_pico import runtime_supervisor
from bxi_example_py_elf3.sonic_pico.runtime_supervisor import (
    ChildProcess,
    PnLinkPauseRequest,
    PnLinkRuntime,
    PicoPipeline,
    StateSnapshotMonitor,
    default_pnlink_python_executable,
    pnlink_ready_paused,
    prepend_existing_ld_paths,
)


def snapshot(current, transition=None):
    return json.dumps(
        {
            "current": {"name": current, "id": 1, "elapsed": 0.0},
            "in_transition": transition is not None,
            "transition": transition,
        }
    )


def transition(source, destination):
    return {
        "from": {"name": source, "id": 1},
        "to": {"name": destination, "id": 23},
        "progress": 0.5,
    }


@pytest.mark.parametrize(
    ("payload", "expected"),
    [
        (snapshot("normal"), False),
        (snapshot("normal", transition("normal", "sonic_teleop")), True),
        (snapshot("sonic_teleop"), True),
        (snapshot("sonic_teleop", transition("sonic_teleop", "normal")), False),
    ],
)
def test_snapshot_tracks_transition_destination(payload, expected):
    monitor = StateSnapshotMonitor(heartbeat_timeout=1.0)

    assert monitor.update(payload, received_at=10.0) is None
    assert monitor.active(now=10.1) is expected


@pytest.mark.parametrize("payload", ["", "{", "[]", "null", "not-json"])
def test_malformed_snapshot_fails_closed(payload):
    monitor = StateSnapshotMonitor(heartbeat_timeout=1.0)

    assert monitor.update(payload, received_at=10.0) is not None
    assert monitor.active(now=10.0) is False


@pytest.mark.parametrize(
    "payload",
    [
        json.dumps({"current": "sonic_teleop", "transition": None}),
        json.dumps(
            {
                "current": {"name": "sonic_teleop"},
                "transition": "invalid",
            }
        ),
    ],
)
def test_malformed_snapshot_shape_fails_closed(payload):
    monitor = StateSnapshotMonitor(heartbeat_timeout=1.0)

    monitor.update(payload, received_at=10.0)

    assert monitor.active(now=10.0) is False


def test_snapshot_heartbeat_timeout_fails_closed():
    monitor = StateSnapshotMonitor(heartbeat_timeout=1.0)
    monitor.update(snapshot("sonic_teleop"), received_at=10.0)

    assert monitor.active(now=11.0) is True
    assert monitor.active(now=11.001) is False


class FakeLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(message)

    def error(self, message):
        self.messages.append(message)

    def warning(self, message):
        self.messages.append(message)


class FakeProcess:
    def __init__(self, pid, returncode=None):
        self.pid = pid
        self.returncode = returncode
        self.signals = []
        self.wait_calls = []

    def poll(self):
        return self.returncode

    def send_signal(self, signum):
        self.signals.append(signum)

    def wait(self, timeout=None):
        self.wait_calls.append(timeout)
        if self.returncode is None:
            raise subprocess.TimeoutExpired("fake-process", timeout)
        return self.returncode


def test_clean_child_exit_stops_its_peer_process_group(monkeypatch):
    manager = FakeProcess(pid=101, returncode=0)
    bridge = FakeProcess(pid=102)
    group_signals = []

    def fake_killpg(pgid, signum):
        group_signals.append((pgid, signum))
        if pgid == manager.pid:
            raise ProcessLookupError

    monkeypatch.setattr(runtime_supervisor.os, "killpg", fake_killpg)
    pipeline = PicoPipeline(FakeLogger(), "python3")
    pipeline.children = [
        ChildProcess("manager", manager),
        ChildProcess("bridge", bridge),
    ]

    assert pipeline.poll() is None

    assert pipeline.stopping is True
    assert manager.signals == []
    assert bridge.signals == []
    assert (bridge.pid, signal.SIGINT) in group_signals


def test_exited_leader_keeps_live_process_group_owned_and_signalled(monkeypatch):
    manager = FakeProcess(pid=201, returncode=0)
    group_signals = []

    def fake_killpg(pgid, signum):
        assert pgid == manager.pid
        group_signals.append(signum)
        # Both the SIGINT and the subsequent signal-0 liveness probe succeed:
        # the leader is gone, but a worker still occupies its process group.

    monkeypatch.setattr(runtime_supervisor.os, "killpg", fake_killpg)
    pipeline = PicoPipeline(FakeLogger(), "python3")
    pipeline.children = [ChildProcess("manager", manager)]

    assert pipeline.poll() is None

    assert pipeline.stopping is True
    assert pipeline.children
    assert group_signals == [signal.SIGINT, 0]


def test_bridge_spawn_failure_escalates_and_reaps_manager_group(monkeypatch):
    manager = FakeProcess(pid=301)
    popen_calls = []
    group_alive = True
    group_signals = []

    def fake_popen(command, **kwargs):
        popen_calls.append((command, kwargs))
        if len(popen_calls) == 1:
            return manager
        raise OSError("bridge exec failed")

    def fake_killpg(pgid, signum):
        nonlocal group_alive
        assert pgid == manager.pid
        if signum == 0:
            if group_alive:
                return
            raise ProcessLookupError
        group_signals.append(signum)
        if signum == signal.SIGKILL:
            group_alive = False
            manager.returncode = -signal.SIGKILL

    monkeypatch.setattr(runtime_supervisor.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(runtime_supervisor.os, "killpg", fake_killpg)
    monkeypatch.setattr(runtime_supervisor, "START_FAILURE_SIGINT_GRACE", 0.0)
    monkeypatch.setattr(runtime_supervisor, "START_FAILURE_SIGTERM_GRACE", 0.0)
    monkeypatch.setattr(runtime_supervisor, "START_FAILURE_SIGKILL_GRACE", 0.0)
    pipeline = PicoPipeline(FakeLogger(), "python3")

    with pytest.raises(OSError, match="bridge exec failed"):
        pipeline.start()

    assert group_signals == [signal.SIGINT, signal.SIGTERM, signal.SIGKILL]
    assert manager.wait_calls
    assert pipeline.children == []
    assert pipeline.stopping is False


def _option(command, name):
    return command[command.index(name) + 1]


def test_native_library_paths_are_prepended_once(tmp_path):
    sdk_dir = tmp_path / "SDK" / "x64"
    lib_dir = tmp_path / "lib"
    sdk_dir.mkdir(parents=True)
    lib_dir.mkdir()
    existing = tmp_path / "existing"
    existing.mkdir()
    env = {"LD_LIBRARY_PATH": f"{existing}:{lib_dir}"}

    added = prepend_existing_ld_paths(
        env,
        [str(sdk_dir), str(tmp_path), str(lib_dir), str(sdk_dir)],
    )

    assert added == [str(sdk_dir), str(tmp_path)]
    assert env["LD_LIBRARY_PATH"].split(":") == [
        str(sdk_dir),
        str(tmp_path),
        str(existing),
        str(lib_dir),
    ]


def test_pipeline_passes_xrt_native_paths_to_both_children(monkeypatch, tmp_path):
    sdk_dir = tmp_path / "SDK" / "x64"
    lib_dir = tmp_path / "lib"
    sdk_dir.mkdir(parents=True)
    lib_dir.mkdir()
    processes = [FakeProcess(pid=351), FakeProcess(pid=352)]
    popen_calls = []

    def fake_popen(command, **kwargs):
        popen_calls.append((command, kwargs))
        return processes[len(popen_calls) - 1]

    monkeypatch.setenv("SONIC_XRT_SERVICE_DIR", str(tmp_path))
    monkeypatch.setenv("LD_LIBRARY_PATH", "/already-present")
    monkeypatch.setattr(runtime_supervisor.subprocess, "Popen", fake_popen)

    PicoPipeline(FakeLogger(), "python3").start()

    assert len(popen_calls) == 2
    for _, kwargs in popen_calls:
        assert kwargs["env"]["LD_LIBRARY_PATH"].split(":") == [
            str(sdk_dir),
            str(tmp_path),
            str(lib_dir),
            "/already-present",
        ]


@pytest.mark.parametrize(
    ("new_values", "old_values", "expected"),
    [
        (
            ("new-host", "6007", "new-topic"),
            ("old-host", "5007", "old-topic"),
            ("new-host", "6007", "new-topic"),
        ),
        (
            (None, None, None),
            ("old-host", "5007", "old-topic"),
            ("old-host", "5007", "old-topic"),
        ),
    ],
)
def test_bridge_output_env_prefers_bxi_names_with_legacy_fallback(
    monkeypatch, new_values, old_values, expected
):
    new_names = (
        "BXI_SONIC_SMPL_REF_ZMQ_HOST",
        "BXI_SONIC_SMPL_REF_ZMQ_PORT",
        "BXI_SONIC_SMPL_REF_ZMQ_TOPIC",
    )
    old_names = (
        "SMPL_REF_ZMQ_HOST",
        "SMPL_REF_ZMQ_PORT",
        "SMPL_REF_ZMQ_TOPIC",
    )
    for name, value in zip(new_names, new_values):
        if value is None:
            monkeypatch.delenv(name, raising=False)
        else:
            monkeypatch.setenv(name, value)
    for name, value in zip(old_names, old_values):
        monkeypatch.setenv(name, value)

    processes = [FakeProcess(pid=401), FakeProcess(pid=402)]
    popen_calls = []

    def fake_popen(command, **kwargs):
        popen_calls.append((command, kwargs))
        return processes[len(popen_calls) - 1]

    monkeypatch.setattr(runtime_supervisor.subprocess, "Popen", fake_popen)
    pipeline = PicoPipeline(FakeLogger(), "python3")

    pipeline.start()

    bridge_command, kwargs = popen_calls[1]
    assert _option(bridge_command, "--out-host") == expected[0]
    assert _option(bridge_command, "--out-port") == expected[1]
    assert _option(bridge_command, "--out-topic") == expected[2]
    assert kwargs["start_new_session"] is True


def test_pico_manager_defaults_to_cpu(monkeypatch):
    processes = [FakeProcess(pid=501), FakeProcess(pid=502)]
    popen_calls = []

    def fake_popen(command, **kwargs):
        popen_calls.append((command, kwargs))
        return processes[len(popen_calls) - 1]

    monkeypatch.delenv("SONIC_PICO_USE_CUDA", raising=False)
    monkeypatch.setattr(runtime_supervisor.subprocess, "Popen", fake_popen)
    pipeline = PicoPipeline(FakeLogger(), "python3")

    pipeline.start()

    manager_command = popen_calls[0][0]
    assert "--cuda" not in manager_command


def test_pico_manager_cuda_is_explicit_opt_in(monkeypatch):
    processes = [FakeProcess(pid=601), FakeProcess(pid=602)]
    popen_calls = []

    def fake_popen(command, **kwargs):
        popen_calls.append((command, kwargs))
        return processes[len(popen_calls) - 1]

    monkeypatch.setenv("SONIC_PICO_USE_CUDA", "1")
    monkeypatch.setattr(runtime_supervisor.subprocess, "Popen", fake_popen)
    pipeline = PicoPipeline(FakeLogger(), "python3")

    pipeline.start()

    manager_command = popen_calls[0][0]
    assert "--cuda" in manager_command


def test_pnlink_python_has_robot_venv_default(monkeypatch):
    monkeypatch.delenv("SONIC_PNLINK_PYTHON", raising=False)

    assert default_pnlink_python_executable() == (
        "/opt/bxi/venvs/sonic_pnlink/bin/python"
    )


def test_pnlink_runtime_starts_source_persistently_and_bridge_separately(
    monkeypatch,
):
    processes = [FakeProcess(pid=701), FakeProcess(pid=702)]
    popen_calls = []

    def fake_popen(command, **kwargs):
        popen_calls.append((command, kwargs))
        return processes[len(popen_calls) - 1]

    env = {
        "SONIC_PNLINK_TOPOLOGY": "local",
        "SONIC_PNLINK_HOST": "127.0.0.1",
        "SONIC_PNLINK_PORT": "5556",
        "SONIC_PNLINK_SDK_PATH": "/opt/vendor/pnlink",
    }
    monkeypatch.setattr(runtime_supervisor.subprocess, "Popen", fake_popen)
    runtime = PnLinkRuntime(FakeLogger(), "/pnlink/python", env)

    runtime.start_persistent()

    assert len(popen_calls) == 1
    source_command, source_kwargs = popen_calls[0]
    assert source_command[:3] == [
        "/pnlink/python",
        "-m",
        "bxi_example_py_elf3.sonic_pnlink.pose_source",
    ]
    assert source_kwargs["env"]["SONIC_PNLINK_SDK_PATH"] == "/opt/vendor/pnlink"
    assert runtime.bridge.children == []

    runtime.start_bridge()

    assert len(popen_calls) == 2
    bridge_command = popen_calls[1][0]
    assert _option(bridge_command, "--source-kind") == "pnlink"
    assert _option(bridge_command, "--pose-host") == "127.0.0.1"
    assert _option(bridge_command, "--pico-port") == "5556"
    assert "--disable-ros-pico-topics" in bridge_command


@pytest.mark.parametrize(
    ("name", "value"),
    [
        ("SONIC_PNLINK_TOPOLOGY", "remote"),
        ("SONIC_PNLINK_HOST", "10.0.0.2"),
        ("SONIC_PNLINK_BIND_HOST", "0.0.0.0"),
        ("BXI_SONIC_SMPL_REF_ZMQ_HOST", "10.0.0.3"),
    ],
)
def test_pnlink_runtime_rejects_nonlocal_topology(monkeypatch, name, value):
    env = {
        "SONIC_PNLINK_TOPOLOGY": "local",
        "SONIC_PNLINK_HOST": "127.0.0.1",
        "SONIC_PNLINK_BIND_HOST": "127.0.0.1",
    }
    env[name] = value

    with pytest.raises(ValueError):
        PnLinkRuntime(FakeLogger(), "python3", env)


def test_pnlink_optional_viewer_is_started_as_low_priority(monkeypatch):
    processes = [FakeProcess(pid=711), FakeProcess(pid=712)]
    popen_calls = []

    def fake_popen(command, **kwargs):
        popen_calls.append((command, kwargs))
        return processes[len(popen_calls) - 1]

    env = {
        "SONIC_PNLINK_DIAGNOSTICS": "1",
        "SONIC_PNLINK_MUJOCO_VIEWER": "1",
    }
    monkeypatch.setattr(runtime_supervisor.subprocess, "Popen", fake_popen)
    runtime = PnLinkRuntime(FakeLogger(), "/pnlink/python", env)

    runtime.start_persistent()

    assert len(popen_calls) == 2
    assert popen_calls[1][0][:4] == ["nice", "-n", "10", "/pnlink/python"]
    assert popen_calls[1][0][-1] == "bxi_example_py_elf3.sonic_pnlink.mujoco_viewer"


def test_pnlink_source_exit_stops_bridge_without_restart(monkeypatch):
    source = FakeProcess(pid=721)
    bridge = FakeProcess(pid=722)
    processes = [source, bridge]
    group_signals = []

    def fake_popen(_command, **_kwargs):
        return processes.pop(0)

    def fake_killpg(pgid, signum):
        group_signals.append((pgid, signum))

    monkeypatch.setattr(runtime_supervisor.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(runtime_supervisor.os, "killpg", fake_killpg)
    runtime = PnLinkRuntime(FakeLogger(), "python3", {})
    runtime.start_persistent()
    runtime.start_bridge()
    source.returncode = 1

    runtime.poll()

    assert runtime.source_failed
    assert runtime.source.stopping
    assert runtime.bridge.stopping
    assert (bridge.pid, signal.SIGINT) in group_signals
    with pytest.raises(RuntimeError, match="restart is required"):
        runtime.start_persistent()


def test_pnlink_viewer_exit_does_not_stop_source_or_bridge(monkeypatch):
    source = FakeProcess(pid=731)
    viewer = FakeProcess(pid=732)
    bridge = FakeProcess(pid=733)
    processes = [source, viewer, bridge]

    def fake_popen(_command, **_kwargs):
        return processes.pop(0)

    def fake_killpg(pgid, signum):
        if pgid == viewer.pid and signum == 0:
            raise ProcessLookupError

    env = {
        "SONIC_PNLINK_DIAGNOSTICS": "1",
        "SONIC_PNLINK_MUJOCO_VIEWER": "1",
    }
    monkeypatch.setattr(runtime_supervisor.subprocess, "Popen", fake_popen)
    monkeypatch.setattr(runtime_supervisor.os, "killpg", fake_killpg)
    runtime = PnLinkRuntime(FakeLogger(), "python3", env)
    runtime.start_persistent()
    runtime.start_bridge()
    viewer.returncode = 1

    runtime.poll()

    assert runtime.viewer_failed
    assert not runtime.source.stopping
    assert not runtime.bridge.stopping
    assert runtime.source_failed is False


class FakeResponse:
    def __init__(self, success=True):
        self.success = success


class FakeFuture:
    def __init__(self, done=False, response=None, error=None):
        self._done = done
        self.response = response
        self.error = error

    def done(self):
        return self._done

    def result(self):
        if self.error is not None:
            raise self.error
        return self.response


def test_pnlink_pause_requires_service_and_ready_paused_status():
    future = FakeFuture(done=True, response=FakeResponse(success=True))
    request = PnLinkPauseRequest(started_at=10.0, future=future)

    assert request.evaluate(10.1) is None
    request.observe_status(
        json.dumps({"state": "READY_PAUSED", "live_enabled": False})
    )

    assert request.evaluate(10.1) == "confirmed"
    assert pnlink_ready_paused("not-json") is False


def test_pnlink_pause_fails_closed_on_service_or_status_timeout():
    pending = PnLinkPauseRequest(
        started_at=10.0,
        future=FakeFuture(done=False),
    )
    completed = PnLinkPauseRequest(
        started_at=10.0,
        future=FakeFuture(done=True, response=FakeResponse(success=True)),
    )

    assert pending.evaluate(10.201) == "service_timeout"
    assert completed.evaluate(10.501) == "status_timeout"


def test_pnlink_pause_does_not_accept_status_before_service_succeeds():
    request = PnLinkPauseRequest(
        started_at=10.0,
        future=FakeFuture(done=False),
    )
    request.observe_status(
        json.dumps({"state": "READY_PAUSED", "live_enabled": False})
    )

    assert request.evaluate(10.1) is None
    assert request.evaluate(10.201) == "service_timeout"
