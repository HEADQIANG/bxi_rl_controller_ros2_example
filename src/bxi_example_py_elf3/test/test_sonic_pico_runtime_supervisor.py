import json
import signal
import subprocess

import pytest

from bxi_example_py_elf3.sonic_pico import runtime_supervisor
from bxi_example_py_elf3.sonic_pico.runtime_supervisor import (
    ChildProcess,
    PicoPipeline,
    StateSnapshotMonitor,
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
