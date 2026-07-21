import sys
from pathlib import Path


VENDOR_ROOT = (
    Path(__file__).resolve().parents[1]
    / "bxi_example_py_elf3"
    / "sonic_pico"
    / "vendor"
)
sys.path.insert(0, str(VENDOR_ROOT))

from gear_sonic.scripts import pico_manager_thread_server as manager  # noqa: E402


class FakeUnixSocket:
    def __init__(self, connect_error=None):
        self.connect_error = connect_error
        self.closed = False

    def settimeout(self, _timeout):
        pass

    def connect(self, _path):
        if self.connect_error is not None:
            raise self.connect_error

    def close(self):
        self.closed = True


def test_stale_singleton_socket_is_detected(tmp_path, monkeypatch):
    socket_path = tmp_path / "roboticsservice.sock"
    socket_path.touch()
    client = FakeUnixSocket(ConnectionRefusedError())
    monkeypatch.setattr(manager.socket, "socket", lambda *_args: client)

    assert manager._xrt_singleton_socket_is_live(socket_path) is False
    assert client.closed is True


def test_live_singleton_socket_is_detected(tmp_path, monkeypatch):
    socket_path = tmp_path / "roboticsservice.sock"
    socket_path.touch()
    client = FakeUnixSocket()
    monkeypatch.setattr(manager.socket, "socket", lambda *_args: client)

    assert manager._xrt_singleton_socket_is_live(socket_path) is True
    assert client.closed is True


def test_live_grpc_service_port_is_detected(monkeypatch):
    connection = FakeUnixSocket()
    monkeypatch.setattr(
        manager.socket,
        "create_connection",
        lambda address, timeout: connection,
    )

    assert manager._xrt_service_port_is_live() is True
    assert connection.closed is True


def test_closed_grpc_service_port_is_detected(monkeypatch):
    def refuse_connection(_address, timeout):
        raise ConnectionRefusedError()

    monkeypatch.setattr(manager.socket, "create_connection", refuse_connection)

    assert manager._xrt_service_port_is_live() is False
