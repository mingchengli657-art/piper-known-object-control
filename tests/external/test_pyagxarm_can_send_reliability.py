"""Regression tests for the local pyAgxArm SocketCAN send fix."""

import errno

import pytest

from pyAgxArm.protocols.can_protocol.comms.can_comm import CanCommImpl


class FakeSendBus:
    def __init__(self, failures):
        self.failures = list(failures)
        self.calls = 0

    def send(self, _message, _timeout):
        self.calls += 1
        if self.failures:
            raise self.failures.pop(0)


def make_comm(failures):
    comm = object.__new__(CanCommImpl)
    comm.send_bus = FakeSendBus(failures)
    comm.recv_bus = object()
    comm.last_error = None
    comm._channel = "can0"
    return comm


def test_enobufs_is_retried_then_succeeds():
    comm = make_comm([OSError(errno.ENOBUFS, "buffer full")])
    comm.send(object())
    assert comm.send_bus.calls == 2
    assert comm.last_error is None


def test_persistent_enobufs_is_not_reported_as_success():
    comm = make_comm(
        [OSError(errno.ENOBUFS, "buffer full") for _ in range(3)]
    )
    with pytest.raises(RuntimeError, match="remained full"):
        comm.send(object())
    assert comm.send_bus.calls == 3


def test_enetdown_is_reported_immediately():
    comm = make_comm([OSError(errno.ENETDOWN, "network down")])
    with pytest.raises(RuntimeError, match="is down"):
        comm.send(object())
    assert comm.send_bus.calls == 1
