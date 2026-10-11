"""Control leases and fail-stop behavior at the browser-to-bridge seam."""
import importlib.util
from pathlib import Path
import socket
import struct
import time
from unittest.mock import patch

import pytest

spec = importlib.util.spec_from_file_location(
    "web_teleop", Path(__file__).parents[1] / "tools/web_teleop.py"
)
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def command(sequence, value=0.1):
    return dict(left=value, right=value, client="test", sequence=sequence)


def decode(data):
    assert data[:3] == b"\x0a\x0a\x0d" and data[7] == 0x15
    return struct.unpack("<f", data[3:7])[0], struct.unpack("<f", data[8:12])[0]


def test_control_lease_reordering_and_silence_stop():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1)
        teleop = module.Teleop(receiver.getsockname()[1], timeout=0.08)
        teleop.bridge_available = True
        teleop.health_checked = time.monotonic()
        try:
            teleop.update("session", command(1))
            assert decode(receiver.recv(64)) == pytest.approx((0.1, 0.1))
            with pytest.raises(ValueError, match="Another browser"):
                teleop.update("other", command(2))
            teleop.update("session", command(3, 0))
            assert decode(receiver.recv(64)) == (0, 0)
            with pytest.raises(ValueError, match="Out-of-order"):
                teleop.update("session", command(2))
            teleop.update("session", command(4))
            receiver.recv(64)
            assert decode(receiver.recv(64)) == (0, 0)
        finally:
            teleop.close()


def test_bridge_unavailable_sends_only_stop():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(1)
        teleop = module.Teleop(receiver.getsockname()[1])
        try:
            with patch.object(module.urllib.request, "urlopen", side_effect=OSError("offline")):
                with pytest.raises(ValueError, match="Control bridge unavailable"):
                    teleop.update("session", command(1))
            assert decode(receiver.recv(64)) == (0, 0)
            assert teleop.owner is None
        finally:
            teleop.close()
