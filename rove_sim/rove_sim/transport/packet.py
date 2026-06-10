"""rove_sensor_api wire codec -- byte-identical to src/protocol/packet.rs.

    [version:1B | msg_type:1B | seq:2B little-endian | JSON payload]

The sim speaks this exact frame so a real-mode RobotStateSource consumes the
same bytes whether they come from the mock SimSensorApi or the real robot.
"""
from __future__ import annotations

import json
import struct

PROTOCOL_VERSION = 0x01
MSG_SUBSCRIBE = 0x01
MSG_UNSUBSCRIBE = 0x02
MSG_DATA = 0x03
MSG_SUBSCRIBE_ACK = 0x04
MSG_COMMAND = 0x10
MSG_COMMAND_ACK = 0x11
MSG_ERROR = 0xFF


def encode(msg_type: int, seq: int, payload) -> bytes:
    body = json.dumps(payload).encode() if payload is not None else b""
    return struct.pack("<BBH", PROTOCOL_VERSION, msg_type, seq & 0xFFFF) + body


def decode(data: bytes):
    if len(data) < 4:
        raise ValueError("short packet")
    ver, mt, seq = struct.unpack("<BBH", data[:4])
    if ver != PROTOCOL_VERSION:
        raise ValueError(f"bad version {ver}")
    body = data[4:]
    return mt, seq, (json.loads(body) if body else None)
