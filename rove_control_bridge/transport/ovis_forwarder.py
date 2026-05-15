"""Forward RoveControl.Ovis frames to rove_ik_engine over UDP.

Every received RoveControl packet's `ovis` field is re-wrapped as a
`forgebot.engine.Ovis` proto with the configured target entity id, then
sent fire-and-forget to the engine's /Ovis listener. The engine handles
IK (and eventually collision sim) and is the layer that will, over time,
also take ownership of tracks + flippers conversion.

This is the input half of the engine integration. The output half
(engine StateUpdate → ODrive NodeCommands) is not implemented yet — for
now the engine drives the arm visually so operators can verify their
twist controls map correctly.
"""
from __future__ import annotations

import logging
import socket
from typing import Any

log = logging.getLogger(__name__)


class OvisForwarder:
    def __init__(
        self,
        engine_host: str,
        engine_port: int,
        target_entity_id: str,
    ) -> None:
        if not target_entity_id:
            raise ValueError(
                "OvisForwarder requires target_entity_id; set "
                "ovis.target_entity_id in the bridge config."
            )
        self._addr = (engine_host, engine_port)
        self._target = target_entity_id
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Lazy import: the proto module only exists after build_protos.py
        # has run, and we don't want to crash bridge startup when ovis is
        # disabled.
        from ..proto.core import IKEngineMessages_pb2

        self._engine_ovis_cls = IKEngineMessages_pb2.Ovis
        self._send_count = 0

        log.info(
            "OvisForwarder ready: bridge → rove_ik_engine UDP %s:%d  target=%s",
            engine_host, engine_port, target_entity_id,
        )

    def forward(self, rove_ovis: Any) -> None:
        """Re-wrap a RoveControl.Ovis as a forgebot.engine.Ovis and send.

        Failures are logged but don't propagate — the bridge's track /
        gripper loop must keep running regardless of engine health.
        """
        try:
            msg = self._engine_ovis_cls()
            msg.target = self._target
            msg.orientation.yaw = rove_ovis.orientation.yaw
            msg.orientation.pitch = rove_ovis.orientation.pitch
            msg.orientation.roll = rove_ovis.orientation.roll
            msg.position.x = rove_ovis.position.x
            msg.position.y = rove_ovis.position.y
            msg.position.z = rove_ovis.position.z
            self._sock.sendto(msg.SerializeToString(), self._addr)
            self._send_count += 1
        except Exception as exc:  # noqa: BLE001
            log.warning("OvisForwarder send failed: %s", exc)

    def close(self) -> None:
        self._sock.close()
