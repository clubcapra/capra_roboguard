"""Forward RoveControl.Ovis frames to the rove_mvp_engine over UDP.

This is the input half of the engine integration: every received RoveControl
packet's `ovis` field is re-wrapped as a `forgebot.engine.Ovis` proto with
`target` filled in from config, then sent fire-and-forget to the engine's
UDP /Ovis listener. The output half (engine StateUpdate -> ODrive
NodeCommands) is not implemented yet -- this forward path exists so the
bundled engine UI can be used to visually verify the operator's twist
controls map correctly.
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
                "[ovis].target_entity_id in the bridge config."
            )
        self._addr = (engine_host, engine_port)
        self._target = target_entity_id
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Lazy import: the proto module is only available after build_protos.py
        # has run, and we don't want to crash bridge startup when ovis is
        # disabled.
        from .proto.core import IKEngineMessages_pb2

        self._engine_ovis_cls = IKEngineMessages_pb2.Ovis
        self._send_count = 0

        log.info(
            "OvisForwarder ready: rove_control_bridge -> engine UDP %s:%d  target=%s",
            engine_host,
            engine_port,
            target_entity_id,
        )

    def forward(self, rove_ovis: Any) -> None:
        """Encode the incoming RoveControl.Ovis into a forgebot.engine.Ovis
        and send to the engine. Errors are logged but don't propagate -- the
        bridge's track/gripper loop continues regardless."""
        try:
            msg = self._engine_ovis_cls()
            msg.target = self._target
            msg.orientation.yaw = rove_ovis.orientation.yaw
            msg.orientation.pitch = rove_ovis.orientation.pitch
            msg.orientation.roll = rove_ovis.orientation.roll
            msg.position.x = rove_ovis.position.x
            msg.position.y = rove_ovis.position.y
            msg.position.z = rove_ovis.position.z
            # tcp_offset_local left zero -- the bridge isn't computing visual
            # centroids; if the gripper target needs centroid-anchored rotate
            # the engine can configure it server-side later.
            self._sock.sendto(msg.SerializeToString(), self._addr)
            self._send_count += 1
        except Exception as exc:  # noqa: BLE001
            log.warning("OvisForwarder send failed: %s", exc)

    def close(self) -> None:
        self._sock.close()
