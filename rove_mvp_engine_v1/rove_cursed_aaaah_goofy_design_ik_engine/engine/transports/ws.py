"""HTTP + WebSocket server (aiohttp).

One server hosts everything the bundled browser UI needs:

  WS    /ovis        Ovis frames in
  WS    /state       StateUpdate frames out
  HTTP  /api/scene   canonical scene JSON (matches editor's /api/v1/scene)
  HTTP  /api/v1/scene             same payload (editor URL compat)
  HTTP  /api/v1/assets/meshes     mesh listing
  HTTP  /api/v1/assets/mesh/{n}   single mesh by stem
  HTTP  /            UI dist (if bundled at engine.toml [ui].dir)
  HTTP  /assets/*    UI dist static assets
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Any

from aiohttp import WSMsgType, web
from aiohttp.web_runner import AppRunner, TCPSite

from ..config import HardwareConfig, parse_bind
from ..hardware import snap_model_to_kinova
from ..proto import Ovis
from ..state import EngineState
from .bus import StateBus

_log = logging.getLogger(__name__)


class HttpWsServer:
    def __init__(
        self,
        state: EngineState,
        bus: StateBus,
        bind: str,
        *,
        input_enabled: bool,
        input_path: str,
        output_enabled: bool,
        output_path: str,
        ui_dir: Path | None,
        data_dir: Path,
        hardware: HardwareConfig | None = None,
    ) -> None:
        self.state = state
        self.bus = bus
        self.bind = bind
        self.input_enabled = input_enabled
        self.input_path = input_path
        self.output_enabled = output_enabled
        self.output_path = output_path
        self.ui_dir = ui_dir if (ui_dir and ui_dir.exists()) else None
        self.data_dir = data_dir
        self.hardware = hardware
        self._state_subs: set[web.WebSocketResponse] = set()
        self._runner: AppRunner | None = None
        if output_enabled:
            bus.subscribe(self._broadcast_state)

    async def start(self) -> None:
        app = web.Application()
        if self.input_enabled:
            app.router.add_get(self.input_path, self._ws_ovis)
        if self.output_enabled:
            app.router.add_get(self.output_path, self._ws_state)

        # Scene metadata: served on the canonical URL the editor's frontend
        # already uses, so the bundled UI doesn't need a different code
        # path to find the scene.
        app.router.add_get("/api/scene", self._http_scene)
        app.router.add_get("/api/v1/scene", self._http_scene)
        app.router.add_get("/api/v1/scene/", self._http_scene)
        app.router.add_get("/api/v1/scene/roots", self._http_roots)
        app.router.add_get("/api/v1/assets/meshes", self._http_meshes_list)
        app.router.add_get(
            "/api/v1/assets/mesh/{stem}", self._http_mesh_by_stem
        )
        # IK profiles, joint values — the bundled UI may want these for the
        # initial-state hydration.
        app.router.add_get("/api/v1/kinematics/profiles", self._http_profiles)
        app.router.add_get("/api/v1/scene/joints", self._http_joint_values)
        # Snap model joints to the latest kinova state (debug button).
        app.router.add_post("/api/v1/sync", self._http_sync)
        app.router.add_get("/api/v1/sync/status", self._http_sync_status)

        if self.ui_dir is not None:
            app.router.add_get("/", self._http_ui_index)
            # Vite emits assets under /assets/ — serve those (and any other
            # subfolder the build produces) as static files.
            for entry in self.ui_dir.iterdir():
                if entry.is_dir():
                    app.router.add_static(
                        f"/{entry.name}",
                        entry,
                        show_index=False,
                        follow_symlinks=False,
                    )

        runner = AppRunner(app, access_log=None)
        await runner.setup()
        host, port = parse_bind(self.bind)
        site = TCPSite(runner, host, port)
        await site.start()
        self._runner = runner
        _log.info(
            "HTTP/WS listening on %s:%d  ovis=%s  state=%s  ui=%s",
            host,
            port,
            self.input_path if self.input_enabled else "off",
            self.output_path if self.output_enabled else "off",
            self.ui_dir or "off",
        )

    async def stop(self) -> None:
        if self._runner is not None:
            await self._runner.cleanup()

    # ---- WebSocket handlers ----

    async def _ws_ovis(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        async for msg in ws:
            if msg.type == WSMsgType.BINARY:
                try:
                    ovis = Ovis()
                    ovis.ParseFromString(msg.data)
                except Exception as e:  # noqa: BLE001
                    _log.debug("dropped malformed Ovis on WS: %s", e)
                    continue
                self.state.set_ovis(ovis)
            elif msg.type == WSMsgType.ERROR:
                _log.debug("WS /ovis error: %s", ws.exception())
        return ws

    async def _ws_state(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._state_subs.add(ws)
        try:
            async for _ in ws:
                # Ignore inbound traffic; this is an output-only socket.
                pass
        finally:
            self._state_subs.discard(ws)
        return ws

    async def _broadcast_state(self, frame: bytes) -> None:
        if not self._state_subs:
            return
        dead: list[web.WebSocketResponse] = []
        for ws in self._state_subs:
            if ws.closed:
                dead.append(ws)
                continue
            try:
                await ws.send_bytes(frame)
            except Exception as e:  # noqa: BLE001
                _log.debug("WS send failed: %s", e)
                dead.append(ws)
        for ws in dead:
            self._state_subs.discard(ws)

    # ---- HTTP handlers ----

    async def _http_scene(self, _request: web.Request) -> web.Response:
        scene_dict = self.state.project.scene.to_toml_dict()
        return web.json_response(scene_dict)

    async def _http_roots(self, _request: web.Request) -> web.Response:
        return web.json_response({"roots": list(self.state.project.scene.roots)})

    async def _http_profiles(self, _request: web.Request) -> web.Response:
        profiles = {
            base: prof.model_dump()
            for base, prof in (self.state.project.ik_profiles or {}).items()
        }
        return web.json_response(profiles)

    async def _http_joint_values(self, _request: web.Request) -> web.Response:
        return web.json_response(
            {jid: float(q) for jid, q in self.state.joint_values.items()}
        )

    async def _http_meshes_list(self, _request: web.Request) -> web.Response:
        # Match the editor's /api/v1/assets/meshes shape exactly so the
        # bundled UI (built from the same source) doesn't need a branch.
        meshes_dir = self.data_dir / "meshes"
        out: dict[str, dict[str, Any]] = {}
        if meshes_dir.exists():
            for f in sorted(meshes_dir.iterdir()):
                if f.is_file():
                    out[f.stem] = {
                        "suffix": f.suffix,
                        "size_bytes": f.stat().st_size,
                        "usage": [],
                    }
        return web.json_response(out)

    async def _http_mesh_by_stem(self, request: web.Request) -> web.FileResponse | web.Response:
        stem = request.match_info["stem"]
        meshes_dir = self.data_dir / "meshes"
        if not meshes_dir.exists():
            raise web.HTTPNotFound()
        for f in meshes_dir.iterdir():
            if f.is_file() and f.stem == stem:
                return web.FileResponse(f)
        raise web.HTTPNotFound()

    async def _http_ui_index(self, _request: web.Request) -> web.FileResponse:
        assert self.ui_dir is not None
        return web.FileResponse(self.ui_dir / "index.html")

    async def _http_sync(self, _request: web.Request) -> web.Response:
        if self.hardware is None or not self.hardware.enabled:
            return web.json_response(
                {
                    "ok": False,
                    "updated": 0,
                    "errors": ["hardware sync disabled in engine.toml ([hardware].enabled=false)"],
                },
                status=409,
            )
        captured, errors, joint_ids, offsets = snap_model_to_kinova(
            self.state,
            arm_base_entity_id=self.hardware.arm_base_entity_id,
            arm_tip_entity_id=self.hardware.arm_tip_entity_id,
            joint_names=self.hardware.joint_names,
        )
        # Log offsets in degrees so the user can sanity-check them against
        # the Kinova 180-degree-zero convention.
        import math as _math
        offsets_deg = {k: v * 180.0 / _math.pi for k, v in offsets.items()}
        _log.info(
            "sync calibrated %d joints  chain=%s  offsets_deg=%s",
            captured,
            joint_ids,
            {k[-8:]: round(v, 2) for k, v in offsets_deg.items()},
        )
        for err in errors:
            _log.warning("  sync error: %s", err)
        return web.json_response(
            {
                "ok": captured > 0,
                "captured": captured,
                "errors": errors,
                "positions": self.state.latest_kinova_positions,
                "joint_ids": joint_ids,
                "offsets": offsets,           # radians
                "offsets_deg": offsets_deg,   # degrees, for the UI display
            }
        )

    async def _http_sync_status(self, _request: web.Request) -> web.Response:
        """Lets the UI grey out the Sync button until the engine has actually
        received a kinova frame, and shows the resolved chain so the
        operator can spot a misconfigured base/tip."""
        if self.hardware is None or not self.hardware.enabled:
            return web.json_response({"enabled": False, "have_frame": False})
        from ..hardware import resolve_arm_joint_ids
        joint_ids, errors = resolve_arm_joint_ids(
            self.state,
            arm_base_entity_id=self.hardware.arm_base_entity_id,
            arm_tip_entity_id=self.hardware.arm_tip_entity_id,
            joint_names=self.hardware.joint_names,
        )
        return web.json_response(
            {
                "enabled": True,
                "have_frame": self.state.latest_kinova_positions is not None,
                "frame_age_s": (
                    None
                    if self.state.latest_kinova_t == 0.0
                    else max(0.0, __import__("time").monotonic() - self.state.latest_kinova_t)
                ),
                "joint_ids": joint_ids,
                "mapping_errors": errors,
            }
        )
