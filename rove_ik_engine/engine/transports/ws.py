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
from typing import Any, Awaitable, Callable

from aiohttp import WSCloseCode, WSMsgType, web
from aiohttp.web_runner import AppRunner, TCPSite

from ..config import FlippersConfig, HardwareConfig, parse_bind
from ..hardware import snap_model_to_kinova
from ..flippers import (
    jog_flipper, release_flipper, set_flipper_sign, set_flipper_step, snap_model_to_flippers,
)
from ..jog import (
    delete_pose, list_movable_joints, list_poses, reset_to_home,
    save_pose, set_home, set_joint,
)
from ..motion import motion_status, plan_to_pose, stop_motion
from ..calibration import save_offsets
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
        flippers: "FlippersConfig | None" = None,
        offsets_path: Path | None = None,
        reheal: "Callable[[], Awaitable[dict]] | None" = None,
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
        self.flippers = flippers
        self.offsets_path = offsets_path
        self.reheal = reheal
        self._state_subs: set[web.WebSocketResponse] = set()
        self._ovis_subs: set[web.WebSocketResponse] = set()
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

        # Recover from a rove_sensor_api reload / e-stop recovery without a
        # restart: re-discover ports + re-anchor synced flippers. The e-stop
        # watchdog POSTs this on arm-recovery (alongside sensor_api's /reload).
        app.router.add_post("/api/v1/reload", self._http_reload)
        app.router.add_post("/reload", self._http_reload)

        # Flippers: capture the model<->ODrive offset (sync), manually rotate a
        # flipper joint in the model (jog, for pre-sync alignment), and inspect
        # live readings.
        app.router.add_post("/api/v1/flippers/sync", self._http_flippers_sync)
        app.router.add_post("/api/v1/flippers/jog", self._http_flippers_jog)
        app.router.add_get("/api/v1/flippers/status", self._http_flippers_status)
        # Flipper commanding: normalised +1/0/-1 step ramps a flipper target.
        app.router.add_post("/api/v1/flippers/command", self._http_flippers_command)
        app.router.add_post("/api/v1/flippers/release", self._http_flippers_release)
        # Drum velocity command (normalised -1..1 per node; velocity-mode drives).
        app.router.add_post("/api/v1/drives/velocity", self._http_drives_velocity)
        # Flip a flipper's motor->model sign (fix a mirrored side), re-anchored live.
        app.router.add_post("/api/v1/flippers/sign", self._http_flippers_sign)

        # General joint posing — set ANY movable joint (arm or flippers) to a
        # pose, so the operator can match the model to the real robot before
        # Sync. Backs the bundled UI's joint-pose panel.
        app.router.add_get("/api/v1/joints", self._http_joints_list)
        app.router.add_post("/api/v1/joints/set", self._http_joints_set)
        app.router.add_post("/api/v1/joints/reset", self._http_joints_reset)
        app.router.add_post("/api/v1/joints/home", self._http_joints_set_home)

        # Named pose library + joint-space pose-to-pose motion.
        app.router.add_get("/api/v1/poses", self._http_poses_list)
        app.router.add_post("/api/v1/poses/save", self._http_poses_save)
        app.router.add_post("/api/v1/poses/goto", self._http_poses_goto)
        app.router.add_post("/api/v1/poses/stop", self._http_poses_stop)
        app.router.add_post("/api/v1/poses/delete", self._http_poses_delete)

        # Live collision-aware IK toggle. Lets operators (or a watchdog)
        # disable collision IK without restarting the engine after the arm
        # locks itself into a self-collision and stops responding.
        app.router.add_get("/api/v1/ik/collision", self._http_ik_collision_get)
        app.router.add_post("/api/v1/ik/collision", self._http_ik_collision_set)

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
        # shutdown_timeout default is 60s — with a browser holding the /state and
        # /ovis WebSockets open, cleanup() blocks for that long on exit. We close
        # the sockets ourselves in stop(); keep this short as a backstop.
        site = TCPSite(runner, host, port, shutdown_timeout=1.0)
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
        # Close every live WebSocket first so aiohttp's cleanup() doesn't wait on
        # them (that wait is what made shutdown take ~a minute).
        for ws in list(self._state_subs) + list(self._ovis_subs):
            try:
                await ws.close(code=WSCloseCode.GOING_AWAY, message=b"shutdown")
            except Exception:  # noqa: BLE001
                pass
        self._state_subs.clear()
        self._ovis_subs.clear()
        if self._runner is not None:
            await self._runner.cleanup()

    # ---- WebSocket handlers ----

    async def _ws_ovis(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)
        self._ovis_subs.add(ws)
        try:
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
        finally:
            self._ovis_subs.discard(ws)
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
            inverted_joints=self.hardware.inverted_joints,
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
        if captured > 0:
            self._persist_offsets()
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

    async def _http_reload(self, _request: web.Request) -> web.Response:
        """Recover from a rove_sensor_api reload / e-stop recovery without
        restarting: re-discover served ports and re-anchor synced flippers to
        their live angle (the worm gear means a power-cycled flipper didn't move,
        so we keep its position and re-derive the offset). The e-stop watchdog
        calls this on arm-recovery so the engine self-heals instead of needing a
        reinstall."""
        if self.reheal is None:
            return web.json_response(
                {"ok": False, "error": "reheal not available (no drive bank configured)"},
                status=409,
            )
        try:
            result = await self.reheal()
        except Exception as exc:  # noqa: BLE001
            _log.exception("reload/reheal failed: %s", exc)
            return web.json_response({"ok": False, "error": str(exc)}, status=500)
        # Persist the (now re-anchored) state so a later restart resumes from it.
        if result.get("reanchored"):
            self._persist_offsets()
        return web.json_response({"ok": True, **result})

    async def _http_ik_collision_get(self, _request: web.Request) -> web.Response:
        return web.json_response({"enabled": bool(self.state.collision_aware)})

    async def _http_ik_collision_set(self, request: web.Request) -> web.Response:
        try:
            body = await request.json()
        except Exception:
            return web.json_response(
                {"ok": False, "error": "expected JSON body {enabled: bool}"},
                status=400,
            )
        if not isinstance(body, dict) or "enabled" not in body:
            return web.json_response(
                {"ok": False, "error": "missing 'enabled' field"},
                status=400,
            )
        enabled = bool(body["enabled"])
        prev = self.state.collision_aware
        self.state.collision_aware = enabled
        if prev != enabled:
            _log.warning(
                "collision_aware IK %s via HTTP (was %s)",
                "ENABLED" if enabled else "DISABLED",
                prev,
            )
        return web.json_response({"ok": True, "enabled": enabled, "previous": prev})

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
                # True after a successful sync: the engine is continuously
                # mirroring kinova into the model frame via kinova_offsets.
                "mirroring": bool(self.state.kinova_offsets),
            }
        )

    # ---- flippers ----

    async def _http_flippers_sync(self, _request: web.Request) -> web.Response:
        """Capture the model<->flipper offset for every flipper joint with a
        fresh ODrive reading. After this, the per-tick mirror tracks the real
        flippers. Align the model first (jog) so the captured offset is sane."""
        if self.flippers is None or not self.flippers.enabled:
            return web.json_response(
                {"ok": False, "errors": ["flippers disabled in engine.toml ([flippers].enabled=false)"]},
                status=409,
            )
        result = snap_model_to_flippers(self.state)
        _log.info(
            "flipper sync: captured %d joint(s)  missing_nodes=%s  offsets_deg=%s",
            result["captured"], result["missing_nodes"],
            {k[-8:]: round(v, 2) for k, v in result["offsets_deg"].items()},
        )
        if result.get("captured"):
            self._persist_offsets()
        return web.json_response(result)

    def _persist_offsets(self) -> None:
        """Save kinova + drive offsets so they survive a restart."""
        if self.offsets_path is not None:
            save_offsets(self.offsets_path, self.state)

    _FLIP_KEYS = {"fl": "FlipperFL", "fr": "FlipperFR", "rl": "FlipperBL", "rr": "FlipperBR"}

    async def _http_flippers_command(self, request: web.Request) -> web.Response:
        """Hold a normalised step ({-1,0,+1}) on one or more flippers; each ramps
        its target while non-zero. The MODEL always moves; a real ODrive packet
        only leaves if [drives].output_enabled is on. Body forms:
          {"joint":"FlipperFL","step":1}            single by name
          {"steps":{"41":1,"42":-1}}                by ODrive node id
          {"fl":1,"fr":0,"rl":0,"rr":-1}            convenience by corner."""
        try:
            body = await request.json()
        except Exception:
            return web.json_response({"ok": False, "error": "expected JSON body"}, status=400)
        if not isinstance(body, dict):
            return web.json_response({"ok": False, "error": "body must be an object"}, status=400)

        results = []
        if "joint" in body or "node" in body:
            results.append(set_flipper_step(
                self.state, joint=str(body.get("joint", "")),
                node_id=body.get("node"), step=int(body.get("step", 0))))
        if isinstance(body.get("steps"), dict):
            for k, v in body["steps"].items():
                results.append(set_flipper_step(self.state, node_id=int(k), step=int(v)))
        for corner, name in self._FLIP_KEYS.items():
            if corner in body:
                results.append(set_flipper_step(self.state, joint=name, step=int(body[corner])))
        if not results:
            return web.json_response({"ok": False, "error": "no flipper step in body"}, status=400)
        ok = all(r.get("ok") for r in results)
        return web.json_response({"ok": ok, "applied": results}, status=200 if ok else 400)

    async def _http_flippers_release(self, request: web.Request) -> web.Response:
        """Drop a flipper's command (hand it back to the read-only mirror). Body
        {"joint":..|"node":..} for one, or {} to release all."""
        try:
            body = await request.json()
        except Exception:
            body = {}
        body = body or {}
        if body.get("joint") or body.get("node") is not None:
            return web.json_response(release_flipper(
                self.state, joint=str(body.get("joint", "")), node_id=body.get("node")))
        # Release all.
        self.state.flipper_targets.clear()
        self.state.flipper_cmd_steps.clear()
        return web.json_response({"ok": True, "released": "all"})

    async def _http_drives_velocity(self, request: web.Request) -> web.Response:
        """Set normalised velocity (-1..1) for velocity-mode drives (drums). The
        sender scales by the node's max_vel_rev_s. Body forms:
          {"node":31,"vel":0.5}                  single by node id
          {"velocities":{"31":0.5,"32":-0.5}}    by node id
          {"left":0.5,"right":-0.5}              both sides (31,34 / 32,33)."""
        try:
            body = await request.json()
        except Exception:
            return web.json_response({"ok": False, "error": "expected JSON body"}, status=400)
        if not isinstance(body, dict):
            return web.json_response({"ok": False, "error": "body must be an object"}, status=400)
        vc = self.state.drive_vel_cmd
        applied = {}
        if "node" in body and "vel" in body:
            vc[int(body["node"])] = float(body["vel"]); applied[int(body["node"])] = float(body["vel"])
        if isinstance(body.get("velocities"), dict):
            for k, v in body["velocities"].items():
                vc[int(k)] = float(v); applied[int(k)] = float(v)
        if "left" in body:
            for n in (31, 34):
                vc[n] = float(body["left"]); applied[n] = float(body["left"])
        if "right" in body:
            for n in (32, 33):
                vc[n] = float(body["right"]); applied[n] = float(body["right"])
        if not applied:
            return web.json_response({"ok": False, "error": "no velocity in body"}, status=400)
        return web.json_response({"ok": True, "applied": applied})

    async def _http_flippers_sign(self, request: web.Request) -> web.Response:
        """Flip/set a flipper's motor->model sign and re-anchor live (no jump,
        future motion reverses). Body {"joint":"FlipperFR"} toggles; add
        {"sign":-1} to set explicitly. Persisted across restart."""
        try:
            body = await request.json()
        except Exception:
            body = {}
        body = body or {}
        result = set_flipper_sign(
            self.state, joint=str(body.get("joint", "")),
            node_id=body.get("node"), sign=body.get("sign"))
        if result["ok"]:
            self._persist_offsets()
        return web.json_response(result, status=200 if result["ok"] else 400)

    async def _http_flippers_jog(self, request: web.Request) -> web.Response:
        """Manually rotate a flipper joint in the model. Body:
        {"joint":"FlipperFL"|"node":41, "angle_deg":30}  (absolute), or
        {..., "delta_deg":5}  (relative). Pre-sync alignment aid."""
        try:
            body = await request.json()
        except Exception:
            return web.json_response(
                {"ok": False, "error": "expected JSON body"}, status=400
            )
        if not isinstance(body, dict):
            return web.json_response({"ok": False, "error": "body must be an object"}, status=400)
        result = jog_flipper(
            self.state,
            joint=str(body.get("joint", "")),
            node_id=body.get("node"),
            angle_deg=body.get("angle_deg"),
            delta_deg=body.get("delta_deg"),
        )
        return web.json_response(result, status=200 if result["ok"] else 400)

    async def _http_joints_list(self, _request: web.Request) -> web.Response:
        """Every movable joint (id, name, angle, axis, mirrored) for the pose panel."""
        return web.json_response({"joints": list_movable_joints(self.state)})

    async def _http_joints_set(self, request: web.Request) -> web.Response:
        """Pose one joint. Body: {"joint":"<id|name>", "angle_deg":x} (abs) or
        {..., "delta_deg":x} (relative)."""
        try:
            body = await request.json()
        except Exception:
            return web.json_response({"ok": False, "error": "expected JSON body"}, status=400)
        if not isinstance(body, dict) or not body.get("joint"):
            return web.json_response(
                {"ok": False, "error": "body must include 'joint' (id or name)"}, status=400
            )
        result = set_joint(
            self.state,
            str(body["joint"]),
            angle_deg=body.get("angle_deg"),
            delta_deg=body.get("delta_deg"),
        )
        return web.json_response(result, status=200 if result["ok"] else 400)

    async def _http_joints_reset(self, _request: web.Request) -> web.Response:
        """Restore all joints to home — do this before a from-home kinova Sync."""
        return web.json_response(reset_to_home(self.state))

    async def _http_joints_set_home(self, _request: web.Request) -> web.Response:
        """Capture the current model pose as the home pose, and persist it."""
        result = set_home(self.state)
        self._persist_offsets()
        return web.json_response(result)

    # ---- pose library + pose-to-pose motion ----

    async def _http_poses_list(self, _request: web.Request) -> web.Response:
        return web.json_response({"poses": list_poses(self.state),
                                  "motion": motion_status(self.state)})

    async def _http_poses_save(self, request: web.Request) -> web.Response:
        """Capture the current model pose under a name. Body: {"name": "stow"}."""
        try:
            body = await request.json()
        except Exception:
            return web.json_response({"ok": False, "error": "expected JSON body"}, status=400)
        result = save_pose(self.state, str((body or {}).get("name", "")))
        if result["ok"]:
            self._persist_offsets()
        return web.json_response(result, status=200 if result["ok"] else 400)

    async def _http_poses_goto(self, request: web.Request) -> web.Response:
        """Plan + start a joint-space move to a saved pose. Body:
        {"name":"stow", "speed_deg_s": 30}. MODEL-ONLY (no robot commands)."""
        try:
            body = await request.json()
        except Exception:
            return web.json_response({"ok": False, "error": "expected JSON body"}, status=400)
        body = body or {}
        name = str(body.get("name", ""))
        speed = body.get("speed_deg_s")
        kwargs = {"speed_deg_s": float(speed)} if isinstance(speed, (int, float)) else {}
        result = plan_to_pose(self.state, name, **kwargs)
        if result.get("ok"):
            _log.info("pose move -> %s (%.2fs, %d joints)",
                      name, result.get("duration_s", 0.0), result.get("joints", 0))
        return web.json_response(result, status=200 if result.get("ok") else 400)

    async def _http_poses_stop(self, _request: web.Request) -> web.Response:
        return web.json_response(stop_motion(self.state))

    async def _http_poses_delete(self, request: web.Request) -> web.Response:
        try:
            body = await request.json()
        except Exception:
            return web.json_response({"ok": False, "error": "expected JSON body"}, status=400)
        result = delete_pose(self.state, str((body or {}).get("name", "")))
        if result["ok"]:
            self._persist_offsets()
        return web.json_response(result, status=200 if result["ok"] else 400)

    async def _http_flippers_status(self, _request: web.Request) -> web.Response:
        """Live flipper readings + resolved mapping, for the UI / debugging."""
        if self.flippers is None or not self.flippers.enabled:
            return web.json_response({"enabled": False, "nodes": []})
        import time as _time
        now = _time.monotonic()
        nodes = []
        # joint entity id -> node id; report per joint so name + live value pair up.
        node_to_joint = {nid: eid for eid, nid in self.state.flipper_joint_to_node.items()}
        for n in self.flippers.nodes:
            eid = node_to_joint.get(n.node_id, "")
            ent = self.state.project.scene.entities.get(eid) if eid else None
            pos = self.state.latest_flipper_positions.get(n.node_id)
            last_t = self.state.latest_flipper_t.get(n.node_id, 0.0)
            nodes.append({
                "node_id": n.node_id,
                "joint": n.joint,
                "joint_eid": eid,
                "joint_name": (ent.name if ent else None),
                "pos_rev": pos,
                "frame_age_s": None if last_t == 0.0 else max(0.0, now - last_t),
                "synced": eid in self.state.flipper_offsets,
                "model_q_deg": (
                    self.state.joint_values.get(eid, 0.0) * 180.0 / 3.141592653589793
                    if eid else None
                ),
            })
        return web.json_response({
            "enabled": True,
            "gear_ratio": self.flippers.gear_ratio,
            "mirroring": bool(self.state.flipper_offsets),
            "nodes": nodes,
        })
