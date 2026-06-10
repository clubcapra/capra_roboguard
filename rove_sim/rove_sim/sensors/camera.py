"""Camera sensor (M2): the GPU render wrapped as a Sensor.

Each instance owns a mount link and intrinsics (width/height/hfov). _sample()
reads the link's world pose, builds the view from the link's optical axis and a
FOV projection, and renders one frame through the engine's renderer (EGL GPU
offscreen headless, the live GL context in GUI). The reading carries RGB + metric
depth + segmentation + the pose/intrinsics, which is what the RTSP plane and the
autonomy vision stack consume downstream.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, Tuple

import numpy as np
import pybullet as p

from .base import Sensor, register


@dataclass
class CameraFrame:
    rgb: np.ndarray                       # (H, W, 3) uint8
    depth: np.ndarray                     # (H, W) float32, METRES
    seg: np.ndarray                       # (H, W) int32 body-id (-1 = miss)
    width: int
    height: int
    pose: Tuple[Any, Any]                 # (eye xyz, orn xyzw)
    intrinsics: Dict[str, float] = field(default_factory=dict)
    t: float = 0.0


@register("camera")
class Camera(Sensor):
    def __init__(self, **kw):
        self.width = int(kw.pop("width", 640))
        self.height = int(kw.pop("height", 480))
        self.hfov = float(kw.pop("hfov_deg", 90.0))
        self.near = float(kw.pop("near", 0.05))
        # far is kept modest: a huge far/near ratio wrecks depth-buffer precision
        # and the layered terrain z-fights (texture "flicker"). 60 m is plenty for
        # the cameras (the lidar carries the long range); ratio 1200:1 vs 4000:1.
        self.far = float(kw.pop("far", 60.0))
        # optical axis / up in the MOUNT-LINK frame. Robot camera links face +X
        # (forward) with +Z up by default; override per-mount if a link differs.
        self.forward = tuple(kw.pop("forward", (1.0, 0.0, 0.0)))
        self.up_axis = tuple(kw.pop("up", (0.0, 0.0, 1.0)))
        # tilt the optical axis DOWN by this many degrees in the world vertical
        # plane (nav cams look out + slightly down to catch the ground, obstacles
        # near the robot, and the robot's own body edge -- not just far terrain).
        self.pitch_down = float(kw.pop("pitch_down_deg", 0.0))
        # inspection cam: aim the optical axis AT this robot link (e.g. the gripper)
        # so the subject stays centred whatever the arm pose -- overrides `forward`.
        self.aim_link = kw.pop("aim_link", None)
        # paint the robot OUT of this camera's render. The nav cams sit in the
        # green sensor cluster and would otherwise show the other cameras/mast
        # filling the frame; masking the whole robot gives a clean outward view.
        # default: show the robot (its real dark colour). Set True to paint it out.
        self.mask_robot = bool(kw.pop("mask_robot", False))
        # body ids ALWAYS painted out (set by the runtime): the lidar self-occluder
        # proxies, which render their collision mesh as a box right next to the cam.
        self.mask_ids = set(kw.pop("mask_ids", None) or [])
        # Level the horizon to world-up by default (nav cams shouldn't inherit the
        # mount-link roll). Set roll_with_mount: true to follow the link's up.
        self.roll_with_mount = bool(kw.pop("roll_with_mount", False))
        kw.setdefault("rate_hz", 15.0)          # cameras are expensive; not 50 Hz
        super().__init__(**kw)
        aspect = self.width / self.height
        vfov = np.degrees(2.0 * np.arctan(np.tan(np.radians(self.hfov) / 2.0) / aspect))
        self._proj = p.computeProjectionMatrixFOV(vfov, aspect, self.near, self.far)
        self._renderer = (self.engine.camera_renderer_flag if self.engine is not None
                          else p.ER_TINY_RENDERER)

    def _centroid(self, pos) -> np.ndarray:
        """World centroid of the camera's OWN housing (eye point). The lens sits at
        the body centre; rendering from here (with the near plane clipping the
        housing itself) means the camera never renders the body it looks out of."""
        idx = self.robot.link_index.get(self.link)
        if idx is not None and idx >= 0:
            try:
                lo, hi = p.getAABB(self.robot.body_id, idx)
                return (np.array(lo) + np.array(hi)) * 0.5
            except Exception:
                pass
        return np.array(pos)

    def _view(self):
        """(pos, orn, eye, view) for this frame: eye = housing centroid, optical
        axis = aim-link midpoint if set else the modelled lens, horizon-levelled."""
        pos, orn = self.link_pose()
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        eye = self._centroid(pos)                 # look out from the lens/centroid
        if self.aim_link:                         # inspection cam: look AT link(s)
            targets = (self.aim_link if isinstance(self.aim_link, (list, tuple))
                       else [self.aim_link])
            pts = [np.array(p.getLinkState(self.robot.body_id, ti,
                                           computeForwardKinematics=True)[4])
                   for t in targets
                   if (ti := self.robot.link_index.get(t)) is not None]
            f = (np.mean(pts, axis=0) - eye) if pts else R @ np.array(self.forward)
        else:
            f = R @ np.array(self.forward)        # optical axis in world (lens dir)
        f = f / (np.linalg.norm(f) + 1e-9)
        if self.pitch_down:                       # optional downward tilt
            horiz = np.cross(f, [0, 0, 1.0])
            if np.linalg.norm(horiz) > 1e-6:
                from numpy import cos, sin, radians
                a = radians(self.pitch_down); k = horiz / np.linalg.norm(horiz)
                f = f * cos(a) + np.cross(k, f) * sin(a) + k * (k @ f) * (1 - cos(a))
        if self.roll_with_mount:
            u = R @ np.array(self.up_axis)
        else:                                     # level horizon to world-up
            u = np.array([0.0, 0.0, 1.0])
            if abs(f @ u) > 0.99:                  # looking near-vertical
                u = R @ np.array(self.up_axis)     # fall back to mount up
        view = p.computeViewMatrix(eye.tolist(), (eye + f).tolist(), u.tolist())
        return pos, orn, eye, view

    def _render(self, view, want_depth: bool):
        """getCameraImage + the tuple->numpy conversions (the real per-frame cost).
        Skips the segmentation readback unless masking, and the depth tuple+metric
        conversion (~25 ms at 640x480) unless want_depth -- the RTSP path doesn't
        need depth, so it renders rgb-only."""
        masking = self.mask_robot or bool(self.mask_ids)
        flags = 0 if masking else p.ER_NO_SEGMENTATION_MASK
        w, h, rgba, depth, seg = p.getCameraImage(
            self.width, self.height, view, self._proj, renderer=self._renderer,
            flags=flags)
        # this pybullet returns pixels as a Python tuple (no numpy build -> won't
        # compile against modern numpy). bytes()+frombuffer is 4-6x faster than
        # np.reshape on the tuple (both C-speed, zero-copy) and byte-identical.
        rgb = np.frombuffer(bytes(rgba), np.uint8).reshape(h, w, 4)[:, :, :3]
        if masking:                               # paint robot / occluders out
            seg = np.reshape(seg, (h, w)).astype(np.int32)
            m = np.isin(seg, list(self.mask_ids)) if self.mask_ids \
                else np.zeros(seg.shape, bool)
            if self.mask_robot:
                m |= seg == self.robot.body_id
            if m.any():
                env = rgb[(seg >= 0) & ~m]        # environment (terrain/foliage)
                fill = (np.median(env.reshape(-1, 3), 0).astype(np.uint8)
                        if len(env) else np.array([150, 140, 120], np.uint8))
                rgb = rgb.copy(); rgb[m] = fill   # blend, not a sky blob
        else:
            seg = np.full((h, w), -1, np.int32)
        metric = None
        if want_depth:
            zbuf = np.reshape(depth, (h, w)).astype(np.float32)
            metric = ((self.far * self.near) /
                      (self.far - (self.far - self.near) * zbuf)).astype(np.float32)
        return w, h, rgb, metric, seg

    def rgb_frame(self) -> np.ndarray:
        """Fast RGB-only render for streaming (no depth/metric conversion)."""
        _, _, _, view = self._view()
        return self._render(view, want_depth=False)[2]

    def _sample(self) -> CameraFrame:
        pos, orn, eye, view = self._view()
        w, h, rgb, metric, seg = self._render(view, want_depth=True)
        return CameraFrame(rgb=rgb, depth=metric, seg=seg,
                           width=w, height=h, pose=(tuple(pos), tuple(orn)),
                           intrinsics={"hfov_deg": self.hfov, "near": self.near,
                                       "far": self.far},
                           t=self.clock.now() if self.clock else 0.0)
