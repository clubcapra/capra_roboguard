"""Native Qt (Wayland) GUI for the ROVE sim -- a panel running PyBullet.

PyBullet runs headless (DIRECT + EGL GPU render); the rendered view is shown in
a Qt panel that talks Wayland directly (no X11/XWayland, no auth headaches).
Drag the view to orbit, scroll to zoom. Drive with the buttons; tune locomotion
and pose the arm/flippers with the live sliders.

    QT_QPA_PLATFORM=wayland PYTHONPATH=. ../rove_sim_venv/bin/python tools/gui.py --profile standard
"""
import argparse
import os
import sys
import numpy as np
import pybullet as p
from PySide6 import QtWidgets, QtCore, QtGui

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from rove_sim.core.engine import Engine, EngineConfig
from rove_sim.robot import loader
from rove_sim.robot.profile import load_profile
from rove_sim.robot.actuation import build_actuators
from rove_sim.control import RoveControl, Tracks, Flippers, Ovis
from rove_sim.world.mock import MockWorld
from rove_sim.world.friction import FrictionField, MATERIALS

# per-part colours (the GLB->OBJ meshes carry no texture, so they render black
# under one light). Same palette as render_clip.py.
_COL = [("Core", (.25, .27, .3)), ("cage", (.2, .4, .75)), ("DrumW", (.1, .1, .12)),
        ("Drum", (.15, .15, .17)), ("Flipper", (.85, .7, .1)), ("Base", (.8, .45, .1)),
        ("Section", (.8, .45, .1)), ("Joint", (.8, .45, .1)), ("robotiq", (.9, .5, .1)),
        ("knuckle", (.9, .5, .1)), ("finger", (.9, .5, .1)), ("mid360", (.1, .6, .2)),
        ("livox", (.1, .6, .2)), ("camera", (.4, .4, .42)), ("vn300", (.5, .12, .12))]


def _colorize_robot(robot):
    for n, idx in robot.link_index.items():
        for key, c in _COL:
            if key in n:
                p.changeVisualShape(robot.body_id, idx, rgbaColor=[*c, 1])
                break


class ViewPanel(QtWidgets.QLabel):
    """Shows the rendered frame. Drive view: drag orbits, wheel zooms. Paint view
    (top-down): drag paints the friction grid like a brush in an image editor."""

    def __init__(self):
        super().__init__()
        self.setMinimumSize(720, 540)
        self.setStyleSheet("background:#202028")
        self.yaw, self.pitch, self.dist = 50.0, -28.0, 2.2
        self.target = np.array([0.0, 0.2, 0.2])
        self._last = None
        # paint mode (top-down ortho); paint_cb(world_x, world_y) set by the GUI
        self.paint_mode = False
        self.paint_half = 8.0                # world half-height shown (m)
        self.paint_cb = None
        self._img_wh = (720, 540)            # last rendered image size

    def _paint_at(self, e):
        if self.paint_cb is None:
            return
        iw, ih = self._img_wh
        # label may letterbox the pixmap (KeepAspectRatio): find the shown rect
        lw, lh = self.width(), self.height()
        scale = min(lw / iw, lh / ih)
        ox, oy = (lw - iw * scale) / 2, (lh - ih * scale) / 2
        sx = (e.position().x() - ox) / scale
        sy = (e.position().y() - oy) / scale
        if not (0 <= sx <= iw and 0 <= sy <= ih):
            return
        aspect = iw / ih
        wx = self.target[0] + (2 * sx / iw - 1) * self.paint_half * aspect
        wy = self.target[1] + (1 - 2 * sy / ih) * self.paint_half
        self.paint_cb(wx, wy)

    def mousePressEvent(self, e):
        if self.paint_mode:
            self._paint_at(e)
        else:
            self._last = e.position()

    def mouseMoveEvent(self, e):
        if self.paint_mode:
            self._paint_at(e)
        elif self._last is not None:
            d = e.position() - self._last
            self.yaw -= d.x() * 0.4
            self.pitch = max(-89, min(-2, self.pitch - d.y() * 0.4))
            self._last = e.position()

    def mouseReleaseEvent(self, e):
        self._last = None

    def wheelEvent(self, e):
        f = 0.9 if e.angleDelta().y() > 0 else 1.1
        if self.paint_mode:
            self.paint_half = max(3.0, min(30.0, self.paint_half * f))
        else:
            self.dist = max(0.6, min(8.0, self.dist * f))

    def view_matrix(self):
        if self.paint_mode:
            t = self.target
            return p.computeViewMatrix([t[0], t[1], 25], [t[0], t[1], 0], [0, 1, 0])
        return p.computeViewMatrixFromYawPitchRoll(
            self.target.tolist(), self.dist, self.yaw, self.pitch, 0, 2)

    def proj_matrix(self, aspect):
        if self.paint_mode:
            h = self.paint_half
            return p.computeProjectionMatrix(-h * aspect, h * aspect, -h, h, 0.1, 100)
        return p.computeProjectionMatrixFOV(60, aspect, 0.05, 60)


class SimGUI(QtWidgets.QMainWindow):
    def __init__(self, profile_name, terrain=None):
        super().__init__()
        self.setWindowTitle(f"ROVE sim — {profile_name}")
        self.eng = Engine(EngineConfig(mode="headless")).connect()
        prof = load_profile(_prof_path(profile_name))
        # World seam owns gravity/ground now (engine.connect no longer does) and
        # carries the paintable friction field. Optional terrain backdrop.
        wspec = {"friction": {"origin": (-20.0, -20.0), "extent": (40.0, 40.0),
                              "cell": 0.25}}
        if terrain:
            wspec["terrain"] = {"source": terrain}      # hilly ground, robot drops on it
        self.world = MockWorld(self.eng, wspec, profile=prof).build()
        self.friction = self.world.friction
        self.robot = loader.load(self.eng, prof)
        # drop the robot onto the terrain surface at the origin (a flat road patch)
        self.spawn_z = 0.3
        if getattr(self.world, "terrain_id", None) is not None:
            zt = self.world.drop_point(0.0, 0.0)
            if zt is not None:
                self.spawn_z = zt + 0.5
                _, orn = p.getBasePositionAndOrientation(self.robot.body_id)
                p.resetBasePositionAndOrientation(self.robot.body_id,
                                                  [0.0, 0.0, self.spawn_z], orn)
        _colorize_robot(self.robot)
        self.acts = build_actuators(prof.actuators, self.robot)
        self.tracks = next((a for a in self.acts if a.intent_field == "tracks"), None)
        if self.tracks:
            self.tracks.friction_field = self.friction
        self.intent = RoveControl()
        self.flip = Flippers()
        self.ovis = Ovis()

        self.view = ViewPanel()
        self.view.target = np.array([0.0, 0.0, max(0.15, self.spawn_z - 0.3)])
        self.metrics = QtWidgets.QLabel("—")
        self.metrics.setStyleSheet("color:#cfc;font-family:monospace")
        self._build_ui()

        self.W, self.H = 600, 340
        self.render_w = 600            # render width (cost scales with pixels^2)
        # Physics and render run on SEPARATE timers: the GPU->CPU readback in
        # getCameraImage is slow (~70 ms), so blocking physics on it makes driving
        # lag. Physics ticks fast (near-realtime); the view refreshes independently.
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick_physics)
        self.timer.start(33)                     # ~30 Hz control (8 substeps = 240 Hz)
        self.rtimer = QtCore.QTimer(self)
        self.rtimer.timeout.connect(self.render_frame)
        self.rtimer.start(50)                    # ~20 fps view

    # -- UI ------------------------------------------------------------------
    def _build_ui(self):
        central = QtWidgets.QWidget()
        h = QtWidgets.QHBoxLayout(central)
        h.addWidget(self.view, 3)
        side = QtWidgets.QVBoxLayout()
        h.addLayout(side, 1)

        # drive buttons
        side.addWidget(_h("Drive"))
        grid = QtWidgets.QGridLayout()
        for txt, lv, rv, r, c in [("Fwd", 1, 1, 0, 1), ("Rev", -1, -1, 2, 1),
                                  ("Pivot L", 1, -1, 1, 0), ("Pivot R", -1, 1, 1, 2),
                                  ("Stop", 0, 0, 1, 1)]:
            b = QtWidgets.QPushButton(txt)
            b.pressed.connect(lambda lv=lv, rv=rv: self._set_tracks(lv, rv))
            if txt != "Stop":
                b.released.connect(lambda: self._set_tracks(0, 0))
            grid.addWidget(b, r, c)
        side.addLayout(grid)

        # locomotion sliders (brush-friction model)
        side.addWidget(_h("Locomotion (live)"))
        self.s_belt = self._slider(side, "belt speed (rad/s)", 10, 60,
                                   int(self.tracks.max_rad_s) if self.tracks else 46)
        self.s_mul = self._slider(side, "mu longitudinal x100", 20, 120,
                                  int(self.tracks.mu_long * 100) if self.tracks else 60)
        self.s_mut = self._slider(side, "mu lateral x100 (low=easy turn)", 3, 80,
                                  int(self.tracks.mu_lat * 100) if self.tracks else 15)

        # flipper / arm pose
        side.addWidget(_h("Flippers"))
        fl = QtWidgets.QHBoxLayout()
        for t, key in [("Up", +1), ("Down", -1), ("Flat", 0)]:
            b = QtWidgets.QPushButton(t)
            b.clicked.connect(lambda _, k=key: self._flippers(k))
            fl.addWidget(b)
        side.addLayout(fl)
        if any(a.intent_field == "ovis" for a in self.acts):
            side.addWidget(_h("Arm (hold to move)"))
            ag = QtWidgets.QGridLayout()
            for t, ax, sgn, r, c in [("Up", "vz", 1, 0, 1), ("Dn", "vz", -1, 2, 1),
                                     ("Fwd", "vx", 1, 1, 0), ("Bk", "vx", -1, 1, 2),
                                     ("Yaw", "wz", 1, 1, 1)]:
                b = QtWidgets.QPushButton(t)
                b.pressed.connect(lambda ax=ax, sgn=sgn: setattr(self.ovis, ax, float(sgn)))
                b.released.connect(lambda ax=ax: setattr(self.ovis, ax, 0.0))
                ag.addWidget(b, r, c)
            side.addLayout(ag)

        # view quality (render width; lower = faster, the readback is pixel-bound)
        side.addWidget(_h("View"))
        self.s_res = self._slider(side, "render width (px)  lower=faster", 360, 1280, 600)
        self.s_res.valueChanged.connect(
            lambda v: setattr(self, "render_w", int(v)))

        # friction painting (top-down brush, like an image editor)
        side.addWidget(_h("Friction paint"))
        self.paint_chk = QtWidgets.QCheckBox("Paint mode (top-down)")
        self.paint_chk.toggled.connect(self._toggle_paint)
        side.addWidget(self.paint_chk)
        self.mat = QtWidgets.QComboBox()
        self.mat.addItems(list(MATERIALS.keys()))
        self.mat.setCurrentText("ice")
        side.addWidget(self.mat)
        self.s_brush = self._slider(side, "brush radius x10 (m)", 3, 30, 8)
        prow = QtWidgets.QHBoxLayout()
        for t, fn in [("Clear", self._clear_friction), ("Save", self._save_friction),
                      ("Load", self._load_friction)]:
            b = QtWidgets.QPushButton(t); b.clicked.connect(fn); prow.addWidget(b)
        side.addLayout(prow)
        self.view.paint_cb = self._paint

        side.addStretch(1)
        side.addWidget(self.metrics)
        self.setCentralWidget(central)
        self.resize(1080, 620)

    def _slider(self, layout, label, lo, hi, val):
        layout.addWidget(QtWidgets.QLabel(label))
        s = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        s.setRange(lo, hi); s.setValue(val)
        layout.addWidget(s)
        return s

    # -- control -------------------------------------------------------------
    def _set_tracks(self, lv, rv):
        self.intent.tracks = Tracks(float(lv), float(rv))

    def _flippers(self, k):
        self.flip = Flippers(k, k, k, k)

    # -- friction painting ---------------------------------------------------
    def _toggle_paint(self, on):
        self.view.paint_mode = bool(on)
        if on:                       # centre the top-down view on the robot
            pos = p.getBasePositionAndOrientation(self.robot.body_id)[0]
            self.view.target = np.array([pos[0], pos[1], 0.0])

    def _paint(self, wx, wy):
        self.friction.paint_material(wx, wy, self.s_brush.value() / 10.0,
                                     self.mat.currentText())

    def _clear_friction(self):
        self.friction.mu[:] = self.friction.default

    def _save_friction(self):
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save friction",
                                                        "friction.json", "*.json")
        if path:
            self.friction.save(path)

    def _load_friction(self):
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "Load friction",
                                                        "", "*.json")
        if path:
            loaded = FrictionField.load(path)
            self.friction.mu = loaded.mu            # keep the same object the actuator holds
            self.friction.nx, self.friction.ny = loaded.nx, loaded.ny
            self.friction.x0, self.friction.y0 = loaded.x0, loaded.y0
            self.friction.cell, self.friction.default = loaded.cell, loaded.default

    @staticmethod
    def _mu_qcolor(mu, default):
        if mu < default:                       # low grip -> icy blue
            t = mu / default
            return QtGui.QColor(int(120 + 80 * t), int(170 + 60 * t), 255, 150)
        t = min(1.0, (mu - default) / 0.4)     # high grip -> gravel green
        return QtGui.QColor(int(120 - 60 * t), int(180 + 40 * t), int(90 - 40 * t), 150)

    def _draw_overlay(self, img):
        f, t = self.friction, self.view.target
        half = self.view.paint_half
        iw, ih = self.W, self.H
        aspect = iw / ih
        cpx = f.cell / (2 * half) * ih
        qp = QtGui.QPainter(img)
        for j in range(f.ny):
            for i in range(f.nx):
                mu = float(f.mu[j, i])
                if abs(mu - f.default) < 1e-3:
                    continue
                wx, wy = f.cell_center(i, j)
                if abs(wx - t[0]) > half * aspect or abs(wy - t[1]) > half:
                    continue
                sx = ((wx - t[0]) / (half * aspect) + 1) / 2 * iw
                sy = (1 - (wy - t[1]) / half) / 2 * ih
                qp.fillRect(QtCore.QRectF(sx - cpx / 2, sy - cpx / 2, cpx, cpx),
                            self._mu_qcolor(mu, f.default))
        rp = p.getBasePositionAndOrientation(self.robot.body_id)[0]
        sx = ((rp[0] - t[0]) / (half * aspect) + 1) / 2 * iw
        sy = (1 - (rp[1] - t[1]) / half) / 2 * ih
        qp.setPen(QtGui.QPen(QtGui.QColor(255, 60, 60), 3))
        qp.drawEllipse(QtCore.QPointF(sx, sy), 10, 10)
        qp.end()

    def _apply_live(self):
        if not self.tracks:
            return
        self.tracks.max_rad_s = float(self.s_belt.value())
        self.tracks.v_max = self.tracks.max_rad_s * self.tracks.drum_radius
        self.tracks.mu_long = self.s_mul.value() / 100.0
        self.tracks.mu_lat = self.s_mut.value() / 100.0

    # -- loop ----------------------------------------------------------------
    def tick_physics(self):
        self._apply_live()
        intent = RoveControl(tracks=self.intent.tracks, flippers=self.flip,
                             ovis=self.ovis)
        for a in self.acts:
            a.apply(intent)
        self.flip = Flippers()                  # flipper step is one-shot per click
        for _ in range(8):                      # 8 physics steps / frame (240 Hz)
            for a in self.acts:                 # brush forces need per-step re-apply
                a.step(intent)
            self.eng.step(1)
        pos, orn = p.getBasePositionAndOrientation(self.robot.body_id)
        ro, pi, _ = p.getEulerFromQuaternion(orn)
        v, _ = p.getBaseVelocity(self.robot.body_id)
        self.metrics.setText(
            f"speed {np.linalg.norm(v[:2])*3.6:4.1f} km/h\n"
            f"roll {np.degrees(ro):+5.1f}  pitch {np.degrees(pi):+5.1f} deg")

    def render_frame(self):
        # Render width is CAPPED (getCameraImage's GPU->CPU readback is pixel-
        # bound: ~70 ms @960w, ~310 ms @1920w). Qt scales the result to the panel.
        aspect = max(0.2, self.view.width() / max(1, self.view.height()))
        pw = self.render_w
        ph = max(240, int(pw / aspect))
        self.W, self.H = pw, ph
        self.view._img_wh = (pw, ph)
        proj = self.view.proj_matrix(pw / ph)
        _, _, rgb, _, _ = p.getCameraImage(
            pw, ph, self.view.view_matrix(), proj,
            renderer=self.eng.camera_renderer_flag, lightDirection=[-1, -1, 2],
            shadow=0, flags=p.ER_NO_SEGMENTATION_MASK)
        img = QtGui.QImage(np.reshape(rgb, (ph, pw, 4)).astype(np.uint8).tobytes(),
                           pw, ph, QtGui.QImage.Format_RGBA8888).copy()
        if self.view.paint_mode:
            self._draw_overlay(img)
        self.view.setPixmap(QtGui.QPixmap.fromImage(img).scaled(
            self.view.size(), QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation))

    def closeEvent(self, e):
        self.timer.stop()
        self.rtimer.stop()
        self.eng.disconnect()


def _h(t):
    lbl = QtWidgets.QLabel(t)
    lbl.setStyleSheet("font-weight:bold;color:#9cf;margin-top:6px")
    return lbl


def _prof_path(name):
    here = os.path.join(os.path.dirname(__file__), "..", "profiles", f"{name}.yaml")
    return here if os.path.exists(here) else name


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", default="standard")
    ap.add_argument("--terrain", nargs="?",
                    const="../free_dirt_road_through_forest.glb", default=None,
                    help="load a terrain backdrop (GLB); bare flag uses the "
                         "forest dirt-road, or pass a path")
    args = ap.parse_args()
    os.environ.setdefault("QT_QPA_PLATFORM", "wayland")
    app = QtWidgets.QApplication(sys.argv)
    gui = SimGUI(args.profile, terrain=args.terrain)
    gui.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
