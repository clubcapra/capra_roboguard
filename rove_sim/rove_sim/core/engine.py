"""PyBullet engine wrapper: connection mode + headless GPU rendering.

Two orthogonal concerns the rest of the sim should never have to think about:

  * connection mode -- GUI (dev, interactive window) vs DIRECT (headless).
  * camera rendering -- on a headless box there is no GL context, so camera
    sensors must go through the EGL plugin (GPU offscreen) or fall back to the
    CPU TinyRenderer. We load EGL once here and record which renderer is live.

The robot loader, sensors and scheduler all take an Engine, not a raw pybullet
client id.
"""
from __future__ import annotations

import glob
import os
from dataclasses import dataclass

import pybullet as p
import pybullet_data


def _ensure_x_auth() -> str | None:
    """Point XAUTHORITY at a valid X cookie for the GUI on Wayland/XWayland.

    PyBullet's GUI is an X11 client; on a Wayland desktop it talks to XWayland,
    which requires the MIT-MAGIC-COOKIE. If XAUTHORITY isn't set (common in a
    plain shell) the connection is refused with "Authorization required". We
    locate the session's cookie and export it. Returns the file used, or None.
    """
    cur = os.environ.get("XAUTHORITY")
    if cur and os.path.exists(cur):
        return cur
    uid = os.getuid()
    run = os.environ.get("XDG_RUNTIME_DIR") or f"/run/user/{uid}"
    candidates = [os.path.expanduser("~/.Xauthority"),
                  *sorted(glob.glob(f"{run}/.mutter-Xwaylandauth.*")),  # GNOME
                  *sorted(glob.glob(f"{run}/xauth_*")),                 # KDE/sddm
                  *sorted(glob.glob(f"{run}/.Xauthority")),
                  f"{run}/gdm/Xauthority"]
    for c in candidates:
        if c and os.path.exists(c):
            os.environ["XAUTHORITY"] = c
            return c
    return None


def _ensure_display() -> None:
    """Pick a DISPLAY if unset. Modern XWayland often uses ABSTRACT sockets that
    don't appear in /tmp/.X11-unix, so also fall back to the common :0/:1."""
    if os.environ.get("DISPLAY"):
        return
    socks = sorted(glob.glob("/tmp/.X11-unix/X*"))
    if socks:
        os.environ["DISPLAY"] = ":" + os.path.basename(socks[0])[1:]
        return
    # abstract-socket fallback: assume the usual XWayland display. If it isn't
    # actually live, p.connect(GUI) fails with a clear "cannot connect" message.
    os.environ["DISPLAY"] = ":0"


def _ensure_egl_vendor() -> str | None:
    """Point glvnd at an NVIDIA EGL vendor ICD if one isn't already visible.

    On this box (and injected-driver sandboxes generally) the NVIDIA glvnd
    egl_vendor.d lives under a non-standard prefix, so libEGL's default search
    path finds 0 EGL devices. We locate a versioned nvidia ICD dir (which
    exposes exactly the GPU as a surfaceless EGL device) and export
    __EGL_VENDOR_LIBRARY_DIRS before the eglRenderer plugin initialises EGL.
    Version-agnostic: globbed, never hardcoded. Returns the dir used, or None.
    """
    if os.environ.get("__EGL_VENDOR_LIBRARY_DIRS"):
        return os.environ["__EGL_VENDOR_LIBRARY_DIRS"]
    cands = sorted(glob.glob("/usr/lib/*/GL/nvidia-*/glvnd/egl_vendor.d"),
                   reverse=True)                      # prefer newest driver
    cands += sorted(glob.glob("/usr/lib/*/GL/glvnd/egl_vendor.d"))
    for d in cands:
        if glob.glob(os.path.join(d, "*nvidia*.json")):
            os.environ["__EGL_VENDOR_LIBRARY_DIRS"] = d
            return d
    return None


@dataclass
class EngineConfig:
    mode: str = "headless"          # "gui" | "headless"
    gravity: float = -9.81
    # 240 Hz: must run realtime headless on a Jetson Orin, so the rate stays low
    # and drum speed is capped (see tracks max_track_rad_s) to keep the contact
    # solver stable -- fast drums bounce/lift at this rate otherwise. (S5.7)
    timestep: float = 1.0 / 240.0
    real_time: bool = False         # let pybullet free-run the GUI clock
    ground_plane: bool = True
    # hardwood-floor friction for the ground plane (rubber-on-wood ~0.6). The
    # brush-friction tracks supply their own anisotropic mu at the tread
    # contacts; this is the floor seen by the body, flippers-on-edges, obstacles.
    floor_friction: float = 0.6
    egl: bool = True                # try EGL for headless GPU camera render
    # Constraint-solver iterations per step. Bullet's default is 50; this scene
    # is contact-rich but low-stiffness (compliant rubber treads, no stiff
    # closed loops), so it converges well below that. 20 cuts physics cost ~1.5x
    # vs 50 with *no* measurable change to drive distance, roll, pitch, pivot
    # drift, or point-turn yaw -- headroom for native 60 fps and the Jetson.
    # (Going lower, e.g. 10, weakens the already-marginal narrow-gauge point-turn
    # below its test threshold; 20 is the safe floor.) The brush model applies its
    # forces per physics step regardless of this.
    solver_iterations: int = 20


class Engine:
    def __init__(self, cfg: EngineConfig | None = None):
        self.cfg = cfg or EngineConfig()
        self.client: int | None = None
        self.plane_id: int | None = None
        self.renderer: str = "none"     # "egl" | "tiny" | "gui"
        self.egl_vendor_dir: str | None = None
        self._egl_plugin: int | None = None

    # -- lifecycle -----------------------------------------------------------
    def connect(self) -> "Engine":
        if self.cfg.mode == "gui":
            # pybullet's GUI is an X11/GLX client; on Wayland it goes through
            # XWayland and needs DISPLAY + the session's auth cookie.
            _ensure_display()
            xauth = _ensure_x_auth()
            if not os.environ.get("DISPLAY"):
                raise RuntimeError(
                    "No DISPLAY / X server found. Open a terminal inside the "
                    "graphical desktop session (not a plain TTY/SSH).")
            self.client = p.connect(p.GUI)
            if self.client < 0:
                raise RuntimeError(
                    "GUI connect refused (DISPLAY=%s XAUTHORITY=%s). If it says "
                    "'Authorization required', run `xhost +SI:localuser:$USER` "
                    "once, then retry." % (os.environ.get("DISPLAY"), xauth))
            self.renderer = "gui"
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 1)
        elif self.cfg.mode == "headless":
            self.client = p.connect(p.DIRECT)
            self._load_camera_renderer()
        else:
            raise ValueError(f"unknown engine mode {self.cfg.mode!r}")

        # Connection + renderer ONLY. World setup (gravity, timestep, ground
        # plane, friction) is owned by the World seam (rove_sim/world/) so the
        # same engine connection serves both the mock-physics world and the
        # real/kinematic world model. We keep the data-path here because the
        # MockWorld loads pybullet_data's plane.urdf.
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        return self

    def _load_camera_renderer(self) -> None:
        """DIRECT has no GL context; pick the best offscreen renderer."""
        if self.cfg.egl:
            self.egl_vendor_dir = _ensure_egl_vendor()
            try:
                import importlib.util
                spec = importlib.util.find_spec("eglRenderer")
                fname = spec.origin if spec else None
                # entry-point symbol is "_eglRendererPlugin" (leading
                # underscore); the wrong name loads the .so but "couldn't bind
                # functions" and silently falls back to CPU.
                self._egl_plugin = (p.loadPlugin(fname, "_eglRendererPlugin")
                                    if fname else
                                    p.loadPlugin("eglRendererPlugin"))
                if self._egl_plugin is not None and self._egl_plugin >= 0:
                    self.renderer = "egl"
                    return
            except Exception:
                pass
        # fall back: software rasteriser, always available
        self.renderer = "tiny"

    @property
    def camera_renderer_flag(self) -> int:
        """Pass to getCameraImage(renderer=...)."""
        if self.renderer in ("egl", "gui"):
            return p.ER_BULLET_HARDWARE_OPENGL
        return p.ER_TINY_RENDERER

    def step(self, n: int = 1) -> None:
        for _ in range(n):
            p.stepSimulation()

    def disconnect(self) -> None:
        if self._egl_plugin is not None and self._egl_plugin >= 0:
            try:
                p.unloadPlugin(self._egl_plugin)
            except Exception:
                pass
            self._egl_plugin = None
        if self.client is not None:
            try:
                p.disconnect()
            except Exception:
                pass
            self.client = None

    def __enter__(self) -> "Engine":
        return self.connect()

    def __exit__(self, *exc) -> None:
        self.disconnect()
