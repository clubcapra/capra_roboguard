"""Camera -> RTSP: serve each camera sensor's frames as an H.264 RTSP stream.

The autonomy/vision stack (rove_vision_engine) consumes the real cameras over
RTSP; this makes the SIM cameras look identical -- one rtsp:// URL per mount --
so the vision model can't tell sim from real. PyBullet has no RTSP, so we shell
out: a single `mediamtx` RTSP server (one static binary, tools/bin/mediamtx) plus
one `ffmpeg` per camera that takes raw RGB frames on stdin, H.264-encodes them and
publishes to the server. Frames are pushed from the sim's render loop via push();
a writer thread paces them to ffmpeg at a fixed fps (repeating the last frame if
the sim is slow) so clients get a steady CFR stream and the sim never blocks.
"""
from __future__ import annotations

import os
import shutil
import subprocess
import tempfile
import threading
import time
from typing import Optional

import numpy as np

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(__file__)))
_MEDIAMTX = os.path.join(_ROOT, "tools", "bin", "mediamtx")


class RtspServer:
    """Wraps the mediamtx RTSP server process (catch-all paths)."""

    def __init__(self, port: int = 8554, bin_path: Optional[str] = None):
        self.port = int(port)
        self.bin = bin_path or _MEDIAMTX
        self.proc: Optional[subprocess.Popen] = None
        self._cfg: Optional[str] = None

    @property
    def available(self) -> bool:
        return os.path.exists(self.bin) and bool(shutil.which("ffmpeg"))

    def start(self) -> "RtspServer":
        if self.proc is not None or not os.path.exists(self.bin):
            return self
        fd, self._cfg = tempfile.mkstemp(suffix=".yml", prefix="mediamtx_")
        with os.fdopen(fd, "w") as f:
            # TCP-only RTSP: avoids binding the shared UDP RTP/RTCP ports (8000/
            # 8001) so multiple/respawned servers don't clash on them.
            f.write(f"rtspAddress: :{self.port}\nrtspTransports: [tcp]\n"
                    f"rtmp: no\nhls: no\nwebrtc: no\nsrt: no\n"
                    f"logLevel: error\npaths:\n  all_others:\n")
        self.proc = subprocess.Popen([self.bin, self._cfg],
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
        time.sleep(1.0)                         # let it bind the port
        return self

    def stop(self) -> None:
        if self.proc is not None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=2)
            except Exception:
                self.proc.kill()
            self.proc = None
        if self._cfg and os.path.exists(self._cfg):
            os.remove(self._cfg)


class RtspStream:
    """One camera -> one rtsp://host:port/<name> H.264 stream."""

    def __init__(self, name: str, width: int, height: int, fps: float = 15.0,
                 host: str = "127.0.0.1", port: int = 8554,
                 scale: Optional[tuple] = None, encoder: str = "libx264",
                 bitrate: str = "2M"):
        self.name = name
        self.url = f"rtsp://{host}:{port}/{name}"
        self.fps = float(fps)
        vf = ["-vf", f"scale={scale[0]}:{scale[1]}"] if scale else []
        enc = (["-c:v", "h264_nvenc", "-preset", "p1", "-tune", "ll"]
               if encoder == "h264_nvenc" else
               ["-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency"])
        cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error", "-y",
               "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
               "-r", f"{self.fps:g}", "-i", "-", *vf, *enc,
               "-pix_fmt", "yuv420p", "-b:v", bitrate, "-g", str(int(self.fps * 2)),
               "-f", "rtsp", "-rtsp_transport", "tcp", self.url]
        self.proc = subprocess.Popen(cmd, stdin=subprocess.PIPE,
                                     stdout=subprocess.DEVNULL,
                                     stderr=subprocess.DEVNULL)
        self._frame: Optional[bytes] = None
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._writer, daemon=True)
        self._t.start()

    def push(self, rgb: np.ndarray) -> None:
        """Hand the latest RGB frame (H,W,3 uint8) to the encoder thread."""
        if rgb is not None:
            with self._lock:
                self._frame = np.ascontiguousarray(rgb, np.uint8).tobytes()

    def _writer(self) -> None:
        period = 1.0 / self.fps
        nxt = time.time()
        while not self._stop.is_set():
            with self._lock:
                frame = self._frame
            if frame is not None and self.proc.stdin is not None:
                try:
                    self.proc.stdin.write(frame)        # repeats last frame -> CFR
                except (BrokenPipeError, ValueError):
                    break
            nxt += period
            time.sleep(max(0.0, nxt - time.time()))

    def stop(self) -> None:
        self._stop.set()
        try:
            self._t.join(timeout=1)
            if self.proc.stdin:
                self.proc.stdin.close()
            self.proc.terminate()
            self.proc.wait(timeout=2)
        except Exception:
            try:
                self.proc.kill()
            except Exception:
                pass


class RtspCameraFeeds:
    """Ties a set of Camera sensors to RTSP streams. Call publish() each frame
    (it rate-gates internally to the stream fps and renders only then)."""

    def __init__(self, cameras, fps: float = 15.0, port: int = 8554,
                 scale: Optional[tuple] = None, encoder: str = "libx264",
                 render_fps: float = 20.0, manage_server: bool = True):
        self.fps = float(fps)
        # when False, push to an EXISTING mediamtx (one shared server for many
        # workers -- multiple mediamtx clash on their UDP RTP/MoQ ports).
        self.manage_server = bool(manage_server)
        # TOTAL camera renders/sec budget, shared round-robin across the cameras
        # (one EGL readback per tick stalls the single-threaded loop, so we never
        # render all of them at once). Each camera's source rate = render_fps/N;
        # the writer threads still emit `fps` CFR by repeating the last frame.
        self.render_fps = float(render_fps)
        self.server = RtspServer(port=port)
        self.streams: dict = {}
        self.cameras = list(cameras)
        self._port, self._scale, self._encoder = port, scale, encoder
        self._accum = 0.0
        self._rr = 0                              # round-robin cursor

    def start(self) -> "RtspCameraFeeds":
        if not self.cameras or (self.manage_server and not self.server.available):
            return self
        if self.manage_server:
            self.server.start()
        for c in self.cameras:
            self.streams[c.name] = RtspStream(
                c.name, c.width, c.height, fps=self.fps, port=self._port,
                scale=self._scale, encoder=self._encoder)
        return self

    def urls(self) -> list:
        return [s.url for s in self.streams.values()]

    def publish(self, dt: float) -> None:
        """Render + push ONE camera per render tick (round-robin), so a single
        EGL readback (~tens of ms) never stalls the loop with N back-to-back."""
        if not self.streams:
            return
        self._accum += dt
        if self._accum < 1.0 / self.render_fps:
            return
        self._accum = 0.0
        c = self.cameras[self._rr % len(self.cameras)]
        self._rr += 1
        s = self.streams.get(c.name)
        if s is not None:
            s.push(c.rgb_frame())                 # rgb-only fast path (no depth)

    def stop(self) -> None:
        for s in self.streams.values():
            s.stop()
        if self.manage_server:
            self.server.stop()
        self.streams.clear()
