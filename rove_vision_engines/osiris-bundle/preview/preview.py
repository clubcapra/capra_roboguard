#!/usr/bin/env python3
"""
Osiris bundle preview tool — a tiny, zero-dependency web UI to create feeds and
preview them visually (live video + detection / track overlays).

It serves:
  GET /            -> the UI (the browser talks to the Osiris API directly;
                      the controller has permissive CORS)
  GET /mjpeg?src=  -> a live MJPEG stream of a source via ffmpeg, so the browser
                      can show RTSP / USB / file / webrtc:// video it can't play
                      natively. Detection boxes are drawn client-side on a canvas
                      from the controller's WebSocket.

Only the Python standard library + ffmpeg are required (ffmpeg already ships as a
bundle runtime dependency).

Config via env (set by preview.sh from config.env):
  OSIRIS_API     controller REST/WS base       (default http://localhost:9090)
  OSIRIS_RTSP    gateway RTSP base for webrtc:// (default rtsp://127.0.0.1:8554)
  PREVIEW_BIND   host:port to serve on          (default 0.0.0.0:8080)
"""

import os
import subprocess
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

API = os.environ.get("OSIRIS_API", "http://localhost:9090").rstrip("/")
RTSP = os.environ.get("OSIRIS_RTSP", "rtsp://127.0.0.1:8554").rstrip("/")
BIND = os.environ.get("PREVIEW_BIND", "0.0.0.0:8080")
HERE = os.path.dirname(os.path.abspath(__file__))
BOUNDARY = "osirisframe"


def _api_port():
    return urllib.parse.urlparse(API).port or 9090


def ffmpeg_cmd(src):
    """Build the ffmpeg command to decode a source into an MJPEG byte stream."""
    cmd = ["ffmpeg", "-hide_banner", "-loglevel", "error"]
    if src.startswith("webrtc://"):
        src = f"{RTSP}/{src[len('webrtc://'):].lstrip('/')}"
    if src.startswith("rtsp://"):
        cmd += ["-rtsp_transport", "tcp", "-fflags", "nobuffer", "-flags", "low_delay"]
    elif src.startswith("/dev/video"):
        cmd += ["-f", "v4l2"]
    cmd += ["-i", src, "-an", "-r", "12",
            "-f", "image2pipe", "-vcodec", "mjpeg", "-q:v", "6", "-"]
    return cmd


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):  # quiet
        pass

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            self._serve_index()
        elif parsed.path == "/mjpeg":
            qs = urllib.parse.parse_qs(parsed.query)
            src = qs.get("src", [""])[0]
            if not src:
                self.send_error(400, "src required")
                return
            self._stream_mjpeg(src)
        else:
            self.send_error(404)

    def _serve_index(self):
        with open(os.path.join(HERE, "index.html"), "rb") as f:
            html = f.read()
        html = (html
                .replace(b"__API_PORT__", str(_api_port()).encode())
                .replace(b"__RTSP_BASE__", RTSP.encode()))
        self.send_response(200)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(html)))
        self.end_headers()
        self.wfile.write(html)

    def _stream_mjpeg(self, src):
        try:
            proc = subprocess.Popen(
                ffmpeg_cmd(src), stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
            )
        except Exception as e:  # noqa: BLE001
            self.send_error(500, str(e))
            return

        self.send_response(200)
        self.send_header(
            "Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}"
        )
        self.send_header("Cache-Control", "no-cache, no-store")
        self.end_headers()

        # Re-frame ffmpeg's MJPEG output into multipart parts. In baseline JPEG
        # 0xFFD9 only appears as the real EOI marker (0xFF in data is byte-stuffed
        # with 0x00), so the SOI/EOI scan is safe.
        buf = b""
        try:
            while True:
                chunk = proc.stdout.read(65536)
                if not chunk:
                    break
                buf += chunk
                while True:
                    start = buf.find(b"\xff\xd8")
                    end = buf.find(b"\xff\xd9", start + 2) if start != -1 else -1
                    if start == -1 or end == -1:
                        break
                    jpg = buf[start:end + 2]
                    buf = buf[end + 2:]
                    self.wfile.write(b"--" + BOUNDARY.encode() + b"\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n"
                    )
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass
        finally:
            proc.kill()


def main():
    host, _, port = BIND.rpartition(":")
    host = host or "0.0.0.0"
    server = ThreadingHTTPServer((host, int(port)), Handler)
    print(f"Osiris preview UI on http://{host}:{port}  (Osiris API: {API})", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
