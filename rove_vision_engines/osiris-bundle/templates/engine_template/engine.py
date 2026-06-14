"""
Osiris Vision Engine Template

Base class implementing the Template Design Pattern for vision engines.
All engines must subclass OsirisEngine and implement initialize() and process_frame().

Communication with the Rust orchestrator uses Unix Domain Sockets with a binary
framing protocol:
    [4 bytes: payload length (big-endian u32)] [1 byte: type tag] [payload]

Type tags:
    0x01 = control message (UTF-8 JSON)
    0x02 = frame data (36-byte feed_id + raw BGR bytes)
"""

import abc
import json
import os
import signal
import socket
import struct
import sys
import time
import traceback

MSG_CONTROL = 0x01
MSG_FRAME = 0x02

FEED_ID_LEN = 36


class OsirisEngine(abc.ABC):
    """Abstract base class for all Osiris vision engines."""

    def __init__(self):
        self._running = False
        self._feed_configs = {}

    @abc.abstractmethod
    def initialize(self, config: dict) -> dict:
        """
        Load the model and return metadata.

        Args:
            config: Dictionary from manifest.toml [model] and [inference] sections.

        Returns:
            dict with at least:
                - "classes": list[str] — class names the model detects
                - "input_size": list[int] — [width, height] expected by the model
        """
        ...

    @abc.abstractmethod
    def process_frame(
        self,
        feed_id: str,
        frame: bytes,
        width: int,
        height: int,
        channels: int,
    ) -> list[dict]:
        """
        Run inference on a single raw BGR frame.

        Args:
            feed_id: UUID of the feed this frame belongs to.
            frame: Raw BGR pixel data (width * height * channels bytes).
            width: Frame width in pixels.
            height: Frame height in pixels.
            channels: Number of color channels (typically 3 for BGR).

        Returns:
            List of detection dicts, each containing:
                - "class": str — detected class name
                - "confidence": float — detection confidence [0, 1]
                - "bbox": list[float] — [x, y, width, height] in pixels
                - "track_id": optional int — tracking ID if available
        """
        ...

    def get_metadata(self) -> dict:
        """Override to provide additional metadata beyond what initialize() returns."""
        return {}

    def on_shutdown(self):
        """Override to perform cleanup before the engine process exits."""
        pass

    # ── Protocol Implementation ──────────────────────────────────────────

    def serve(self, socket_path: str):
        """
        Main entry point. Binds a Unix domain socket, accepts a connection
        from the Rust orchestrator, and processes messages in a loop.
        """
        self._running = True
        signal.signal(signal.SIGTERM, self._handle_signal)
        signal.signal(signal.SIGINT, self._handle_signal)

        if os.path.exists(socket_path):
            os.unlink(socket_path)

        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.bind(socket_path)
        sock.listen(1)
        sock.settimeout(1.0)

        # Signal readiness by writing a marker file
        ready_path = socket_path + ".ready"
        with open(ready_path, "w") as f:
            f.write(str(os.getpid()))

        conn = None
        try:
            # Wait for orchestrator connection
            while self._running:
                try:
                    conn, _ = sock.accept()
                    conn.settimeout(5.0)
                    break
                except socket.timeout:
                    continue

            if conn is None:
                return

            self._message_loop(conn)

        except Exception:
            traceback.print_exc()
        finally:
            if conn:
                conn.close()
            sock.close()
            if os.path.exists(socket_path):
                os.unlink(socket_path)
            if os.path.exists(ready_path):
                os.unlink(ready_path)
            self.on_shutdown()

    def _message_loop(self, conn: socket.socket):
        """Read and dispatch messages from the orchestrator."""
        while self._running:
            try:
                msg_type, payload = self._recv_message(conn)
            except (ConnectionResetError, BrokenPipeError):
                break
            except socket.timeout:
                continue
            except Exception:
                traceback.print_exc()
                break

            if msg_type is None:
                break

            try:
                if msg_type == MSG_CONTROL:
                    response = self._handle_control(payload)
                    if response is not None:
                        self._send_control(conn, response)
                elif msg_type == MSG_FRAME:
                    response = self._handle_frame(payload)
                    self._send_control(conn, response)
            except Exception as e:
                traceback.print_exc()
                error_resp = {"error": str(e)}
                try:
                    self._send_control(conn, error_resp)
                except Exception:
                    break

    def _handle_control(self, payload: bytes) -> dict | None:
        """Dispatch a control message."""
        msg = json.loads(payload.decode("utf-8"))
        cmd = msg.get("cmd")

        if cmd == "initialize":
            config = msg.get("config", {})
            metadata = self.initialize(config)
            extra = self.get_metadata()
            if extra:
                metadata.update(extra)
            return {"status": "ready", "metadata": metadata}

        elif cmd == "configure_feed":
            feed_id = msg["feed_id"]
            self._feed_configs[feed_id] = {
                "width": msg["width"],
                "height": msg["height"],
                "channels": msg.get("channels", 3),
            }
            return {"status": "ok", "feed_id": feed_id}

        elif cmd == "remove_feed":
            feed_id = msg["feed_id"]
            self._feed_configs.pop(feed_id, None)
            return {"status": "ok", "feed_id": feed_id}

        elif cmd == "get_metadata":
            metadata = self.get_metadata()
            return {"metadata": metadata}

        elif cmd == "shutdown":
            self._running = False
            return {"status": "shutdown"}

        else:
            return {"error": f"unknown command: {cmd}"}

    def _handle_frame(self, payload: bytes) -> dict:
        """Extract feed_id and frame data, run inference."""
        feed_id = payload[:FEED_ID_LEN].decode("utf-8")
        frame_data = payload[FEED_ID_LEN:]

        feed_cfg = self._feed_configs.get(feed_id)
        if feed_cfg is None:
            return {"error": f"unknown feed: {feed_id}", "feed_id": feed_id}

        start = time.monotonic()
        detections = self.process_frame(
            feed_id,
            frame_data,
            feed_cfg["width"],
            feed_cfg["height"],
            feed_cfg["channels"],
        )
        inference_ms = (time.monotonic() - start) * 1000

        return {
            "feed_id": feed_id,
            "detections": detections,
            "inference_ms": round(inference_ms, 2),
        }

    # ── Wire Protocol ────────────────────────────────────────────────────

    @staticmethod
    def _recv_exact(conn: socket.socket, n: int) -> bytes | None:
        """Read exactly n bytes from the socket."""
        data = bytearray()
        while len(data) < n:
            chunk = conn.recv(n - len(data))
            if not chunk:
                return None
            data.extend(chunk)
        return bytes(data)

    def _recv_message(self, conn: socket.socket) -> tuple[int | None, bytes | None]:
        """Read one length-prefixed message. Returns (type_tag, payload) or (None, None)."""
        header = self._recv_exact(conn, 5)  # 4 bytes length + 1 byte type
        if header is None:
            return None, None

        length = struct.unpack(">I", header[:4])[0]
        msg_type = header[4]

        if length == 0:
            return msg_type, b""

        payload = self._recv_exact(conn, length)
        if payload is None:
            return None, None

        return msg_type, payload

    @staticmethod
    def _send_control(conn: socket.socket, msg: dict):
        """Send a control (JSON) message."""
        payload = json.dumps(msg).encode("utf-8")
        header = struct.pack(">I", len(payload)) + bytes([MSG_CONTROL])
        conn.sendall(header + payload)

    def _handle_signal(self, signum, frame):
        self._running = False


def main(engine_class: type[OsirisEngine]):
    """
    Entry point helper for engine scripts.

    Usage at the bottom of your engine.py:
        if __name__ == "__main__":
            main(MyEngine)
    """
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <socket_path>", file=sys.stderr)
        sys.exit(1)

    socket_path = sys.argv[1]
    engine = engine_class()
    engine.serve(socket_path)
