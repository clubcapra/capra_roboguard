# Osiris Streaming API Bundle

A self-contained vision streaming API. Point it at a WebRTC, USB camera, RTSP,
HTTP, or file source; it runs the bundled models and streams detections (with
timestamps) over WebSocket.

## Engines included
- `rt_detr_coco`
- `rt_detr_pose`
- `trained_67f67256`

## Run

```bash
./run.sh
```

- REST + Scalar API docs: http://localhost:9092/docs
- The first launch of each engine sets up its Python venv or builds its Rust
  crate automatically (needs `python3` / `cargo` + `ffmpeg` on the target).

## Preview UI (create feeds + watch live)

```bash
./preview.sh        # then open http://localhost:8080
```

A zero-dependency web tool (python3 + ffmpeg only) to create a feed from any
source (RTSP / USB / file / `webrtc://`) and watch the **live video with
detection and track-ID overlays**. Set the port via `PREVIEW_PORT` in
`config.env`. Example source: `rtsp://192.168.2.4:8554/cam_front`.

## Run as a service (systemd, starts on boot)

```bash
sudo ./install-service.sh                 # installs + starts osiris-bundle.service
sudo ./install-service.sh my-cameras      # custom service name
sudo ./uninstall-service.sh               # stop + remove (bundle files kept)
```

Runs the bundle (gateway + API) under systemd as the installing user, with
`Restart=always`. Inspect with `systemctl status osiris-bundle` and
`journalctl -u osiris-bundle -f`. Override the user with
`OSIRIS_SERVICE_USER=...` and the bind address with `OSIRIS_BIND=...`.

## Endpoints

- `GET  /api/engines`            — list bundled engines
- `POST /api/feeds`              — start a feed: `{"source": "...", "engines": ["rt_detr_coco"]}`
- `GET  /api/feeds`              — list active feeds
- `DELETE /api/feeds/{id}`       — stop a feed
- `WS   /ws/{engine}/{feed_id}` — stream detections (JSON, with `timestamp_ms`)

## Source examples (the `source` field of POST /api/feeds)

| Source type | Value |
|-------------|-------|
| Video file  | `/abs/path/clip.mp4` |
| RTSP camera | `rtsp://user:pass@host:554/stream` |
| HTTP stream | `http://host/stream.m3u8` |
| USB camera  | `/dev/video0` |
| WebRTC      | `webrtc://mystream` |

### WebRTC

1. Start the bundle (`run.sh` launches the gateway when `gateway/mediamtx` is present).
2. Publish your WebRTC stream via WHIP to
   `http://<host>:8889/mystream/whip`.
3. Create the feed:
   ```bash
   curl -X POST http://localhost:9092/api/feeds \
     -H 'Content-Type: application/json' \
     -d '{"source": "webrtc://mystream", "engines": ["rt_detr_coco"]}'
   ```
4. Subscribe: `ws://localhost:9092/ws/rt_detr_coco/<feed_id>`.

## Configuration

Edit **`config.env`** and re-run `./run.sh` (or `sudo systemctl restart osiris-bundle`):

- `OSIRIS_BIND` — REST + WebSocket API address (default `0.0.0.0:9092`)
- `OSIRIS_RTSP_PORT` — gateway RTSP port (default `8554`)
- `OSIRIS_WEBRTC_PORT` — gateway WebRTC/WHIP port (default `8889`)

First-run engine setup auto-detects the GPU and installs matching CUDA wheels
(or CPU). Tune install timeouts with `OSIRIS_ENGINE_INIT_TIMEOUT` /
`OSIRIS_ENGINE_READY_TIMEOUT` if on a slow link.
