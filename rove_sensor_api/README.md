# rove_sensor_api

Unified sensor & actuator API for the Capra Roboguard payload. Runs on the
Jetson (or any aarch64 Linux box on the rover). Each connected device — the
VectorNav VN-300 IMU, the Kinova Gen2 arm, the Robotiq 2F-140 gripper, and any
ODrives on the CAN bus — is wrapped in a driver that registers with a central
`SensorRegistry`. The framework then exposes every driver over three transports
simultaneously:

- **HTTP / OpenAPI** — REST endpoints with a Scalar UI at `/docs` for
  introspection, one-shot commands, config read/write, calibration, e-stop,
  and log download.
- **UDP** — per-sensor data port (subscription-based push) and command port
  (one-shot or streaming). Used by the Steam Deck operator UI and ROS bridge.
- **CSV logs** — every sensor is polled at `LOG_POLL_HZ` and written to disk;
  every command received over UDP/HTTP is logged in the same hour bucket.

The binary is `capra-rove-interface` (single Rust crate at the workspace
root). HTTP is served on `0.0.0.0:8080`; UDP ports are auto-assigned starting
at `5000` (data, cmd, data, cmd, …) in registration order.

## How it works

```
                ┌─────────────────────────────────────────────┐
                │              SensorRegistry                  │
                │                                              │
   VectorNav ──▶│  vectornav  data:5000  cmd:5001              │
   Kinova    ──▶│  kinova     data:5002  cmd:5003              │
   Robotiq   ──▶│  robotiq    data:5004  cmd:5005              │
   ODrive(0) ──▶│  odrive_0   data:5006  cmd:5007              │
   ODrive(1) ──▶│  odrive_1   data:5008  cmd:5009              │
                │   …                                          │
                └────────────┬────────────────────────────────┘
                             │
              ┌──────────────┼──────────────┐
              ▼              ▼              ▼
         HTTP :8080      UDP per-sensor    CSV logs
         /docs (Scalar)  Subscribe/Data    logs/<date>/<hh>/
         /discover       Command/Ack
```

Every driver implements one trait — [`SensorDriver`](src/core/driver.rs) —
which declares its data schema, command schema, and (optionally) e-stop,
config, calibration, and per-endpoint write support. The HTTP and UDP layers
read those schemas at runtime, so adding a new sensor is just:

1. Implement `SensorDriver` in `src/drivers/<sensor>.rs`.
2. `pub mod <sensor>;` in `src/drivers/mod.rs`.
3. `registry.register(<sensor>::connect(...).await?);` in `main.rs`.

No HTTP routes, UDP plumbing, or doc strings to update.

### Drivers in this build

| ID prefix | Device | Transport | Notes |
|---|---|---|---|
| `vectornav` | VectorNav VN-300 GNSS/INS | Serial (`/dev/ttyUSB_VN300`) | Binary group output, auto-configured on connect |
| `kinova` | Kinova Gen2 6-DOF arm | Ethernet UDP (legacy SDK) | `.so` libs vendored at [vendor/kinova/aarch64/](vendor/kinova/) |
| `robotiq` | Robotiq 2F-140 gripper | Modbus RTU over `/dev/ttyUSB_gripper` | Auto-activates on connect |
| `odrive_<n>` | ODrive Pro / S1 | CAN (`can0`) | Discovered via heartbeat, one driver per node ID |

If a config file is missing or a device fails to connect, that driver is
skipped and the rest of the API still comes up.

### UDP wire format

4-byte header + JSON payload, little-endian:

```
┌──────────┬──────────┬───────────┬──────────────────────┐
│ version  │ msg_type │ seq_num   │ payload (JSON bytes) │
│ 1 byte   │ 1 byte   │ 2 bytes   │ variable             │
└──────────┴──────────┴───────────┴──────────────────────┘
```

Message types: `Subscribe 0x01`, `Unsubscribe 0x02`, `Data 0x03`,
`SubscribeAck 0x04`, `Command 0x10`, `CommandAck 0x11`, `Error 0xFF`. See
[src/protocol/packet.rs](src/protocol/packet.rs) for full details.

To subscribe to a data stream, send a `Subscribe` packet to the sensor's
data port; payload is optional `{"interval_ms": <n>}` (default 100 ms).
The server pushes `Data` packets at that cadence until you send
`Unsubscribe` or stop receiving.

For commands: REST-mode drivers expect a single `Command` packet and reply
with `CommandAck`. Stream-mode drivers (ODrive setpoints) expect `Command`
packets at the interval declared in their `command_mode`; each one is
processed as it arrives.

## Configs

All driver configs live in [config/](config/). **A driver is enabled iff its
config file exists.** Delete the file to skip the driver. Edit the file to
change ports, baudrates, IPs, etc.

- [config/vectornav.toml](config/vectornav.toml) — serial port and baud.
- [config/kinova.toml](config/kinova.toml) — local/robot IPs, UDP ports,
  command rate, joint offsets, optional `lib_dir` override.
- [config/robotiq.toml](config/robotiq.toml) — serial port, slave ID, poll
  rate, auto-activate flag.

ODrive has no TOML config — nodes are auto-discovered from CAN heartbeats.
The flat-endpoint map (firmware-generated JSON) is loaded at runtime; upload
a new one via `POST /odrive/endpoints` if you flash new firmware.

### udev rules

USB serial devices need stable symlinks. The rules referenced in the configs:

```
/dev/ttyUSB_VN300     → VectorNav VN-300 USB-serial bridge
/dev/ttyUSB_gripper   → Robotiq RS-485 adapter
```

Add them to `/etc/udev/rules.d/99-capra.rules` keyed off `idVendor`/
`idProduct` (use `udevadm info -a -n /dev/ttyUSBx` to find them).

### Environment variables

| Var | Default | Purpose |
|---|---|---|
| `RUST_LOG` | `capra_rove_interface=info` | tracing-subscriber filter |
| `CAN_IFACE` | `can0` | CAN interface used for ODrive discovery |
| `LOG_DIR` | `./logs` | Root directory for CSV logs |
| `LOG_POLL_HZ` | `10` | Per-sensor data poll rate for CSV logging |

CAN must be brought up before launch (e.g. `sudo ip link set can0 up type can
bitrate 500000`); the systemd unit handles this — see [Deploy as a service](#deploy-as-a-service).

## How to run

### Prerequisites

- Rust stable (1.75+).
- `libudev-dev`, `libssl-dev`, `pkg-config` (for `socketcan`/`tokio-serial`).
- For the Kinova driver: the vendored `.so` libs at
  [vendor/kinova/aarch64/](vendor/kinova/) — already in the tree, nothing to
  install.
- CAN tools if you need ODrives: `can-utils`, plus the kernel `can` and
  `can_raw` modules (`sudo modprobe can can_raw`).

### Local development

From this directory:

```sh
# Build (debug)
cargo build

# Run with default configs
cargo run

# Run with a specific CAN interface and verbose logging
RUST_LOG=capra_rove_interface=debug CAN_IFACE=can0 cargo run

# Release build for the Jetson
cargo build --release
```

When it starts you should see a line per registered sensor:

```
INFO registered sensor sensor=vectornav data_port=5000 cmd_port=5001 mode=Rest
INFO registered sensor sensor=kinova    data_port=5002 cmd_port=5003 mode=Stream { interval_ms: 13 }
INFO Scalar UI:   http://localhost:8080/docs
```

Open <http://localhost:8080/docs> for the interactive API browser, or
<http://localhost:8080/discover> for a JSON list of registered sensors with
their UDP ports.

### HTTP quick reference

```
GET  /discover                       — list sensors + per-sensor URL map
GET  /{id}/info                      — full schema + UDP protocol info
GET  /{id}/data                      — one-shot read
POST /{id}/command   {…JSON…}        — one-shot command
GET  /{id}/commands                  — command schema
GET  /{id}/endpoints                 — readable endpoints
GET  /{id}/endpoint/{path}           — read one endpoint
POST /{id}/endpoint/{path}  {value}  — write one endpoint (if supported)
POST /{id}/estop                     — e-stop (if supported)
GET  /{id}/config | POST /{id}/config — config read/write (if supported)
POST /{id}/calibrate {…params…}      — trigger calibration (if supported)
POST /odrive/endpoints  {…json…}     — upload flat-endpoint map (firmware)
GET  /logs                           — list CSV log files
GET  /logs/file/{*path}              — download a CSV
```

### Smoke test

```sh
curl -s http://localhost:8080/discover | jq
curl -s http://localhost:8080/vectornav/data | jq
curl -s -X POST http://localhost:8080/robotiq/command \
     -H 'content-type: application/json' \
     -d '{"position": 128, "speed": 200, "force": 50}'
```

A Python utility for poking the Kinova driver lives at
[tools/kinova_test.py](tools/kinova_test.py).

### Logs

CSVs are written to `$LOG_DIR/<YYYY-MM-DD>/<HH>/<sensor>.csv`, one row per
poll. Commands received on any UDP/HTTP path are logged to
`$LOG_DIR/<YYYY-MM-DD>/<HH>/inputs.csv` in the same hour bucket. Files
rotate on the hour automatically.

## Deploy as a service

Two scripts in [scripts/](scripts/) install/remove the API as a systemd unit
on the Jetson. They build the release binary, copy the unit file, enable it,
and start it.

```sh
# Install (asks for sudo)
sudo ./scripts/install_service.sh

# Check status / logs
systemctl status rove-sensor-api
journalctl -u rove-sensor-api -f

# Remove
sudo ./scripts/uninstall_service.sh
```

The unit runs as the `capra` user, brings `can0` up at 500 kbit/s before
launch, sets `LOG_DIR=/var/log/rove-sensor-api`, and restarts on failure.
Edit [scripts/rove-sensor-api.service](scripts/rove-sensor-api.service) to
change CAN bitrate, log dir, environment, or user.
