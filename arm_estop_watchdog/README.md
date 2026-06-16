# arm_estop_watchdog

Stopgap e-stop indicator. The robot has no software e-stop status feed yet, but
the Kinova arm (`192.168.2.50`) is powered through the e-stop circuit, so we use
**arm reachable = e-stop released** as a proxy. The watchdog also folds in a
basic device-health check so the tower light shows when the robot isn't fully
booted.

Runs on the **Jetson** as a systemd service. Stdlib Python only — no deps.

## What it does

Every 5 s it pings the arm + a few hosts (in parallel) and drives the tower
light (`status-light-rove-2026` API on the Jetson) by priority, most → least
important. **The buzzer beeps once on every state change.**

| Light | Trigger | Default host |
|-------|---------|--------------|
| 🔴 solid RED       | arm down → **e-stopped** | `192.168.2.50` |
| 🟠 blinking ORANGE | local comm down          | `10.10.62.22` |
| 🟡 blinking YELLOW | station comm down        | `10.10.62.21` |
| 🟡 solid YELLOW    | steamdeck down           | `192.168.2.4` |
| 🟠 solid ORANGE    | any local device down    | `DEVICE_IPS` |
| 🟢 solid GREEN     | everything up            | — |

Yellow is the tower's *virtual* channel (lights red+orange+green together);
blinking uses software blink (`/{color}/blink`, the only blink mode the virtual
yellow supports).

On an arm **down→up** transition (e-stop recovery) it POSTs `/reload` to the
sensor API on the Pi (`rove_sensor_api`), which bounces the process so every
driver re-runs its connect path — the reliable fix for the arm not answering
after a power-cycle. `/reload` fires **only** on arm recovery.

## Monitored hosts

Each of these is pinged individually and has its own priority/color (see table):
`ARM_IP`, `LOCAL_COMM_IP`, `STATION_COMM_IP`, `STEAMDECK_IP`, plus the
`DEVICE_IPS` health set (solid orange if any are down).

## Configuration (env vars)

| Var               | Default                  | Meaning |
|-------------------|--------------------------|---------|
| `ARM_IP`          | `192.168.2.50`           | e-stop proxy → solid red |
| `LOCAL_COMM_IP`   | `10.10.62.22`            | → blinking orange |
| `STATION_COMM_IP` | `10.10.62.21`            | → blinking yellow |
| `STEAMDECK_IP`    | `192.168.2.4`            | → solid yellow |
| `DEVICE_IPS`      | health-set IPs (CSV)     | → solid orange if any down |
| `TOWER_URL`       | `http://192.168.2.3:3000`| tower-api (Jetson) |
| `SENSOR_API_URL`  | `http://192.168.2.2:8080`| Pi `rove_sensor_api` |
| `BLINK_ON_MS` / `BLINK_OFF_MS` | `400` / `400` | blink period |
| `PING_INTERVAL`   | `5`                      | seconds between sweeps |
| `PING_TIMEOUT`    | `1`                      | per-ping timeout (s) |
| `HTTP_TIMEOUT`    | `3`                      | tower/sensor HTTP timeout (s) |
| `LOG_LEVEL`       | `INFO`                   | logging level |

## Install

1. Edit `arm-estop-watchdog.service` and set `SENSOR_API_URL` to the Pi's IP
   (replace the `192.168.2.X` placeholder).
2. Install + enable:

   ```bash
   sudo ./install_service.sh
   ```

The unit assumes the repo lives at
`/home/capra/data/capra_rove_stack/capra_roboguard/arm_estop_watchdog` (matching
`rove_sensor_api`). Adjust the paths in the `.service` file if yours differs.

## Manage

```bash
systemctl status  arm-estop-watchdog
systemctl restart arm-estop-watchdog
journalctl -u     arm-estop-watchdog -f
sudo ./uninstall_service.sh
```

## Quick dry run (no install)

```bash
SENSOR_API_URL=http://<pi-ip>:8080 python3 arm_estop_watchdog.py
```
