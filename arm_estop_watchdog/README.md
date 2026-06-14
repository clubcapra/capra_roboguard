# arm_estop_watchdog

Stopgap e-stop indicator. The robot has no software e-stop status feed yet, but
the Kinova arm (`192.168.2.50`) is powered through the e-stop circuit, so we use
**arm reachable = e-stop released** as a proxy. The watchdog also folds in a
basic device-health check so the tower light shows when the robot isn't fully
booted.

Runs on the **Jetson** as a systemd service. Stdlib Python only — no deps.

## What it does

Every 5 s it pings the arm and a list of other devices (in parallel) and drives
the tower light (`status-light-rove-2026` API, local on the Jetson) by priority:

| Light  | Meaning                                              |
|--------|------------------------------------------------------|
| 🔴 RED    | Arm `.50` unreachable → **e-stopped**. Highest priority. |
| 🟠 ORANGE | Arm up, but ≥1 other device unreachable → not fully booted. |
| 🟢 GREEN  | Arm up and every monitored device reachable.         |

On an arm **down→up** transition (e-stop recovery) it POSTs `/reload` to the
sensor API on the Pi (`rove_sensor_api`), which bounces the process so every
driver re-runs its connect path — the reliable fix for the arm not answering
after a power-cycle. `/reload` fires **only** on arm recovery: not at startup,
not every poll, and not when a health-set device flaps.

## Monitored devices

- **E-stop proxy:** `192.168.2.50` (arm).
- **Health set (orange):** `.2 .7 .10 .12 .40 .41 .31 .32 .33 .34 .35 .36`.
  - `.36` is currently offline **for testing**, so expect orange until it's back.

## Configuration (env vars)

| Var              | Default                  | Meaning |
|------------------|--------------------------|---------|
| `ARM_IP`         | `192.168.2.50`           | e-stop proxy host |
| `DEVICE_IPS`     | the 12 IPs above (CSV)   | health set → orange |
| `TOWER_URL`      | `http://127.0.0.1:3000`  | local tower-api |
| `SENSOR_API_URL` | `http://192.168.2.X:8080`| **set to the Pi's IP** |
| `PING_INTERVAL`  | `5`                      | seconds between sweeps |
| `PING_TIMEOUT`   | `1`                      | per-ping timeout (s) |
| `HTTP_TIMEOUT`   | `3`                      | tower/sensor HTTP timeout (s) |
| `LOG_LEVEL`      | `INFO`                   | logging level |

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
