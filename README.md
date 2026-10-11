# capra_roboguard

Software stack for the **Roboguard** payload — the manipulation arm carried by
Capra rovers. The Roboguard adds a Kinova Gen2 6-DOF arm, a Robotiq 2F-140
gripper, a VectorNav VN-300 GNSS/INS, and one or more ODrives to the rover,
all driven from a single Jetson.

## Layout

- [HALo/](HALo/) — Hardware Abstraction Layer and I/O: reusable Rust hardware API (HTTP + UDP),
  including Kinova, Robotiq, VectorNav, ODrive, and optional Livox SDK2 drivers.
- [rove_control_bridge/](rove_control_bridge/) — Rust teleop/autonomy
  orchestrator: missions, positioning, perception, reflexes, and telemetry.
- [rove_ik_engine/](rove_ik_engine/) — Python motion engine: arm IK,
  collision checking, hardware mirroring, flipper/drum teleop, and saved poses.
- [rove_sim/](rove_sim/) — profile-driven PyBullet simulation and sensor feeds.
- [robot_doc/](robot_doc/) — caged/standard URDFs, meshes, and model documentation.

Teleop motion passes through the IK engine; autonomous track commands and
gripper commands use the sensor API. The hardware Livox driver runs inside
the sensor API and forwards the existing LVXR point/IMU streams to perception.
The simulator retains its own lidar feeds and can back the sensor API's mock
drivers with physics-derived telemetry.

## Quick start

```sh
cd HALo
cargo build --release
# Set the operator password locally: target/release/halo --set-password
cargo run
# Local console: http://localhost:8080/
```

To run as a systemd service on the Jetson:

```sh
cd HALo
sudo ./scripts/install_service.sh
```

## Hardware

| Device | Interface | Driver |
|---|---|---|
| Kinova Gen2 6-DOF arm | Ethernet (192.168.2.0/24) | [HALo/src/drivers/kinova/](HALo/src/drivers/kinova/) |
| Robotiq 2F-140 gripper | RS-485 (USB adapter) | [HALo/src/drivers/robotiq/](HALo/src/drivers/robotiq/) |
| VectorNav VN-300 | USB serial | [HALo/src/drivers/vectornav/](HALo/src/drivers/vectornav/) |
| ODrive Pro / S1 | CAN (`can0`) | [HALo/src/drivers/odrive/](HALo/src/drivers/odrive/) |
| Livox Mid-360 (bottom/top) | Ethernet (`192.168.2.40` / `.41`) | [HALo/src/drivers/livox/](HALo/src/drivers/livox/) |

The Kinova arm sits on a private subnet — the Jetson is `192.168.2.2`, the
arm is `192.168.2.50` by default. Both USB-serial devices have udev symlinks
(`/dev/ttyUSB_VN300`, `/dev/ttyUSB_gripper`); ODrives are auto-discovered from
CAN heartbeats.

## Refactor deployment

The Python control bridge and arm reachability watchdog have been removed.
Use the Rust control bridge's installer for `rove-control-bridge.service`.
On machines that previously installed the watchdog, retire its existing unit:

```sh
sudo systemctl disable --now arm-estop-watchdog.service
sudo rm -f /etc/systemd/system/arm-estop-watchdog.service
sudo systemctl daemon-reload
```

Stop any previously launched standalone Livox bridge before enabling the
[sensor API Livox driver](HALo/src/drivers/livox/README.md), since
both need the same SDK network ports. The driver defaults to the existing
LVXR destinations `127.0.0.1:7020` (points) and `:7021` (IMU).
Model paths in the simulator profiles and IK configs now point into `robot_doc`.
Keep local configuration overrides in sync with those paths when deploying.

HALo preserves dynamic discovery and OpenAPI. Its local console adds password
login, persistent port reservations, controller/hardware readiness, diagnostics
and bounded recovery per driver group. See [HALo's operating guide](HALo/README.md)
and [architecture audit](HALo/doc/architecture.md). Previously installed
`rove-sensor-api.service` units are retired by the HALo installer.
