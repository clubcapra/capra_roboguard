# capra_roboguard

Software stack for the **Roboguard** payload — the manipulation arm carried by
Capra rovers. The Roboguard adds a Kinova Gen2 6-DOF arm, a Robotiq 2F-140
gripper, a VectorNav VN-300 GNSS/INS, and one or more ODrives to the rover,
all driven from a single Jetson.

## Layout

- [rove_sensor_api/](rove_sensor_api/) — unified Rust API (HTTP + UDP) that
  exposes every device on the payload through one process. See its
  [README](rove_sensor_api/README.md) for protocol details, configs, and
  deployment scripts.

## Quick start

```sh
cd rove_sensor_api
cargo run
# Scalar UI: http://localhost:8080/docs
```

To run as a systemd service on the Jetson:

```sh
cd rove_sensor_api
sudo ./scripts/install_service.sh
```

## Hardware

| Device | Interface | Driver |
|---|---|---|
| Kinova Gen2 6-DOF arm | Ethernet (192.168.2.0/24) | [rove_sensor_api/src/drivers/kinova/](rove_sensor_api/src/drivers/kinova/) |
| Robotiq 2F-140 gripper | RS-485 (USB adapter) | [rove_sensor_api/src/drivers/robotiq/](rove_sensor_api/src/drivers/robotiq/) |
| VectorNav VN-300 | USB serial | [rove_sensor_api/src/drivers/vectornav/](rove_sensor_api/src/drivers/vectornav/) |
| ODrive Pro / S1 | CAN (`can0`) | [rove_sensor_api/src/drivers/odrive/](rove_sensor_api/src/drivers/odrive/) |

The Kinova arm sits on a private subnet — the Jetson is `192.168.2.2`, the
arm is `192.168.2.50` by default. Both USB-serial devices have udev symlinks
(`/dev/ttyUSB_VN300`, `/dev/ttyUSB_gripper`); ODrives are auto-discovered from
CAN heartbeats.
