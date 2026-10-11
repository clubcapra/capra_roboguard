# HALo — Hardware Abstraction Layer and I/O

HALo gives software a discoverable HTTP/UDP interface to hardware. Drivers own
serial, CAN or vendor SDK connections and cache telemetry; HALo serves their
schemas, commands and health without embedding robot motion planning. Robot
composition belongs in instance configuration and adapters. The Rove simulator
is one such adapter.

The design and repository audit are recorded in [doc/architecture.md](doc/architecture.md).
New hardware follows [the driver template](doc/driver-template.md): driver, instance TOML, catalogue registration.

## Start and connect

```sh
cargo build --release
# Read a password securely, then pass it on stdin; it is never a command argument.
read -r -s -p 'HALo password: ' HALO_PASSWORD; echo
printf '%s\n' "$HALO_PASSWORD" | target/release/halo --set-password
unset HALO_PASSWORD
cargo run --release
```

Use a password of 12–1024 bytes. Only its salted Argon2id PHC hash is stored in
`config/password.hash` (mode 0600). No default password is shipped. Open
`http://localhost:8080/` and log in. `/docs` serves the same local console.
HTML, CSS and JavaScript are embedded in the binary, with no CDN, package loader,
web fonts or JavaScript build step. The styling follows the compact layout of
Teltonika's [RutOS WebUI](https://wiki.teltonika-networks.com/view/RUTX11_WebUI).

1. HALo probes every configured driver as it starts; the console is available
   while probes run. Missing hardware and invalid configuration remain visible.
2. Inspect **Overview** for controller connection, hardware readiness, freshness
   and errors. A controller connection alone does not advertise its hardware.
3. In **Hardware connections**, edit physical connection settings, recovery
   limits and served UDP ports. Saving a connection retries its driver.
4. Use **Retry** to recover a driver resource group. Use **Restart HALo** for a
   full service bounce. Clients must reconnect after a full bounce or port edit.
5. Other software fetches `/discover` and binds using the returned identities,
   ports, command modes and endpoint paths. Fetch `/openapi.json` for schemas.

Change the administrator password in **Settings** using the current password and
a new password entered twice. The update is applied immediately, atomically
persists a fresh Argon2id hash and revokes every active session. Sign in again.
The CLI remains available for initial setup or local recovery; restart after CLI changes.

The login protects `/admin/*` and `/auth/password`, including the console's hardware commands.
Sessions last eight hours, use random HttpOnly/SameSite=Strict cookies and are
invalidated on restart. Mutating management requests require
`X-HALo-Request: 1`; login attempts are throttled. Use an HTTPS reverse proxy for
production management and set `HALO_SECURE_COOKIE=1` behind it.
**Existing hardware HTTP routes, discovery, OpenAPI, health, logs and raw UDP
remain accessible to trusted robot-network clients for compatibility.** Isolate
that network; console login does not authenticate the legacy motion protocol.

## Instance configuration and persistent ports

Set `HALO_CONFIG_DIR` to an instance-specific directory, or use `config/` beside
the checkout/binary. Files are loaded and validated again on each recovery attempt.

| File | Connection settings |
| --- | --- |
| `vectornav.toml` | Serial device and baudrate; use a stable udev path |
| `robotiq.toml` | Serial device, baudrate, Modbus slave ID, polling and activation |
| `kinova.toml` | USB/Ethernet, controller addresses/ports, receive timeout, SDK directory, joint offsets, gravity vector |
| `odrive.toml` | CAN interface and discovery interval; defaults to `CAN_IFACE=can0` |
| `livox.toml` | Host/lidar IPs, SDK adapter library and optional SDK network ports; driver generates vendor JSON internally |
| `server.toml` | HTTP port and default UDP push interval |
| `ports.toml` | New-instance served-port seeds and simulator backend mapping |
| `recovery.toml` | Default bounded recovery policy (copy the example) |
| `<driver>.recovery.toml` | Console-saved policy overriding the default |

Missing serial/Kinova/Livox connection files disable their probes until configured.
ODrive always attempts CAN discovery. Configure the CAN bitrate/interface in the
host's network setup; HALo does not require a Rove-specific systemd pre-start.

`ports.hardware.json` and `ports.sim.json` retain identities and assigned port
pairs across driver failures, discovery order changes and full restarts. Offline
reservations are never recycled. These files are authoritative after initial
commissioning; TOML served pins seed only identities without a saved assignment.
User-saved pairs survive physical estops, hardware power cycles and removal from
the device list. Returning identities reclaim their saved pair; a new identity
gets one free pair and keeps it until an operator changes it. Changing the
allocation base affects only new identities. Occupied saved ports report a bind
error instead of moving the hardware to another port.
Intentional port edits are validated, bound and atomically persisted. A bind
failure rolls back to the previous listeners. Data and command ports must be
nonzero, distinct, unique and outside reserved SDK/simulator ports.

The VectorNav identity remains stable after changing its serial path. ODrive
nodes are sorted before first assignment and identified by CAN node ID. Keep
instance configuration, port files and the password hash together when migrating
an installed system. Protect and back up the config directory.

Default recovery values are `max_attempts=3`, `attempt_timeout_ms=30000`,
`retry_delay_ms=2000`, `stale_after_ms=3000`. Recovery stops after its budget;
manual retry starts a fresh cycle. A healthy session lasting ten seconds resets
the failure budget. Retries replace hardware behind stable HTTP/UDP driver slots,
so existing UDP subscriptions resume after the device recovers.

CAN nodes share one physical bus recovery group; configured Livox devices share
one SDK session. The arm, gripper and VectorNav recover independently. A native
SDK call cannot be forcibly killed safely: a timed-out attempt remains
`timed_out` until it returns and releases its resources. Other drivers continue;
HALo never launches overlapping SDK initializations for that group.

Lifecycle changes, errors and command results go into hourly CSV logs. Last
state, last error and last readiness time also persist under
`LOG_DIR/driver_state/`. Historical readiness is never reused as current health.

## Discovery, health and command compatibility

`/discover` retains the `{"sensors": [...]}` contract with the existing IDs,
`data_port`, `command_port`, `command_mode` and endpoint paths. A new
`endpoints.health` path is additive. Initializing, absent and stale devices stay
out of discovery. Previously verified hardware with a fresh fault stays visible
as degraded so clients can diagnose and clear it; `/health` and `/<id>/health`
provide current errors and availability. Clients should refresh discovery during
startup/recovery instead of treating the first HTTP response as a complete robot.

`/openapi.json` is generated from current known driver schemas, including offline
slots useful for commissioning. Sensor routes resolve their current driver at
request time. Existing configuration, calibration, estop, endpoint-upload and log
routes remain available. Known schemas are retained after disconnecting hardware.
Commands reject unknown fields, wrong scalar types and partial joint setpoints.
Motion is rejected before hardware readiness; activation and fault-clearing
commands remain available where a controller is connected.

UDP framing is unchanged: version `1`, message type, sequence `u16` little-endian,
then JSON (four-byte header). Data-port messages are Subscribe `0x01`,
Unsubscribe `0x02`, Data `0x03`, SubscribeAck `0x04`; command-port messages are
Command `0x10` and CommandAck `0x11`; errors are `0xff`. Subscribe accepts
`{"interval_ms":100}`; valid intervals are 10–60000 ms, with at most 256
subscribers per device. Stream mode is a client publishing hint, not a separate
StreamStart/StreamStop protocol.

An arm/gripper command acknowledgement can indicate queuing rather than physical
completion. Confirm results in telemetry and health. Kinova's SDK remains on a
single worker thread; finite subnormal/all-zero position sentinels, unpowered
supply and missing actuators do not establish readiness. Robotiq readiness
requires fresh reads plus `gACT=1`, `gSTA=3`; failed polls clear `link_up`.

## Driver subscriptions and sensor commands

Every consumer obtains sensor identities, schemas and ports from `/discover` and
`/{id}/info`. Data uses the same four-byte UDP header and JSON DATA payload for
all drivers. Send Subscribe (`0x01`) to a sensor’s advertised data port; every
subscriber receives its own stream. Send Unsubscribe (`0x02`) to leave. No output
destinations need commissioning in the console.

Drivers declare `data_delivery`: `Snapshots` uses periodic `read_data`, while
`Updates` publishes source events through a bounded broadcast channel. A Subscribe
with no payload uses the driver’s declared delivery. An explicit `interval_ms`
selects periodic snapshots for either kind of driver. Slow consumers receive an
Error packet reporting dropped updates. UDP can still lose/reorder packets in
transit; use sequence numbers and driver timestamps to identify gaps.

Both Livox drivers publish point segments and IMU updates through this standard
subscription path. Points are sensor-frame metres, gyro is rad/s, acceleration
is g. Point packets are segmented into at most 128 points per DATA message;
`point_packet_index`, `point_packet_parts`, timestamps and packet counters identify
segments. `packet_type` distinguishes point, IMU and HTTP/sampled snapshots. IMU
updates contain an empty point array. Point and IMU freshness independently gate
readiness. Driver retries retain subscription sockets and reconnect update sources.
Optional LVXR mirrors remain in the physical driver configuration for existing
perception/calibration clients; they are not part of console destination management.

**Sensor commands** generates a tab for each loaded sensor and typed inputs from
its `command_schema`, with descriptions, units and live `data_schema` feedback.
Only selected fields are sent through the authenticated standard command endpoint.
Moving drivers declare `has_estop`; the same generic tab provides an Emergency stop
action through `/admin/sensors/{id}/estop`, with driver-owned confirmation text.
Read-only drivers show their data without a command form. Drivers declare readiness
requirements, standalone commands and optional confirmation text in field metadata;
the UI and transport apply those declarations without device-name exceptions.
The console sends one command per submission; streaming command cadence remains
advertised in `command_mode` for software control loops.

The simulator feeds `livox_40` (bottom) and `livox_41` (top) over backend channels
12 and 13 from the shared port map. `--no-lidar` disables in-process raycasts;
use `lidar_worker.py --rsa-backend --ports <same ports.toml>` for offloaded clouds.
`sim_fleet.sh` passes backend mode to its workers when `RSA_BACKEND=1`.

Health errors are decoded by their owning driver. ODrive error bits, Robotiq
faults and Kinova SDK failures include readable meanings alongside their original
codes; unknown codes remain visible. Shared health/UI code does not interpret
vendor fields.

## Kinova limp mode

Send `{"limp_mode":true}` alone to `/kinova_arm/command`, or use its generated field on the authenticated
Sensor commands tab. HALo clears queued trajectories, stops velocity resends,
selects `DIRECTTORQUE`, sends a 70-float zero torque command, suppresses the SDK
base gravity vector and switches to `TORQUE`. It checks
`GetTrajectoryTorqueMode` before reporting success, resends zero torque on the
worker cadence and blocks position/velocity/home commands. An uncertain or
failed mode transition appears in health and keeps motion blocked if restoration
fails. No torque-sensor zeroing, flash calibration, brake override or weakening
of vendor protection limits is performed.

Kinova also exposes the standard `/kinova_arm/estop` route and authenticated
console estop action: clear queued trajectories and enter limp mode. An unconfirmed
estop keeps motion inhibited even if internal rollback succeeds; only an explicit
restore can release that interlock.

`{"limp_mode":false}` explicitly returns to position control, restores the
configured `gravity_vector`, selects angular control and sends zero velocity.
A driver reconnect also restores normal control and that gravity vector. Limp
mode is an operator command and is not persisted across process restarts.
Telemetry adds `limp_supported`, `limp_mode` and `motion_inhibited`. Missing SDK
symbols disable the feature without preventing normal arm operation.

The arm can fall under gravity when enabled; support its load and clear its
workspace. This is a zero-commanded-torque implementation using the vendor gravity
vector setter, **not a guarantee of mechanically zero torque**: firmware damping,
friction, brakes, limits and protection behavior remain. Validate gravity
suppression, mode switching and release behavior on this custom arm before using
it for transport. Kinova's normal zero-torque mode includes gravity compensation;
see [the vendor torque documentation](https://github.com/Kinovarobotics/kinova-ros#torque-control)
and [SDK headers](https://github.com/Kinovarobotics/kinova-ros/blob/noetic-devel/kinova_driver/include/kinova/Kinova.API.EthCommandLayerUbuntu.h).

## Simulator and tests

```sh
cp config/sim.toml.example config/sim.toml
# Run the simulator using its Python environment, in another terminal:
cd ../rove_sim
python tools/sim_server.py --profile standard --no-rtsp --lidar-rays 256 \
  --rsa-backend --ports ../HALo/config/ports.toml
# Start HALo from HALo/ after setting its password.
```

`HALO_SIM_BACKEND=127.0.0.1:5000` also enables mocks; the older
`ROVE_SIM_BACKEND` override remains supported. Invalid simulator configuration
fails startup instead of falling through to physical drivers. The thirteen mocks (including both Mid-360 lidars)
use the same health/recovery supervisor and schemas as their physical drivers.
The simulator shares backend pins from `ports.toml`; console port edits change
served ports without changing simulator telemetry ports.

```sh
cargo test --all-targets
# Use the simulator Python interpreter; starts isolated HALo + real sim processes:
python tests/test_sim_flow.py
# Optional Playwright installation/browser required for actual console validation:
python tests/test_sim_flow.py --browser
# Check the fleet-style offloaded LiDAR worker too:
python tests/test_sim_flow.py --split-lidar
```

The end-to-end test covers missing telemetry, login/hash permissions, bounded
recovery, UDP continuity, limp mode settling under gravity, latched ODrive faults,
port edits, full bounce, standard multi-subscriber LiDAR delivery and discovery/OpenAPI. Browser validation prevents
external requests and checks login, password rotation/session revocation/persistence,
navigation, recovery settings, schema-generated commands and estops,
logs and mobile layout. The simulator does not reproduce Kinova firmware or SDK
hardware teardown; those require tests on the robot.

## Deployment and migration

`scripts/install_service.sh` builds and renders `halo.service` with the actual
checkout/user paths. Run it with sudo on the target Linux host. It stops the old
`rove-sensor-api.service` if installed, enables HALo and retains restart-on-failure
for `/reload`. No service installation is performed by the refactor itself.

The standalone Livox process is replaced by the optional
[in-process SDK2 driver](src/drivers/livox/README.md). Existing LVXR outputs
(points `7020`, IMU `7021` when configured) remain compatible. Stop any old standalone
Livox process before enabling it. The Python bridge and arm estop watchdog are
removed; use the Rust control bridge and retire installed old watchdog units as
described in [the stack README](../README.md). Robot models now live in
`../robot_doc`, with simulator/IK paths and mesh assets updated together.
