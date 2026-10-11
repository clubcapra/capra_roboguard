# Livox Mid-360 driver

`HALo` owns the Livox SDK2 session in its own process. The native
adapter calls Rust for each point/IMU packet; there is no sidecar executable.
One read-only sensor per configured lidar (`livox_<final IP octet>`) is registered
with persistent identity-based port reservations.

HTTP `/livox_40/data` and normal sensor-API UDP subscriptions expose the latest
point packet, IMU values, arrival timestamps, freshness, and packet counters.
Point coordinates are sensor-frame metres; gyro is rad/s and acceleration is g.
Full-rate point segments are delivered to each standard HALo UDP subscriber.
The driver can also mirror the existing LVXR contract for legacy consumers.
The autonomy bridge's decoding, leveling, and reflex math remain unchanged.

## Build and enable

Build [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) with position-independent
code so its static library can be linked into the driver shared library:

```sh
cmake -S /path/to/Livox-SDK2 -B /path/to/Livox-SDK2/build \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_CXX_STANDARD=17 -DCMAKE_CXX_FLAGS="-include cstdint"
cmake --build /path/to/Livox-SDK2/build --parallel
```

From `HALo/`:

```sh
cmake -S src/drivers/livox/native -B src/drivers/livox/native/build \
  -DLIVOX_SDK2_DIR=/path/to/Livox-SDK2
cmake --build src/drivers/livox/native/build --parallel
cp config/livox.toml.example config/livox.toml
# Edit host_ip and lidar_ips in config/livox.toml for the actual instance.
cargo run
```

`livox.toml` is the only operator configuration. The driver validates its IPs and
ports, resolves the library path relative to the configuration directory and
creates the SDK's required JSON privately (mode 0600) for that session. It removes
that file after SDK shutdown. No checked-in device JSON or proxy process is needed.
Host UDP ports are reserved from HALo's served port allocation. `host_ports` and
`lidar_ports` tables can override the documented SDK defaults; each table needs
`cmd_data_port`, `push_msg_port`, `point_data_port`, `imu_data_port`, `log_data_port`.
Configured final IP octets must be unique because the legacy callback ABI uses
an eight-bit lidar identity. Existing addresses ending in 40/41 retain their IDs.

The service installer rebuilds the native library when `config/livox.toml`
exists. For a different SDK location, configure the CMake build directory first;
the installer reuses its cache. Rust builds/tests need no SDK or native library.
Missing config disables the Livox group; invalid config or SDK load failures
remain visible in HALo health and exhaust a bounded recovery budget. Sim-backend
mode feeds both simulator lidars through the same driver ingest and subscriptions.

SDK callback threads stop before Rust state or the native library is released.
Freshness is tracked independently for points and IMU; a registered but silent
lidar reports `connected=false`. Perception continues to apply its configured
missing-lidar policy to the LVXR stream.

Older diagnostic tools are in `tools/livox/`. These tools and the bridge's
legacy `source = "lvxr"` perception mode require the optional `points_target`
and `imu_target` mirrors enabled in `config/livox.toml`. Each mirror has one
destination; standard HALo subscribers can coexist without binding those ports.

## Validation

`cargo test` includes a native ABI fixture compiled with Rust: it checks per-lidar
discovery/state, binary forwarding, freshness, failed startup, and shared SDK
session cleanup without hardware. To also check SDK structs and native packet
conversion, configure CMake with `-DHALO_LIVOX_BUILD_TESTS=ON`, build, and run
`ctest --test-dir src/drivers/livox/native/build --output-on-failure`.

## LVXR wire format

All fields are little-endian. The 20-byte header contains magic `LVXR`, version
1, message type (1 points, 2 IMU), lidar ID (last IP octet), data type (1 points,
0 IMU), u16 count, u16 reserved, and u64 host-arrival timestamp in nanoseconds.
Points carry `count` triples of f32 x/y/z in metres. IMU carries one tuple of
six f32 values: gyro x/y/z, then acceleration x/y/z in g.

## Multiple consumers

Consumers discover each lidar through HALo and use its advertised UDP data port.
Subscribe (`0x01`, the standard four-byte HALo header) starts full source updates
in JSON DATA messages (`0x03`). Unsubscribe (`0x02`) stops that client. Every client
has an independent subscription; no static destination list is needed.

Point packets are segmented into at most 128 points per DATA payload to keep
JSON datagrams bounded. `packet_type`, `point_packet_index`, `point_packet_parts`,
arrival timestamps and counters identify point segments and separate IMU updates.
Snapshots served through HTTP or an explicit subscription `interval_ms` expose
the current point segment and latest IMU values. Update subscriptions receive all
source segments. Slow consumers are told when their bounded queue drops updates;
sequence numbers also help detect network loss. Subscriptions survive driver retry.

The simulator uses the same native-adapter ingest path for both lidars. Legacy
LVXR point/IMU mirrors remain optional physical-driver connection settings for
existing perception/calibration consumers. The console has no LiDAR-specific panel.
