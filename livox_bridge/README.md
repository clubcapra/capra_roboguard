# livox_bridge

Thin **Livox-SDK2 sidecar** for the Roboguard's two **Mid-360** lidars. The real
units are raw on the LAN (`192.168.2.40` = bottom, `192.168.2.41` = top) — not
behind `rove_sensor_api` — and stay idle until a host handshake. This sidecar
does that handshake (via the mature SDK) and re-emits each unit's **point cloud +
IMU** as small `LVXR` UDP datagrams to `rove_control_bridge`.

Points are in the **lidar sensor frame, in metres**. The bridge applies the URDF
mount extrinsics + **Livox-IMU pole-shake compensation** and runs the hazard
reflex / cost map. Keeping the geometry/filtering in Rust (not here) keeps it
testable and tunable without rebuilding C++.

## Build

Needs Livox-SDK2 built in-tree (default `/home/capra/Livox-SDK2`). On this box
(gcc-15 / cmake-4) the SDK itself needs:

```sh
cd /home/capra/Livox-SDK2 && mkdir -p build && cd build
cmake -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_CXX_STANDARD=17 \
      -DCMAKE_CXX_FLAGS="-include cstdint" .. && make -j
```

Then:

```sh
cd livox_bridge && mkdir -p build && cd build
cmake .. && make -j          # override SDK loc with -DLIVOX_SDK2_DIR=...
```

## Run

```sh
./livox_bridge ../config/mid360.json [dest_ip=127.0.0.1] [base_port=7020]
```

Emits to `dest_ip:base_port` (points) and `base_port+1` (imu). `host_ip` in the
config must be the host's IP on the lidar subnet (`192.168.2.2`).

## LVXR wire format (little-endian)

| off | type | field |
|----|------|-------|
| 0  | `[u8;4]` | magic `"LVXR"` |
| 4  | u8 | version (1) |
| 5  | u8 | msg_type (1 = points, 2 = imu) |
| 6  | u8 | lidar_id (last IP octet: 40 = bottom, 41 = top) |
| 7  | u8 | data_type (1 = cartesian-high, 0 = imu) |
| 8  | u16 | count (points; 1 for imu) |
| 10 | u16 | reserved |
| 12 | u64 | timestamp_ns (host arrival) |
| 20 | payload | points: `count*(f32 x,y,z)` m · imu: `f32 gyro_xyz, acc_xyz` (acc in g) |
