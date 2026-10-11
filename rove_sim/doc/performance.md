# Forest simulation performance

Measured on the Quadro P600 (2 GB VRAM), six virtual CPU cores and 12 GB RAM,
using `config/world.yaml`, `scenes/forest_demo.json` and the standard robot profile.
Physics, both LiDARs, five RTSP encoders, the GPU observer and HALo ran together.

| Measurement | Result |
| --- | --- |
| Five cameras, each 640×480 | 28.68 fps mean; 25.65 fps minimum window |
| Both LiDARs, each 4,000 rays/scan | 29.62 Hz mean; 28.32 Hz minimum window |
| Physics control | 43–46 Hz in recent control logs |
| GPU memory | 373 MiB during this run, including browser validation |
| Available host RAM | Approximately 6.06 GiB in the verification snapshot |
| Measurement duration | 120.7 seconds; 41 fresh rate samples |

The observer is 800×600. The minimum camera rate exceeded the requested 24 fps
in every measured three-second window. Rates count fresh renderer iterations and
LiDAR scans; the H.264 30 fps header alone is insufficient because the encoder
can repeat a last frame. Buffered readback adds one rendered frame of latency.
This is a sustained throughput result, not a guarantee against individual OS
scheduling stalls or extra workloads. Physics control targets 50 Hz; the measured
rate above corresponds to its actual realtime factor. The 240 Hz integration
step remains unchanged. Forest solver iterations are configured at 12.

Each LiDAR emits 120,000 simulated rays/s at this budget. This is lower than the
physical Mid-360’s nominal 200,000 points/s. Coverage accumulates with the
nonrepetitive scan pattern; neither alpha coverage nor geometry is approximated
to achieve the measured rate. Increase `LRAYS` only after measuring the resulting
rate on the target machine.

## What changed

The previous exporter removed image alpha, making opacity unrecoverable by a
renderer. Original diffuse/opacity textures from the user-provided ZIP were restored
and retained as RGBA, with both channels flipped together to match the OBJ UVs.
Web-viewer CDN derivatives contained rearranged image blocks and were removed.
Source and exported images have SHA-256 checks; workers reject replaced files.
The exporter now retains all 26 visual materials. Previously it excluded six
layers, including base terrain underneath the transparent road, exposing sky.
An 8×8 m grid around the robot now finds ground for all 6,561 downward rays.
The GPU discards uncovered fragments before depth writes. LiDAR samples
the same UVs, repeat wrapping, bilinear alpha and cutoff inside Embree traversal,
so gaps expose the nearest covered surface behind them. This follows the
[glTF alpha-mask definition](https://registry.khronos.org/glTF/specs/2.0/glTF-2.0.pdf)
and [Embree’s geometry filter API](https://github.com/RenderKit/embree/blob/v4.4.0/doc/src/api.md).

One EGL scene replaces camera scene copies and the software desktop viewer.
Static draw ranges are frustum culled, textures stay at original resolution,
framebuffer readback uses two buffers per stream, colour uses mipmaps and 4×
anisotropic filtering, and lighting is simple diffuse without live shadow passes. One CPU worker shares the terrain BVH between both
LiDARs; local BVHs follow robot links and props, with compiled AABB rejection.
The robot base visual and ray geometry now use the URDF base-link frame, obtained
by removing Bullet’s local inertial transform from its world centre-of-mass pose.
The previous renderer displaced the core by about 21 cm. Child link transforms
are verified against Bullet’s reference rendering, including rotated inertias.
Physics loads contact meshes instead of foliage/render geometry. Original mast
openings and self-occlusion are retained.

The old cameras ran at 256×192 and 5 fps. The software viewer alone consumed
roughly two CPU cores and 2.4 GB RSS; original workers duplicated expensive
foliage collision meshes. Host RAM use was approximately 11.3 GB. After the
initial changes it was approximately 4.8 GB; with all terrain layers restored,
the final host snapshot showed approximately 5.6 GiB used. Preview logs had also filled the RAM-backed
`/tmp`; those logs were preserved on disk under `.runtime/preview`, with the old
path retained as a symlink. Preview service configuration and saved ports are
unchanged. Runtime caches and preview data are ignored by Git.

## Validation and reproduction

GPU tests verify depth discard, UV orientation, transparent gaps, range limits,
more than 64 transparent layers, moving-object occlusion and buffered frame
ordering. Forest tests verify worker roles, physics stability and dynamic pose
sync. Twenty-six relevant regression tests passed, including import orientation,
checksum validation, colour minification, ground coverage and inertial transforms.
The HALo integration test from the optimization pass covered
readiness, discovery/OpenAPI compatibility, commands, estop/limp physics, readable
faults, password changes, persistent ports, targeted/full recovery and standard
multi-client UDP delivery. All 13 preview devices were discovered and all driver
groups were online. Browser login, live image, reload, orbit controls and logout
were checked separately. FFprobe verified all five 640×480 RTSP streams.
The fleet launcher was smoke-tested on an isolated RTSP port; shutdown preserved
the running preview. Unused MediaMTX listeners, including MoQ, are disabled.

```bash
SIM_REQUIRE_GPU=1 python -m pytest tests/test_texture_coverage.py tests/test_forest_world.py tests/test_m2_scene.py tests/test_m2_sensors.py -q
python tools/benchmark_fleet.py \
  --camera-metrics /path/to/gpu-metrics.json \
  --lidar-metrics /path/to/lidar-metrics.json \
  --state-file /path/to/state.json --seconds 120 --output /path/to/result.json
```

Set `--metrics` on `gpu_worker.py` and `lidar_worker.py` to provide those files.
The fleet’s `METRICS` and `LIDAR_METRICS` variables select its camera and LiDAR
metrics files; defaults are `/tmp/rove_sim_gpu_metrics.json` and
`/tmp/rove_sim_lidar_metrics.json`. [Raw measurements](performance.json) include rates, memory samples
and the final ground/device validation. [Forest view](../media/optimization/forest_view.jpg),
[RTSP camera frame](../media/optimization/cam_front.png), and
[top](../media/lidar/forest_livox_top.png)/[bottom](../media/lidar/forest_livox_bottom.png)
LiDAR maps are captured from the running system.
