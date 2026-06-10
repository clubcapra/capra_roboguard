"""Livox Mid-360 lidar sensor: rosette raycast -> point cloud.

The Mid-360 has a 360 deg horizontal x ~59 deg vertical (-7..+52) field and a
NON-REPETITIVE scan pattern: successive frames sweep different points, so
coverage fills in over time rather than tracing fixed rings. We model that with a
golden-angle spiral whose azimuth phase advances each frame -> every scan is a
fresh rosette. Rays are cast with p.rayTestBatch against the COLLISION world;
each hit becomes a point.

Foliage pass-through falls out for free: foliage carries no collision (and once
the cutout mesh is promoted to collision, the rays pass through its gaps and hit
the leaves) -- exactly the "transparent gap -> through, leaf -> return" model.
"""
from __future__ import annotations

import socket
import struct
from dataclasses import dataclass, field
from typing import Any, Tuple

import numpy as np
import pybullet as p

from .base import Sensor, register

_GOLDEN = np.pi * (3.0 - np.sqrt(5.0))            # ~2.39996 rad
_BATCH = 16000                                    # pybullet rayTestBatch ceiling
_G_VEC = np.array([0.0, 0.0, -9.80665])           # gravity, world ENU (m/s^2)
_GRAVITY = 9.80665
# Mid-360 built-in IMU (ICM-40609) origin offset from the point-cloud origin,
# per the Livox user manual (same axes as the point cloud). Metres.
_IMU_LEVER_M = np.array([0.011, 0.02329, -0.04412])


@dataclass
class LidarScan:
    points: np.ndarray                            # (N, 3) world xyz of hits
    ranges: np.ndarray                            # (N,) metres
    hit_ids: np.ndarray                           # (N,) pybullet body id
    pose: Tuple[Any, Any]                         # sensor (pos, orn xyzw)
    n_rays: int = 0
    t: float = 0.0


@register("livox_mid360")
class LivoxMid360(Sensor):
    """Livox Mid-360: 360 deg azimuth x (-7..+52) deg elevation, 200k pts/s, 10 Hz,
    0.1-70 m, non-repetitive scan. The scan is built about the sensor's SPIN axis
    in the mount-link frame (default local +Y -- the rove URDF mounts the puck with
    +Y up; the real Livox datum is its +Z, so override `spin_axis` per mount if a
    link differs). Azimuth sweeps the plane perpendicular to the spin axis."""

    def __init__(self, **kw):
        self.range_m = float(kw.pop("range_m", 70.0))
        # min_range = closest distance we REGISTER a return (drops near clutter /
        # the sensor's own near field); real Mid-360 ~0.2 m here.
        self.min_range = float(kw.pop("min_range_m", 0.2))
        # Rays emanate from the sensor GEOMETRY CENTROID (centre of its own
        # housing) and must NOT collide with that own housing -- but they DO hit
        # everything else (the cage, the other Livox, the world), which casts the
        # real shadows. The "ignore own housing" is a collision-group mask set by
        # the runtime (see ray_mask); we never let the ray die on ourselves.
        self.ray_mask = int(kw.pop("ray_mask", -1))
        self.el_min = np.radians(float(kw.pop("el_min_deg", -7.0)))
        self.el_max = np.radians(float(kw.pop("el_max_deg", 52.0)))
        pps = float(kw.pop("points_per_sec", 200000.0))
        # spin (vertical) axis + a reference azimuth-zero axis, in the LINK frame
        spin = np.array(kw.pop("spin_axis", (0.0, 1.0, 0.0)), float)
        ref = np.array(kw.pop("forward", (1.0, 0.0, 0.0)), float)
        kw.setdefault("rate_hz", 10.0)
        self.max_rays = int(kw.pop("max_rays", 24000))
        # rayTestBatch worker threads: 0 = pybullet's default (ALL cores). On a
        # core-limited host two parallel lidar workers each grabbing all cores
        # oversubscribe and starve the physics server -> cap this (e.g. 2) so the
        # raycast leaves cores for realtime physics. Settable via set_ray_threads.
        self.ray_threads = int(kw.pop("ray_threads", 0))
        self.exclude_self = bool(kw.pop("exclude_self", True))
        # bodies that OCCLUDE rays but whose hits we don't register as points --
        # the synced concave self-occluders (the real pole/mast mesh). They stop
        # the ray (-> shadow) but never appear in the cloud (they're the robot).
        self.occluder_ids = set(kw.pop("occluder_ids", None) or [])
        super().__init__(**kw)
        # orthonormal scan basis: w = spin (pole), (u, v) span the azimuth plane
        self._w = spin / np.linalg.norm(spin)
        u = ref - self._w * (ref @ self._w)
        if np.linalg.norm(u) < 1e-6:                    # ref parallel to spin
            u = np.array([1.0, 0.0, 0.0]) - self._w * self._w[0]
        self._u = u / np.linalg.norm(u)
        self._v = np.cross(self._w, self._u)
        self._frame = 0
        self.set_rays(min(self.max_rays,
                          max(64, int(pps / max(self.rate_hz, 1.0)))))

    def set_ray_threads(self, n: int) -> None:
        """Cap rayTestBatch worker threads (0 = all cores). Keeps two parallel
        lidar workers from oversubscribing the cores the physics server needs."""
        self.ray_threads = int(n)

    def set_rays(self, n: int) -> None:
        """Set rays-per-scan (recomputes the spiral). Lower it for a fast live
        preview; the real sensor is points_per_sec/rate_hz."""
        self.rays_per_scan = int(n)
        i = np.arange(self.rays_per_scan)
        self._el = self.el_min + (self.el_max - self.el_min) * (i / self.rays_per_scan)
        self._az0 = i * _GOLDEN                          # golden-angle azimuth spread

    def _centroid(self) -> np.ndarray:
        """World centroid of the sensor's OWN housing geometry (ray origin)."""
        idx = self.robot.link_index.get(self.link)
        if idx is not None and idx >= 0:
            try:
                lo, hi = p.getAABB(self.robot.body_id, idx)
                return (np.array(lo) + np.array(hi)) * 0.5
            except Exception:
                pass
        return np.array(self.link_pose()[0])

    def _dirs_local(self) -> np.ndarray:
        az = self._az0 + self._frame * 0.61803           # irrational phase / frame
        ce, se = np.cos(self._el), np.sin(self._el)
        # cos(el) in the (u,v) azimuth plane + sin(el) along the spin pole w
        return (ce * np.cos(az))[:, None] * self._u \
            + (ce * np.sin(az))[:, None] * self._v \
            + se[:, None] * self._w

    def _sample(self) -> LidarScan:
        pos, orn = self.link_pose()
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        # ALL rays emanate from the sensor's own geometry centroid.
        origin = self._centroid().astype(np.float32)
        dirs = (R @ self._dirs_local().T).T              # (N,3) world
        starts = np.repeat(origin[None, :], len(dirs), axis=0)
        ends = origin + dirs * self.range_m
        self._frame += 1

        id_parts, hit_parts = [], []
        for s in range(0, len(dirs), _BATCH):
            # numThreads parallelises (0 = all cores; capped via ray_threads so
            # parallel workers don't starve physics); ray_mask makes the ray ignore
            # ONLY the sensor's own housing (its group) -- hits everything else.
            res = p.rayTestBatch(starts[s:s + _BATCH].tolist(),
                                 ends[s:s + _BATCH].tolist(),
                                 numThreads=self.ray_threads,
                                 collisionFilterMask=self.ray_mask)
            # column-extract once per batch (comprehensions beat per-ray append)
            id_parts.append(np.fromiter((r[0] for r in res), np.int32, len(res)))
            hit_parts.append(np.array([r[3] for r in res], np.float32))
        ids = (np.concatenate(id_parts) if id_parts else np.empty(0, np.int32))
        hits = (np.concatenate(hit_parts) if hit_parts
                else np.empty((0, 3), np.float32)).reshape(-1, 3)

        keep = ids >= 0                                  # a ray that hit something
        if self.exclude_self:                            # drop robot parts (they
            keep &= ids != self.robot.body_id            # still OCCLUDE -> shadows)
        if self.occluder_ids:                            # ditto for the mast mesh
            keep &= ~np.isin(ids, list(self.occluder_ids))
        pts = hits[keep]
        rng = (np.linalg.norm(pts - origin, axis=1) if len(pts)
               else np.empty(0, np.float32))
        reg = rng >= self.min_range                      # register only >= min_range
        return LidarScan(points=pts[reg], ranges=rng[reg], hit_ids=ids[keep][reg],
                         pose=(tuple(origin), tuple(orn)), n_rays=len(dirs),
                         t=self.clock.now() if self.clock else 0.0)

    def imu_sample(self):
        """The Mid-360 built-in IMU: body-frame gyro (rad/s) + specific-force
        accel (g), sampled at the IMU's lever-arm offset. Returns (gyro3, accel3)
        numpy arrays in the lidar/point-cloud frame -- matching the real sensor's
        coordinate convention. 200 Hz on the real unit."""
        import time as _time
        idx = self.robot.link_index.get(self.link)
        if idx is None or idx < 0:
            return np.zeros(3), np.array([0.0, 0.0, 1.0])
        ls = p.getLinkState(self.robot.body_id, idx, computeLinkVelocity=1,
                            computeForwardKinematics=1)
        orn = ls[5]
        v_lin = np.array(ls[6])
        w_world = np.array(ls[7])
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        now = _time.time()
        last_v = getattr(self, "_imu_last_v", None)
        last_t = getattr(self, "_imu_last_t", None)
        dt = (now - last_t) if last_t is not None else 0.0
        a_world = (v_lin - last_v) / dt if (last_v is not None and dt > 1e-9) else np.zeros(3)
        self._imu_last_v, self._imu_last_t = v_lin, now
        # lever-arm centripetal term (skip angular-accel term: needs alpha)
        r_world = R @ _IMU_LEVER_M
        a_imu = a_world + np.cross(w_world, np.cross(w_world, r_world))
        f_body = R.T @ (a_imu - _G_VEC)              # specific force, m/s^2
        gyro = R.T @ w_world                          # rad/s, body frame
        return gyro, f_body / _GRAVITY                # accel in g (Livox unit)


# native Livox-SDK2 IMU UDP packet: 36-byte LivoxLidarEthernetPacket header
# (data_type=0 for IMU) + 24-byte payload (6x float32: gyro xyz, accel xyz). A
# future real-hardware Livox driver speaks the identical format.
_IMU_HDR = struct.Struct("<B H H H H B B B 12s I Q")   # 36 bytes
_IMU_PAYLOAD = struct.Struct("<6f")                     # gyro xyz, accel xyz
_IMU_DATA_TYPE = 0


def encode_imu_packet(gyro, accel, ts_ns: int, udp_cnt: int = 0,
                      frame_cnt: int = 0) -> bytes:
    """Pack one Mid-360 IMU sample as a native Livox UDP datagram."""
    payload = _IMU_PAYLOAD.pack(float(gyro[0]), float(gyro[1]), float(gyro[2]),
                                float(accel[0]), float(accel[1]), float(accel[2]))
    hdr = _IMU_HDR.pack(0, _IMU_HDR.size + _IMU_PAYLOAD.size, 0, 1,
                        udp_cnt & 0xFFFF, frame_cnt & 0xFF, _IMU_DATA_TYPE, 0,
                        b"\x00" * 12, 0, ts_ns & 0xFFFFFFFFFFFFFFFF)
    return hdr + payload


def decode_imu_packet(buf: bytes):
    """Inverse of encode_imu_packet -> (gyro3, accel3, ts_ns). Raises on a
    non-IMU or malformed datagram."""
    if len(buf) < _IMU_HDR.size + _IMU_PAYLOAD.size:
        raise ValueError("short imu packet")
    fields = _IMU_HDR.unpack_from(buf, 0)
    data_type, ts_ns = fields[6], fields[10]
    if data_type != _IMU_DATA_TYPE:
        raise ValueError(f"not an imu packet (data_type={data_type})")
    g0, g1, g2, a0, a1, a2 = _IMU_PAYLOAD.unpack_from(buf, _IMU_HDR.size)
    return np.array([g0, g1, g2]), np.array([a0, a1, a2]), ts_ns


class LivoxImuUdpPublisher:
    """Sends Mid-360 IMU samples as native Livox UDP datagrams (default port
    56401, the real host-side IMU port). Fire-and-forget."""

    def __init__(self, host: str = "127.0.0.1", port: int = 56401):
        self.host, self.port = host, int(port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cnt = 0

    def publish(self, gyro, accel, ts_ns: int) -> None:
        try:
            self._sock.sendto(encode_imu_packet(gyro, accel, ts_ns, udp_cnt=self._cnt),
                              (self.host, self.port))
            self._cnt += 1
        except OSError:
            pass

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass


# binary point-cloud UDP stream: each scan is fragmented across one or more
# datagrams (a full Mid-360 scan is ~13k points -> ~160 KB, well over the UDP
# datagram limit), then reassembled by frame id on the consumer -- exactly how
# the real Livox streams its cloud as many packets per frame. NO decimation: the
# consumer receives every returned point.
_PC_MAGIC = b"LVX2"
# magic, frame_id, packet_idx, n_packets, total_points, t, pose(xyz + quat xyzw)
_PC_HDR = struct.Struct("<4s I H H I d 7f")
# points per datagram. 4000 pts * 12 B + 52 B hdr ~= 48 KB, safely one datagram.
_PC_PTS_PER_PKT = 4000


def encode_cloud(scan: "LidarScan", frame_id: int = 0,
                 max_points_per_packet: int = _PC_PTS_PER_PKT) -> list:
    """Pack a LidarScan as a LIST of UDP datagrams (the full cloud, no decimation).

    Each datagram is `_PC_HDR` + that fragment's float32 xyz; the consumer
    reassembles them by frame id (see `CloudReassembler`). An empty scan still
    yields one header-only datagram so the stream stays live (heartbeat)."""
    pts = np.ascontiguousarray(scan.points, np.float32).reshape(-1, 3)
    total = len(pts)
    mpp = max(1, int(max_points_per_packet))
    n_pkts = max(1, (total + mpp - 1) // mpp)
    pos, orn = scan.pose
    pose = [float(v) for v in pos] + [float(v) for v in orn]
    out = []
    for idx in range(n_pkts):
        chunk = pts[idx * mpp:(idx + 1) * mpp]
        hdr = _PC_HDR.pack(_PC_MAGIC, frame_id & 0xFFFFFFFF, idx, n_pkts,
                           total, float(scan.t), *pose)
        out.append(hdr + chunk.tobytes())
    return out


def decode_cloud(buf: bytes):
    """Decode ONE datagram -> (points Nx3 float32, t, (pos, orn), frame_id,
    packet_idx, n_packets, total_points). Feed these to a `CloudReassembler` to
    rebuild the whole scan."""
    magic, fid, idx, n_pkts, total, t, *pose = _PC_HDR.unpack_from(buf, 0)
    if magic != _PC_MAGIC:
        raise ValueError("bad cloud magic")
    n = (len(buf) - _PC_HDR.size) // 12
    pts = np.frombuffer(buf, np.float32, count=n * 3,
                        offset=_PC_HDR.size).reshape(-1, 3)
    return pts, t, (tuple(pose[:3]), tuple(pose[3:])), fid, idx, n_pkts, total


class CloudReassembler:
    """Rebuilds a full multi-packet Livox scan from its datagrams.

    `feed(buf)` returns `(points, t, (pos, orn))` once every packet of a frame has
    arrived, else None. Loopback UDP doesn't reorder or drop, so a frame completes
    when its packet count is reached; a new frame id resets any partial frame."""

    def __init__(self):
        self._fid = None
        self._parts = {}
        self._meta = None          # (t, pose, n_pkts)

    def feed(self, buf: bytes):
        pts, t, pose, fid, idx, n_pkts, total = decode_cloud(buf)
        if fid != self._fid:                       # new frame -> drop any partial
            self._fid, self._parts, self._meta = fid, {}, (t, pose, n_pkts)
        self._parts[idx] = pts
        if len(self._parts) >= self._meta[2]:      # all packets for this frame in
            ordered = [self._parts[i] for i in sorted(self._parts)]
            allpts = (np.concatenate(ordered) if ordered
                      else np.empty((0, 3), np.float32))
            t0, pose0, _ = self._meta
            self._fid, self._parts, self._meta = None, {}, None
            return allpts, t0, pose0
        return None


class LidarUdpPublisher:
    """Streams each Livox scan as binary point-cloud UDP datagrams (one or more
    per scan, one stream per sensor channel/port). Fire-and-forget, like the real
    sensor -- the consumer reassembles by frame id with `CloudReassembler`."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5022,
                 max_points_per_packet: int = _PC_PTS_PER_PKT):
        self.host, self.port = host, int(port)
        self.max_points_per_packet = int(max_points_per_packet)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._frame = 0

    def publish(self, scan: "LidarScan") -> None:
        if scan is None:
            return
        for pkt in encode_cloud(scan, self._frame, self.max_points_per_packet):
            try:
                self._sock.sendto(pkt, (self.host, self.port))
            except OSError:
                pass
        self._frame = (self._frame + 1) & 0xFFFFFFFF

    def close(self) -> None:
        try:
            self._sock.close()
        except OSError:
            pass
