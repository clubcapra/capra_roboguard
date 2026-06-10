"""Per-driver telemetry codecs mirroring rove_sensor_api's data schemas.

Each device is a symmetric pair:
  * sample()      -- read the sim's physics/kinematic state -> the driver's JSON
                     payload (what the mock SimSensorApi publishes).
  * apply(payload,state) -- decode that payload back into RobotState contributions
                     (what a real-mode RobotStateSource consumes).

Keeping encode + decode in ONE class per device is deliberate: the wire mapping
can never drift between the publisher and the consumer. Devices are auto-wired
from the existing profile (arm bind, gripper params, vn300 sensor) -- no new
config block required.

Joint-frame calibration (Kinova zero offset / signs, exact ENU datum) is the part
that needs the real robot; it defaults to identity here so the offline mock->real
loop round-trips exactly. TODO(calibrate) against hardware before real wiring.
"""
from __future__ import annotations

import math
import os
import time
from typing import Dict, List

import numpy as np
import pybullet as p

from ..state.base import RobotState

DEFAULT_DATUM = {"lat": 46.7512, "lon": 7.6131, "alt": 560.0}   # Thun-ish
_EARTH_R = 6378137.0
_G = np.array([0.0, 0.0, -9.80665])        # gravity (world ENU, m/s^2)
_GPS_WEEK_S = 604800.0


def _now_ns() -> int:
    return time.time_ns()


class KinovaArmDevice:
    """Kinova Gen2 6-DOF arm: joint_{1..6}_pos/vel/current/temp (degrees)."""
    channel = "kinova"

    def __init__(self, robot, joint_names: List[str],
                 offset_deg=None, sign=None):
        self.robot = robot
        self.body = robot.body_id
        self.joint_names = joint_names
        self.idx = [robot.joint_index[n] for n in joint_names]
        n = len(joint_names)
        self.offset_deg = list(offset_deg) if offset_deg else [0.0] * n
        self.sign = list(sign) if sign else [1.0] * n

    def sample(self) -> dict:
        o: Dict[str, float] = {}
        total_cur = 0.0
        for i, ix in enumerate(self.idx):
            st = p.getJointState(self.body, ix)
            o[f"joint_{i+1}_pos"] = math.degrees(st[0]) * self.sign[i] + self.offset_deg[i]
            o[f"joint_{i+1}_vel"] = math.degrees(st[1]) * self.sign[i]
            cur = abs(st[3])
            total_cur += cur
            o[f"joint_{i+1}_current"] = cur
            o[f"joint_{i+1}_temp"] = 25.0
        # Base power + IMU fields the real Kinova schema carries (at rest ~1 G up).
        o["bus_voltage"] = 24.0
        o["bus_current"] = round(total_cur, 3)
        o["accel_x"] = 0.0
        o["accel_y"] = 0.0
        o["accel_z"] = 1.0
        o["timestamp_ns"] = _now_ns()
        return o

    def apply(self, payload: dict, state: RobotState) -> None:
        for i, name in enumerate(self.joint_names):
            deg = payload.get(f"joint_{i+1}_pos")
            if deg is None:
                continue
            state.joints[name] = math.radians((deg - self.offset_deg[i]) * self.sign[i])


class RobotiqGripperDevice:
    """Robotiq 2F-140: a single `position` byte 0..255 (0=open .. 255=closed)."""
    channel = "robotiq"

    def __init__(self, robot, driven_joint: str, closed_rad: float, mimic: dict):
        self.robot = robot
        self.body = robot.body_id
        self.dj = driven_joint
        self.di = robot.joint_index[driven_joint]
        self.closed = float(closed_rad)
        self.mimic = dict(mimic or {})

    def sample(self) -> dict:
        rad = p.getJointState(self.body, self.di)[0]
        pos = int(max(0, min(255, round(rad / self.closed * 255))))
        # Full real RobotiqState schema so the Rust mock is a verbatim passthrough.
        return {"activated": True, "going_to_position": False, "status": 3,
                "object_status": 3, "fault": 0, "position_request_echo": pos,
                "position": pos, "current_raw": 0, "current_a": 0.0,
                "link_up": True, "timestamp_ns": _now_ns()}

    def apply(self, payload: dict, state: RobotState) -> None:
        pos = payload.get("position")
        if pos is None:
            return
        rad = float(pos) / 255.0 * self.closed
        state.joints[self.dj] = rad
        for mj, mult in self.mimic.items():          # PyBullet has no <mimic>:
            if mj in self.robot.joint_index:          # expand so the twin closes
                state.joints[mj] = rad * mult


class VectorNavDevice:
    """VN-300 INS: attitude (yaw/pitch/roll deg) + GNSS (lat/lon/alt), ENU about
    a fixed datum. Carries the robot base pose in real mode."""
    channel = "vectornav"

    def __init__(self, robot, datum=None, gnss_mode="nominal", seed=0, errors=None):
        self.robot = robot
        self.body = robot.body_id
        self.datum = dict(DEFAULT_DATUM, **(datum or {}))
        # GNSS operating mode (settable live for missions): nominal | degraded |
        # denied | spoofed. M4: degraded = big noise + dropouts; denied = no fix;
        # spoofed = an adversarial offset that creeps the reported position away.
        self.gnss_mode = gnss_mode
        # Error model on/off. Default ON (realistic random-walk bias + white noise);
        # set env ROVE_VN_ERRORS=0 to report the clean ground-truth pose -- useful
        # for autonomy bring-up / deterministic CI before PositionService fusion.
        self.errors = (os.environ.get("ROVE_VN_ERRORS", "1") != "0") if errors is None else errors
        self._rng = np.random.default_rng(seed)
        self._walk = np.zeros(2)          # slow position random walk (m)
        self._spoof = np.zeros(2)         # accumulating adversarial offset (m)
        self._last_v = np.zeros(3)        # for finite-diff specific force
        self._last_t = None

    def _gnss_enu(self, x: float, y: float):
        m = self.gnss_mode
        # denied/spoofed are DELIBERATE GNSS states (mission/test driven), so they
        # apply even with the noise model off -- that's how we exercise the
        # autonomy's GNSS-denied dead-reckoning / spoof rejection on a clean pose.
        if m == "denied":
            return x, y, False            # no fix -- autonomy holds last / SLAM
        if not self.errors:
            if m == "spoofed":
                self._spoof += np.array([0.03, 0.0])   # adversarial creep east
                return x + self._spoof[0], y + self._spoof[1], True
            return x, y, True             # clean ground-truth pose (errors off)
        sigma = {"nominal": 0.4, "degraded": 4.0, "spoofed": 0.4}.get(m, 0.4)
        self._walk += self._rng.normal(0, 0.03, 2)
        noise = self._rng.normal(0, sigma, 2)
        if m == "spoofed":
            self._spoof += np.array([0.03, 0.0])    # creep east ~0.3 m/s @10 Hz
        fix = (self._rng.random() > 0.3) if m == "degraded" else True
        return (x + noise[0] + self._walk[0] + self._spoof[0],
                y + noise[1] + self._walk[1] + self._spoof[1], fix)

    def sample(self) -> dict:
        (x, y, z), orn = p.getBasePositionAndOrientation(self.body)
        lin, ang = p.getBaseVelocity(self.body)
        roll, pitch, yaw = p.getEulerFromQuaternion(orn)
        gx, gy, fix = self._gnss_enu(x, y)
        lat0, lon0 = self.datum["lat"], self.datum["lon"]
        lat = lat0 + math.degrees(gy / _EARTH_R)
        lon = lon0 + math.degrees(gx / (_EARTH_R * math.cos(math.radians(lat0))))
        alt = self.datum["alt"] + z

        # --- IMU: body-frame gyro + specific-force accel (incl. gravity) ---
        R = np.array(p.getMatrixFromQuaternion(orn)).reshape(3, 3)
        v = np.array(lin)
        now = time.time()
        dt = (now - self._last_t) if self._last_t is not None else 0.0
        a_world = (v - self._last_v) / dt if dt > 1e-9 else np.zeros(3)
        self._last_v, self._last_t = v, now
        f_body = R.T @ (a_world - _G)             # accelerometer reads a - g
        w_body = R.T @ np.array(ang)              # body angular rate (rad/s)

        # --- GNSS solution quality + INS status, consistent with the fix ---
        sigma = {"nominal": 0.4, "degraded": 4.0, "denied": 0.0,
                 "spoofed": 0.4}.get(self.gnss_mode, 0.4)
        ins_mode = 2 if fix else 1                # 2=Tracking, 1=Aligning
        ins_raw = ins_mode | (int(fix) << 2) | (int(fix) << 8) | (int(fix) << 9)
        num_sats = 14 if fix else 0
        # ENU world velocity -> NED
        vel_n, vel_e, vel_d = float(v[1]), float(v[0]), float(-v[2])
        # barometric pressure from altitude (ISA troposphere), kPa
        pressure = 101.325 * (1.0 - 2.25577e-5 * alt) ** 5.2559
        # crude magnetic field from heading (Gauss), tilted dipole
        mag = R.T @ np.array([0.20 * math.cos(math.radians(yaw)),
                              -0.20 * math.sin(math.radians(yaw)), 0.44])
        tow = now % _GPS_WEEK_S

        return {
            "port": "sim",
            "gps_tow": round(tow, 3), "gps_week": int(now // _GPS_WEEK_S) - 311040,
            "ins_status_raw": int(ins_raw), "ins_mode": ins_mode, "ins_error": 0,
            "gnss_fix": fix, "gnss_compass_active": fix, "gnss_heading_aiding": fix,
            "yaw": math.degrees(yaw), "pitch": math.degrees(pitch),
            "roll": math.degrees(roll),
            "latitude": lat, "longitude": lon, "altitude": alt,
            "vel_north": vel_n, "vel_east": vel_e, "vel_down": vel_d,
            "att_uncertainty": 0.5, "pos_uncertainty": max(0.05, sigma),
            "vel_uncertainty": 0.1,
            "mag_x": float(mag[0]), "mag_y": float(mag[1]), "mag_z": float(mag[2]),
            "accel_x": float(f_body[0]), "accel_y": float(f_body[1]),
            "accel_z": float(f_body[2]),
            "gyro_x": float(w_body[0]), "gyro_y": float(w_body[1]),
            "gyro_z": float(w_body[2]),
            "gnss_num_sats": num_sats, "gnss_fix_type": 3 if fix else 0,
            "temperature": 25.0, "pressure": round(pressure, 3),
            "last_async_header": "VNINS", "messages_parsed": 0, "messages_dropped": 0,
            "gnss_mode": self.gnss_mode, "timestamp_ns": _now_ns(),
        }

    def apply(self, payload: dict, state: RobotState) -> None:
        lat = payload.get("latitude")
        if lat is None:
            return
        lat0, lon0 = self.datum["lat"], self.datum["lon"]
        y = math.radians(lat - lat0) * _EARTH_R
        x = math.radians(payload["longitude"] - lon0) * _EARTH_R * math.cos(math.radians(lat0))
        z = payload.get("altitude", self.datum["alt"]) - self.datum["alt"]
        orn = p.getQuaternionFromEuler([math.radians(payload.get("roll", 0.0)),
                                        math.radians(payload.get("pitch", 0.0)),
                                        math.radians(payload.get("yaw", 0.0))])
        state.base_pose = ([x, y, z], list(orn))


class Battery:
    """Shared pack model: drains SoC from the total motor current draw, exposes a
    SoC-sagged bus voltage the ODrives report. Plain model, good enough for L0
    battery/brownout reflexes; calibrate the curve against the real pack later."""

    def __init__(self, nominal_v=48.0, capacity_ah=20.0, soc=1.0):
        self.nominal_v = float(nominal_v)
        self.capacity_as = float(capacity_ah) * 3600.0
        self.soc = float(soc)
        self.pack_current = 0.0
        self._last_t = None

    def drain(self, total_current_a: float) -> None:
        now = time.time()
        dt = (now - self._last_t) if self._last_t is not None else 0.0
        self._last_t = now
        self.pack_current = float(total_current_a)
        self.soc = max(0.0, self.soc - total_current_a * dt / self.capacity_as)

    @property
    def voltage(self) -> float:
        # ~linear sag: full at nominal, ~-15% empty, minus IR droop
        return self.nominal_v * (0.85 + 0.15 * self.soc) - 0.02 * self.pack_current


class OdriveDevice:
    """One ODrive axis driving a track side. Physics-derived current (from the
    track-force model), an I^2R thermal model, vel estimate, and a stuck/error
    flag -- the stream L0's thermal/current/stuck reflexes consume."""

    def __init__(self, robot, channel, side, tracks=None, battery=None,
                 ambient_c=25.0, flipper_jidx=None):
        self.robot = robot
        self.body = robot.body_id
        self.channel = channel
        self.node_id = int(channel.split("_")[-1]) if "_" in channel else 0
        self.side = side                  # "L" | "R" (drums); flipper key for flippers
        self.tracks = tracks
        self.battery = battery
        self.temp = float(ambient_c)
        self.ambient = float(ambient_c)
        self._last_t = None
        self._pos_rev = 0.0               # integrated encoder position
        # When set, this ODrive drives a FLIPPER (node 41-44): telemetry comes
        # from the flipper joint, not the track-force model.
        self.flipper_jidx = flipper_jidx

    def sample(self) -> dict:
        lim = self.tracks.current_limit_a if self.tracks else 7.2
        now = time.time()
        dt = (now - self._last_t) if self._last_t is not None else 0.0
        self._last_t = now
        if self.flipper_jidx is not None:
            # flipper ODrive: current/vel from the flipper joint (worm gear holds
            # position -> a small idle current + load-proportional draw).
            st = p.getJointState(self.body, self.flipper_jidx)
            vel_rad = abs(st[1])
            cur = min(lim, 0.4 + abs(st[3]) * 0.02)
        else:
            cur = self.tracks.side_current_a()[self.side] if self.tracks else 0.0
            lin, _ = p.getBaseVelocity(self.body)
            speed = math.hypot(lin[0], lin[1])
            drum_r = self.tracks.drum_radius if self.tracks else 0.0899
            vel_rad = speed / drum_r
        # I^2R heating vs Newton cooling toward ambient
        self.temp += dt * (0.6 * cur * cur - 0.05 * (self.temp - self.ambient))
        vel_rev = vel_rad / (2.0 * math.pi)
        self._pos_rev += vel_rev * dt
        stuck = bool(cur > 0.8 * lim and vel_rad < 0.05)
        vbus = self.battery.voltage if self.battery else 48.0
        torque = round(cur * 0.083, 4)            # ~Kt for the drive motors
        # Emit the EXACT real OdriveNodeState schema so the Rust mock is a
        # verbatim passthrough. A stuck axis surfaces as a nonzero error flag
        # (real schema has no "stuck" field; L0 reads axis_error/active_errors).
        err = 0x40 if stuck else 0
        return {
            "node_id": self.node_id, "axis_error": err, "axis_state": 8,
            "procedure_result": 0, "trajectory_done": True,
            "pos_estimate": round(self._pos_rev, 4), "vel_estimate": round(vel_rev, 4),
            "shadow_count": int(self._pos_rev * 8192), "count_cpr": 8192,
            "iq_setpoint": round(cur, 3), "iq_measured": round(cur, 3),
            "bus_voltage": round(vbus, 2), "bus_current": round(cur, 3),
            "active_errors": err, "disarm_reason": 0,
            "torque_target": torque, "torque_estimate": torque,
            "electrical_power": round(vbus * cur, 2),
            "mechanical_power": round(torque * vel_rad, 2),
            "fet_temp": round(self.temp, 1), "motor_temp": round(self.temp - 4, 1),
            "timestamp_ns": _now_ns(),
        }

    def apply(self, payload: dict, state: RobotState) -> None:
        pass                              # informational; base pose syncs off VN-300


class PmicDevice:
    """Battery/PMIC channel: SoC, voltage, pack current. Drains the shared Battery
    from the summed ODrive currents (+ a base electronics load)."""
    channel = "pmic"

    def __init__(self, robot, battery, odrives, base_load_a=2.0):
        self.robot = robot
        self.battery = battery
        self.odrives = odrives
        self.base_load = float(base_load_a)

    def sample(self) -> dict:
        total = self.base_load + sum(
            (o.tracks.side_current_a()[o.side] if o.tracks else 0.0) for o in self.odrives)
        self.battery.drain(total)
        return {"bus_voltage": round(self.battery.voltage, 2),
                "pack_current": round(total, 2),
                "soc": round(self.battery.soc, 4),
                "temp": 30.0, "timestamp_ns": _now_ns()}

    def apply(self, payload: dict, state: RobotState) -> None:
        pass


def build_devices(profile, robot, actuators=None) -> list:
    """Auto-wire telemetry devices from what the profile declares. `actuators`
    (the live actuator list) enables the physics-derived ODrive/Pmic telemetry;
    omit it on the consumer side (apply-only) where physics isn't sampled."""
    devices: list = []
    idx_to_name = {v: k for k, v in robot.joint_index.items()}
    datum = (profile.raw.get("world", {}).get("terrain", {}) or {}).get("datum")
    for a in profile.actuators:
        if a.use == "arm_ik":
            names = [idx_to_name[robot.movable_joint[link]]
                     for link in a.bind if link in robot.movable_joint]
            devices.append(KinovaArmDevice(robot, names))
        elif a.use in ("mimic_gripper", "robotiq_2f140"):
            pr = a.params
            devices.append(RobotiqGripperDevice(
                robot, pr["driven_joint"], pr["closed_rad"], pr.get("mimic", {})))
    if any(s.use in ("vn300", "vectornav") for s in profile.sensors):
        devices.append(VectorNavDevice(robot, datum))
    # 8 ODrives: 4 drums (track sides) + 4 flippers. Drums need the live tracks
    # actuator (physics current); flippers read their own joint. Per the
    # rove_control_bridge config: 31,34 = LEFT, 32,33 = RIGHT.
    tracks = next((a for a in (actuators or []) if getattr(a, "intent_field", "") == "tracks"), None)
    battery = Battery()
    odrv = [OdriveDevice(robot, "odrive_31", "L", tracks, battery),
            OdriveDevice(robot, "odrive_32", "R", tracks, battery),
            OdriveDevice(robot, "odrive_33", "R", tracks, battery),
            OdriveDevice(robot, "odrive_34", "L", tracks, battery)]
    # flippers: node 41-44 -> FlipperFL/FR/BL/BR. The movable joint per flipper
    # link carries its telemetry.
    flipper_links = [("odrive_41", "FlipperFL"), ("odrive_42", "FlipperFR"),
                     ("odrive_43", "FlipperBL"), ("odrive_44", "FlipperBR")]
    mj = getattr(robot, "movable_joint", {})
    for chan, link in flipper_links:
        jidx = mj.get(link)
        odrv.append(OdriveDevice(robot, chan, link, tracks=None, battery=battery,
                                 flipper_jidx=jidx))
    devices += odrv
    devices.append(PmicDevice(robot, battery, odrv))
    return devices
