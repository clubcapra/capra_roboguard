"""M5: sim backs the real rove_sensor_api Rust binary.

The Rust mock drivers are verbatim passthroughs of the sim's telemetry, so the
sim must emit the EXACT field set each real driver's `data_schema()` declares.
These tests pin that contract (real keys ⊆ sim keys), the native Livox IMU codec,
and the shared port map.

    ../rove_sim_venv/bin/python -m pytest tests/test_m5_sim_backend.py -q
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from rove_sim import runtime
from rove_sim.api.sim_sensor_api import SimSensorApi
from rove_sim.transport import InProcessTransport
from rove_sim.transport.ports import PortMap
from rove_sim.sensors.lidar import encode_imu_packet, decode_imu_packet

# The real rove_sensor_api driver data_schema() field names (state.rs).
VECTORNAV_KEYS = {
    "port", "gps_tow", "gps_week", "ins_status_raw", "ins_mode", "ins_error",
    "gnss_fix", "gnss_compass_active", "gnss_heading_aiding", "yaw", "pitch",
    "roll", "latitude", "longitude", "altitude", "vel_north", "vel_east",
    "vel_down", "att_uncertainty", "pos_uncertainty", "vel_uncertainty",
    "mag_x", "mag_y", "mag_z", "accel_x", "accel_y", "accel_z", "gyro_x",
    "gyro_y", "gyro_z", "gnss_num_sats", "gnss_fix_type", "temperature",
    "pressure", "last_async_header", "messages_parsed", "messages_dropped",
    "timestamp_ns",
}
ODRIVE_KEYS = {
    "node_id", "axis_error", "axis_state", "procedure_result", "trajectory_done",
    "pos_estimate", "vel_estimate", "shadow_count", "count_cpr", "iq_setpoint",
    "iq_measured", "bus_voltage", "bus_current", "active_errors", "disarm_reason",
    "torque_target", "torque_estimate", "electrical_power", "mechanical_power",
    "fet_temp", "motor_temp", "timestamp_ns",
}
ROBOTIQ_KEYS = {
    "activated", "going_to_position", "status", "object_status", "fault",
    "position_request_echo", "position", "current_raw", "current_a", "link_up",
    "timestamp_ns",
}
KINOVA_KEYS = ({f"joint_{i}_{f}" for i in range(1, 7)
                for f in ("pos", "vel", "current", "temp")}
               | {"bus_voltage", "bus_current", "accel_x", "accel_y", "accel_z",
                  "timestamp_ns"})


def test_sim_emits_full_real_schema():
    """Every real driver schema field must be present in the sim's payload, so
    the Rust mock can serve it byte-for-byte."""
    sim = runtime.build("standard", mode="headless", world="mock")
    try:
        bus = InProcessTransport(bus="t_m5")
        api = SimSensorApi(sim.robot, sim.robot.profile, transport=bus,
                           actuators=sim.actuators)
        api.publish()
        assert VECTORNAV_KEYS <= set(bus.latest("vectornav"))
        # 8 ODrives: 4 drums (31-34) + 4 flippers (41-44), all the real schema.
        for n in (31, 32, 33, 34, 41, 42, 43, 44):
            od = bus.latest(f"odrive_{n}")
            assert od is not None and ODRIVE_KEYS <= set(od), f"odrive_{n}"
            assert od["node_id"] == n
        assert ROBOTIQ_KEYS <= set(bus.latest("robotiq"))
        assert KINOVA_KEYS <= set(bus.latest("kinova"))
    finally:
        sim.disconnect()


def test_vectornav_latlon_moves_with_pose():
    """Driving the robot must move lat/lon in the right direction (ENU→geodetic)."""
    import pybullet as p
    sim = runtime.build("standard", mode="headless", world="mock")
    try:
        bus = InProcessTransport(bus="t_ll")
        api = SimSensorApi(sim.robot, sim.robot.profile, transport=bus,
                           actuators=sim.actuators)
        api.publish()
        a = bus.latest("vectornav")
        # teleport +100 m east (+x), +50 m north (+y)
        (x, y, z), orn = p.getBasePositionAndOrientation(sim.robot.body_id)
        p.resetBasePositionAndOrientation(sim.robot.body_id, [x + 100, y + 50, z], orn)
        # nominal GNSS noise is ~0.4 m, far below the 100/50 m move
        vn = next(d for d in api.devices if d.channel == "vectornav")
        vn.gnss_mode = "nominal"
        api.publish()
        b = bus.latest("vectornav")
        assert b["longitude"] > a["longitude"]     # east -> +lon
        assert b["latitude"] > a["latitude"]       # north -> +lat
    finally:
        sim.disconnect()


def test_livox_imu_packet_roundtrip_and_layout():
    gyro = np.array([0.1, -0.2, 0.3])
    accel = np.array([0.0, 0.0, 1.0])
    ts = 1234567890123456789
    buf = encode_imu_packet(gyro, accel, ts)
    assert len(buf) == 60                          # 36-byte header + 24-byte payload
    assert buf[10] == 0                             # data_type == IMU at offset 10
    g, a, t = decode_imu_packet(buf)
    assert np.allclose(g, gyro) and np.allclose(a, accel) and t == ts


def test_port_map_backend_ports():
    pm = PortMap(None)                              # defaults
    bp = pm.backend_telemetry_ports()
    assert bp["vectornav"] == 6000 and bp["odrive_31"] == 6006
    assert bp["odrive_41"] == 6014 and bp["odrive_44"] == 6020  # flippers
    assert bp["pmic"] == 6022 and bp["control"] == 5100   # control clear of served 5000-5021
    # a non-default base shifts the whole block (overlap avoidance)
    pm2 = PortMap({"backend_base": 7000})
    assert pm2.backend_telemetry_ports()["vectornav"] == 7000
