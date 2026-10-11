/// Cached telemetry for a VectorNav VN-300, updated by the serial receive loop.
///
/// Field availability depends on which async output messages the sensor is
/// configured to send. `last_message_*` flags indicate which fields are fresh.
/// All angles are in degrees, distances in meters, velocities in m/s.
#[derive(Debug, Clone, Default)]
pub struct VectorNavState {
    // ── INS Solution LLA ($VNINS) ──────────────────────────────────────────
    /// GPS time of week in seconds.
    pub gps_tow: f64,
    /// GPS week number.
    pub gps_week: u16,
    /// Raw INS status bitfield (16-bit hex from the message).
    pub ins_status_raw: u16,
    /// Decoded INS mode (0=NotTracking, 1=Aligning, 2=Tracking, 3=LossOfGNSS).
    pub ins_mode: u8,
    /// True when the GNSS receiver has a valid 3D fix.
    pub gnss_fix: bool,
    /// True when the GNSS compass is operational and reporting heading.
    pub gnss_compass_active: bool,
    /// True when the INS heading is currently being aided by the GNSS compass.
    pub gnss_heading_aiding: bool,
    /// Sensor error code from INS status (bits 3..6). 0 = no error.
    pub ins_error: u8,

    /// Yaw angle (degrees, range -180..180; in NED frame, true heading).
    pub yaw: f32,
    /// Pitch angle (degrees, range -90..90).
    pub pitch: f32,
    /// Roll angle (degrees, range -180..180).
    pub roll: f32,

    /// Latitude (degrees, WGS84).
    pub latitude: f64,
    /// Longitude (degrees, WGS84).
    pub longitude: f64,
    /// Altitude above WGS84 ellipsoid (meters).
    pub altitude: f64,

    /// Velocity North (m/s).
    pub vel_north: f32,
    /// Velocity East (m/s).
    pub vel_east: f32,
    /// Velocity Down (m/s).
    pub vel_down: f32,

    /// Attitude uncertainty (1-sigma, degrees).
    pub att_uncertainty: f32,
    /// Position uncertainty (1-sigma, meters).
    pub pos_uncertainty: f32,
    /// Velocity uncertainty (1-sigma, m/s).
    pub vel_uncertainty: f32,

    // ── IMU ($VNYMR / $VNIMU) ──────────────────────────────────────────────
    /// Compensated magnetic field (Gauss, body frame).
    pub mag_x: f32,
    pub mag_y: f32,
    pub mag_z: f32,
    /// Compensated linear acceleration (m/s², body frame, includes gravity).
    pub accel_x: f32,
    pub accel_y: f32,
    pub accel_z: f32,
    /// Compensated angular rate (rad/s, body frame).
    pub gyro_x: f32,
    pub gyro_y: f32,
    pub gyro_z: f32,

    // ── GNSS Solution ($VNGPS) ─────────────────────────────────────────────
    /// Number of satellites used in the GNSS solution.
    pub gnss_num_sats: u8,
    /// GNSS fix type (0=NoFix, 1=TimeOnly, 2=2D, 3=3D).
    pub gnss_fix_type: u8,

    // ── Environment (binary MagPres group) ─────────────────────────────────
    /// IMU temperature (°C).
    pub temperature: f32,
    /// Barometric pressure (kPa).
    pub pressure: f32,

    // ── Bookkeeping ────────────────────────────────────────────────────────
    /// Header of the most recently parsed async message (e.g. `"VNINS"`).
    pub last_async_header: String,
    /// Unix timestamp (ns) of the most recent async message received.
    /// 0 until the first message arrives.
    pub timestamp_ns: i64,
    /// Number of async messages successfully parsed since startup.
    pub messages_parsed: u64,
    /// Number of malformed / failed-checksum lines since startup.
    pub messages_dropped: u64,
}

impl VectorNavState {
    /// Decode the 16-bit INS status bitfield into individual flags.
    /// Layout (per VN-300 manual, INS Solution LLA register):
    ///   bits 0-1: Mode
    ///   bit  2:   GnssFix
    ///   bits 3-6: Error
    ///   bit  7:   Reserved
    ///   bit  8:   GnssHeadingIns
    ///   bit  9:   GnssCompass
    pub fn apply_ins_status(&mut self, raw: u16) {
        self.ins_status_raw = raw;
        self.ins_mode = (raw & 0b11) as u8;
        self.gnss_fix = (raw & (1 << 2)) != 0;
        self.ins_error = ((raw >> 3) & 0b1111) as u8;
        self.gnss_heading_aiding = (raw & (1 << 8)) != 0;
        self.gnss_compass_active = (raw & (1 << 9)) != 0;
    }
}
