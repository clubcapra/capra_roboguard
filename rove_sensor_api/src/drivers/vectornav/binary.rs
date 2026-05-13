//! VectorNav binary output protocol decoder.
//!
//! Frames look like:
//! ```text
//! 0xFA <group-bitmap:u8> [<field-bitmap:u16le>]+ <payload> <crc:u16be>
//! ```
//! `field-bitmap` is repeated for each set bit in `group-bitmap` (lowest first).
//! The CRC is CRC-16/CCITT-FALSE (poly 0x1021, init 0x0000) computed from the
//! group-bitmap byte through the end of the payload.
//!
//! We currently configure the device to emit Common-group frames only, so the
//! decoder only sizes/decodes that group. Frames with other groups set are
//! treated as misalignment and trigger a one-byte resync.
//!
//! Reference: VN-300 User Manual §4.2 "Binary Output Messages".

use super::state::VectorNavState;

pub const SYNC_BYTE: u8 = 0xFA;

pub const GROUP_COMMON: u8 = 1 << 0;

pub const COMMON_TIME_STARTUP: u16 = 1 << 0;
pub const COMMON_TIME_GPS: u16 = 1 << 1;
pub const COMMON_TIME_SYNC_IN: u16 = 1 << 2;
pub const COMMON_YPR: u16 = 1 << 3;
pub const COMMON_QUATERNION: u16 = 1 << 4;
pub const COMMON_ANGULAR_RATE: u16 = 1 << 5;
pub const COMMON_POSITION: u16 = 1 << 6;
pub const COMMON_VELOCITY: u16 = 1 << 7;
pub const COMMON_ACCEL: u16 = 1 << 8;
pub const COMMON_IMU: u16 = 1 << 9;
pub const COMMON_MAG_PRES: u16 = 1 << 10;
pub const COMMON_DELTA_THETA: u16 = 1 << 11;
pub const COMMON_INS_STATUS: u16 = 1 << 12;
pub const COMMON_SYNC_IN_CNT: u16 = 1 << 13;
pub const COMMON_TIME_GPS_PPS: u16 = 1 << 14;

/// Common-group fields the driver requests on connect: GPS time, attitude,
/// gyro, position, velocity, accel, mag/temp/pres, INS status — everything
/// the existing data schema cares about, in one frame per tick.
pub const COMMON_DEFAULT_FIELDS: u16 = COMMON_TIME_GPS
    | COMMON_YPR
    | COMMON_ANGULAR_RATE
    | COMMON_POSITION
    | COMMON_VELOCITY
    | COMMON_ACCEL
    | COMMON_MAG_PRES
    | COMMON_INS_STATUS;

#[derive(Debug)]
pub enum FrameStep {
    /// A complete frame was consumed (n bytes from the front of the buffer).
    Consumed(usize),
    /// Buffer started misaligned; n bytes should be dropped before retrying.
    Resync(usize),
    /// Need more bytes before a decision is possible.
    NeedMore,
}

/// CRC-16/XMODEM: poly 0x1021, init 0x0000, no reflection, no xorout.
/// (Same parameters the VN-300 uses for binary-frame integrity.)
pub fn crc16(data: &[u8]) -> u16 {
    let mut crc: u16 = 0;
    for &b in data {
        let mut x = ((crc >> 8) as u8 ^ b) as u16;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    }
    crc
}

/// Try to consume one binary frame from the start of `buf`.
///
/// On `Consumed`, `state` has been updated with whichever fields the frame
/// contained. The caller is expected to have already verified `buf[0] ==
/// SYNC_BYTE`; if not, we return `Resync(1)` so the caller can drop one byte
/// and try again.
pub fn try_consume(buf: &[u8], state: &mut VectorNavState) -> FrameStep {
    if buf.is_empty() {
        return FrameStep::NeedMore;
    }
    if buf[0] != SYNC_BYTE {
        return FrameStep::Resync(1);
    }
    if buf.len() < 2 {
        return FrameStep::NeedMore;
    }
    let group_mask = buf[1];
    let n_groups = group_mask.count_ones() as usize;
    let header_len = 2 + 2 * n_groups;
    if buf.len() < header_len {
        return FrameStep::NeedMore;
    }

    // Decode field bitmaps in group-bit order (lowest first).
    let mut field_masks = [0u16; 8];
    let mut bm_idx = 0;
    for bit in 0..7u8 {
        if group_mask & (1 << bit) != 0 {
            let off = 2 + 2 * bm_idx;
            field_masks[bit as usize] = u16::from_le_bytes([buf[off], buf[off + 1]]);
            bm_idx += 1;
        }
    }

    // Sized payload — we only know how to size the Common group. Anything
    // else is most likely a misaligned sync byte; resync one byte at a time.
    let payload_len = match payload_size(group_mask, &field_masks) {
        Some(n) => n,
        None => {
            tracing::trace!(group_mask, "VN binary: unsupported group, resyncing");
            return FrameStep::Resync(1);
        }
    };

    let total_len = header_len + payload_len + 2;
    if buf.len() < total_len {
        return FrameStep::NeedMore;
    }

    let crc_input = &buf[1..header_len + payload_len];
    let want = crc16(crc_input);
    let got = u16::from_be_bytes([buf[total_len - 2], buf[total_len - 1]]);
    if want != got {
        // Misaligned sync — drop one byte and let the caller retry from the
        // next 0xFA on subsequent reads.
        return FrameStep::Resync(1);
    }

    let payload = &buf[header_len..header_len + payload_len];
    if group_mask & GROUP_COMMON != 0 {
        decode_common(field_masks[0], payload, state);
        state.last_async_header = "VNBIN".to_string();
    }

    FrameStep::Consumed(total_len)
}

fn payload_size(group_mask: u8, field_masks: &[u16; 8]) -> Option<usize> {
    if group_mask & !GROUP_COMMON != 0 {
        return None;
    }
    if group_mask & GROUP_COMMON != 0 {
        return Some(common_payload_size(field_masks[0]));
    }
    Some(0)
}

fn common_payload_size(mask: u16) -> usize {
    const SIZES: [usize; 15] = [
        8,  // TimeStartup
        8,  // TimeGps
        8,  // TimeSyncIn
        12, // YPR
        16, // Quaternion
        12, // AngularRate
        24, // Position (LLA, 3x f64)
        12, // Velocity (NED)
        12, // Accel
        24, // Imu (uncompMag + uncompAccel)
        20, // MagPres (mag + temp + pres)
        28, // DeltaThetaVel
        2,  // InsStatus
        4,  // SyncInCnt
        8,  // TimeGpsPps
    ];
    let mut sum = 0;
    for (bit, size) in SIZES.iter().enumerate() {
        if mask & (1 << bit) != 0 {
            sum += size;
        }
    }
    sum
}

#[inline]
fn read_f32(buf: &[u8], off: usize) -> f32 {
    f32::from_le_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

#[inline]
fn read_f64(buf: &[u8], off: usize) -> f64 {
    f64::from_le_bytes([
        buf[off],
        buf[off + 1],
        buf[off + 2],
        buf[off + 3],
        buf[off + 4],
        buf[off + 5],
        buf[off + 6],
        buf[off + 7],
    ])
}

#[inline]
fn read_u16(buf: &[u8], off: usize) -> u16 {
    u16::from_le_bytes([buf[off], buf[off + 1]])
}

#[inline]
fn read_u64(buf: &[u8], off: usize) -> u64 {
    u64::from_le_bytes([
        buf[off],
        buf[off + 1],
        buf[off + 2],
        buf[off + 3],
        buf[off + 4],
        buf[off + 5],
        buf[off + 6],
        buf[off + 7],
    ])
}

/// Decode a Common-group payload, writing into `state`. Field order in the
/// payload matches bit order (lowest first).
fn decode_common(mask: u16, payload: &[u8], state: &mut VectorNavState) {
    let mut o = 0usize;

    if mask & COMMON_TIME_STARTUP != 0 {
        o += 8;
    }
    if mask & COMMON_TIME_GPS != 0 {
        // gps_tow is exposed in seconds for parity with the $VNINS schema.
        let ns = read_u64(payload, o);
        state.gps_tow = ns as f64 * 1e-9;
        o += 8;
    }
    if mask & COMMON_TIME_SYNC_IN != 0 {
        o += 8;
    }
    if mask & COMMON_YPR != 0 {
        state.yaw = read_f32(payload, o);
        state.pitch = read_f32(payload, o + 4);
        state.roll = read_f32(payload, o + 8);
        o += 12;
    }
    if mask & COMMON_QUATERNION != 0 {
        o += 16;
    }
    if mask & COMMON_ANGULAR_RATE != 0 {
        state.gyro_x = read_f32(payload, o);
        state.gyro_y = read_f32(payload, o + 4);
        state.gyro_z = read_f32(payload, o + 8);
        o += 12;
    }
    if mask & COMMON_POSITION != 0 {
        state.latitude = read_f64(payload, o);
        state.longitude = read_f64(payload, o + 8);
        state.altitude = read_f64(payload, o + 16);
        o += 24;
    }
    if mask & COMMON_VELOCITY != 0 {
        state.vel_north = read_f32(payload, o);
        state.vel_east = read_f32(payload, o + 4);
        state.vel_down = read_f32(payload, o + 8);
        o += 12;
    }
    if mask & COMMON_ACCEL != 0 {
        state.accel_x = read_f32(payload, o);
        state.accel_y = read_f32(payload, o + 4);
        state.accel_z = read_f32(payload, o + 8);
        o += 12;
    }
    if mask & COMMON_IMU != 0 {
        // Uncompensated mag+accel — we already capture the compensated values
        // via Accel and MagPres, so skip the raw pair.
        o += 24;
    }
    if mask & COMMON_MAG_PRES != 0 {
        state.mag_x = read_f32(payload, o);
        state.mag_y = read_f32(payload, o + 4);
        state.mag_z = read_f32(payload, o + 8);
        state.temperature = read_f32(payload, o + 12);
        state.pressure = read_f32(payload, o + 16);
        o += 20;
    }
    if mask & COMMON_DELTA_THETA != 0 {
        o += 28;
    }
    if mask & COMMON_INS_STATUS != 0 {
        let raw = read_u16(payload, o);
        state.apply_ins_status(raw);
        o += 2;
    }
    let _ = o;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crc_known_vector() {
        // Standard CRC-16/XMODEM check vector for "123456789".
        assert_eq!(crc16(b"123456789"), 0x31C3);
    }

    #[test]
    fn round_trip_common_frame() {
        let mask: u16 = COMMON_TIME_GPS | COMMON_YPR | COMMON_INS_STATUS;
        let mut payload = Vec::new();
        payload.extend_from_slice(&123_456_789_000u64.to_le_bytes()); // TimeGps ns
        payload.extend_from_slice(&10.0f32.to_le_bytes()); // yaw
        payload.extend_from_slice(&(-5.0f32).to_le_bytes()); // pitch
        payload.extend_from_slice(&1.0f32.to_le_bytes()); // roll
        payload.extend_from_slice(&0x0204u16.to_le_bytes()); // INS status

        let mut frame = vec![SYNC_BYTE, GROUP_COMMON];
        frame.extend_from_slice(&mask.to_le_bytes());
        frame.extend_from_slice(&payload);
        let crc = crc16(&frame[1..]);
        frame.extend_from_slice(&crc.to_be_bytes());

        let mut s = VectorNavState::default();
        match try_consume(&frame, &mut s) {
            FrameStep::Consumed(n) => assert_eq!(n, frame.len()),
            other => panic!("expected Consumed, got {other:?}"),
        }
        assert!((s.yaw - 10.0).abs() < 1e-6);
        assert!((s.pitch - -5.0).abs() < 1e-6);
        assert!((s.roll - 1.0).abs() < 1e-6);
        // 123_456_789_000 ns → 123.456789 s
        assert!((s.gps_tow - 123.456_789).abs() < 1e-6);
        assert!(s.gnss_fix); // INS status bit 2
        assert!(s.gnss_compass_active); // INS status bit 9
    }

    #[test]
    fn need_more_when_truncated() {
        let frame = vec![SYNC_BYTE, GROUP_COMMON, 0x00];
        let mut s = VectorNavState::default();
        assert!(matches!(try_consume(&frame, &mut s), FrameStep::NeedMore));
    }

    #[test]
    fn bad_crc_resyncs() {
        // Common group, no fields, bogus CRC.
        let frame = vec![SYNC_BYTE, GROUP_COMMON, 0x00, 0x00, 0xDE, 0xAD];
        let mut s = VectorNavState::default();
        assert!(matches!(try_consume(&frame, &mut s), FrameStep::Resync(1)));
    }

    #[test]
    fn unsupported_group_resyncs() {
        // Group bit 1 (Time) set — we don't size that group.
        let frame = vec![SYNC_BYTE, 0b0000_0010, 0x00, 0x00, 0x00, 0x00];
        let mut s = VectorNavState::default();
        assert!(matches!(try_consume(&frame, &mut s), FrameStep::Resync(1)));
    }
}
