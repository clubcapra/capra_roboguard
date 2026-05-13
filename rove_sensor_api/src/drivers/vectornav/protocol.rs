//! VectorNav ASCII protocol helpers.
//!
//! All async messages are framed:
//! ```text
//! $VN<3-char-header>,<comma-separated fields>*<2-hex-checksum>\r\n
//! ```
//! where the 8-bit checksum is the XOR of every byte between `$` and `*`.
//!
//! The string `*XX` is accepted in lieu of a real checksum (so that humans
//! can hand-type commands in a serial terminal), so we mirror that on output.
//!
//! References: VN-300 User Manual §3.5.1 (Serial ASCII), §3.8 (Checksum/CRC).

use super::state::VectorNavState;

/// Compute the 8-bit XOR checksum of all bytes between (but not including)
/// the leading `$` and the trailing `*`.
pub fn checksum8(payload: &[u8]) -> u8 {
    payload.iter().fold(0u8, |acc, &b| acc ^ b)
}

/// Format a fully-framed VN command from a comma-separated body.
/// Example: `format_command("VNTAR")` → `"$VNTAR*5F\r\n"`.
pub fn format_command(body: &str) -> String {
    let cs = checksum8(body.as_bytes());
    format!("${body}*{cs:02X}\r\n")
}

/// Result of parsing one ASCII line received from the sensor.
#[derive(Debug, Clone)]
pub enum ParsedMessage<'a> {
    /// Async data message we successfully decoded into shared state.
    Async(&'a str),
    /// Non-async response (e.g. `$VNRRG,01,VN-310`). We don't currently use these
    /// but they're surfaced for logging and future correlation.
    Response { header: &'a str, fields: Vec<&'a str> },
    /// Error response (`$VNERR,N`).
    Error(u8),
    /// Garbage / failed-checksum line — caller bumps the dropped counter.
    Invalid(&'static str),
    /// Empty line or unknown header — silently ignored.
    Ignored,
}

/// Parse one trimmed line and (for async messages) update `state` in place.
///
/// Returns `ParsedMessage` so the caller can update bookkeeping counters.
/// The line must start with `$VN`; anything else is `Ignored`.
pub fn parse_line<'a>(line: &'a str, state: &mut VectorNavState) -> ParsedMessage<'a> {
    if !line.starts_with("$VN") {
        return ParsedMessage::Ignored;
    }

    // Validate checksum if present.
    let body = match line.find('*') {
        None => return ParsedMessage::Invalid("missing '*' before checksum"),
        Some(idx) => {
            let payload = &line[1..idx]; // between '$' and '*'
            let trailer = &line[idx + 1..];
            // Only validate when the trailer looks like real hex; allow XX bypass.
            if trailer.len() >= 2 && &trailer[..2] != "XX" {
                if let Ok(expected) = u8::from_str_radix(&trailer[..2], 16) {
                    if checksum8(payload.as_bytes()) != expected {
                        return ParsedMessage::Invalid("checksum mismatch");
                    }
                }
            }
            payload
        }
    };

    let mut fields = body.split(',');
    let header = match fields.next() {
        Some(h) => h, // includes the "VN" prefix, e.g. "VNINS"
        None => return ParsedMessage::Invalid("empty body"),
    };
    let rest: Vec<&str> = fields.collect();

    match header {
        "VNINS" => {
            if parse_vnins(&rest, state).is_some() {
                state.last_async_header = header.to_string();
                ParsedMessage::Async(header)
            } else {
                ParsedMessage::Invalid("malformed $VNINS")
            }
        }
        "VNYMR" => {
            if parse_vnymr(&rest, state).is_some() {
                state.last_async_header = header.to_string();
                ParsedMessage::Async(header)
            } else {
                ParsedMessage::Invalid("malformed $VNYMR")
            }
        }
        "VNGPS" => {
            if parse_vngps(&rest, state).is_some() {
                state.last_async_header = header.to_string();
                ParsedMessage::Async(header)
            } else {
                ParsedMessage::Invalid("malformed $VNGPS")
            }
        }
        "VNYPR" => {
            if parse_vnypr(&rest, state).is_some() {
                state.last_async_header = header.to_string();
                ParsedMessage::Async(header)
            } else {
                ParsedMessage::Invalid("malformed $VNYPR")
            }
        }
        "VNIMU" => {
            if parse_vnimu(&rest, state).is_some() {
                state.last_async_header = header.to_string();
                ParsedMessage::Async(header)
            } else {
                ParsedMessage::Invalid("malformed $VNIMU")
            }
        }
        "VNERR" => {
            let code = rest
                .first()
                .and_then(|s| s.trim().parse::<u8>().ok())
                .unwrap_or(0);
            ParsedMessage::Error(code)
        }
        // RRG/WRG/etc. — treat as responses for now, no correlation.
        h if h.len() >= 3 => ParsedMessage::Response {
            header: h,
            fields: rest,
        },
        _ => ParsedMessage::Ignored,
    }
}

// ── Per-message decoders ────────────────────────────────────────────────────
//
// All return Option<()>: Some(()) on success, None if the field count is short
// or any expected number fails to parse. We deliberately keep these strict so
// a malformed line is never silently treated as a valid update.

/// `$VNINS,time,week,status,yaw,pitch,roll,lat,lon,alt,vN,vE,vD,attUnc,posUnc,velUnc`
fn parse_vnins(f: &[&str], state: &mut VectorNavState) -> Option<()> {
    if f.len() < 15 {
        return None;
    }
    let gps_tow = f[0].trim().parse::<f64>().ok()?;
    let gps_week = f[1].trim().parse::<u16>().ok()?;
    let status = u16::from_str_radix(f[2].trim(), 16).ok()?;
    let yaw = f[3].trim().parse::<f32>().ok()?;
    let pitch = f[4].trim().parse::<f32>().ok()?;
    let roll = f[5].trim().parse::<f32>().ok()?;
    let lat = f[6].trim().parse::<f64>().ok()?;
    let lon = f[7].trim().parse::<f64>().ok()?;
    let alt = f[8].trim().parse::<f64>().ok()?;
    let vn = f[9].trim().parse::<f32>().ok()?;
    let ve = f[10].trim().parse::<f32>().ok()?;
    let vd = f[11].trim().parse::<f32>().ok()?;
    let att_u = f[12].trim().parse::<f32>().ok()?;
    let pos_u = f[13].trim().parse::<f32>().ok()?;
    let vel_u = f[14].trim().parse::<f32>().ok()?;

    state.gps_tow = gps_tow;
    state.gps_week = gps_week;
    state.apply_ins_status(status);
    state.yaw = yaw;
    state.pitch = pitch;
    state.roll = roll;
    state.latitude = lat;
    state.longitude = lon;
    state.altitude = alt;
    state.vel_north = vn;
    state.vel_east = ve;
    state.vel_down = vd;
    state.att_uncertainty = att_u;
    state.pos_uncertainty = pos_u;
    state.vel_uncertainty = vel_u;
    Some(())
}

/// `$VNYMR,yaw,pitch,roll,magX,magY,magZ,accX,accY,accZ,gyroX,gyroY,gyroZ`
fn parse_vnymr(f: &[&str], state: &mut VectorNavState) -> Option<()> {
    if f.len() < 12 {
        return None;
    }
    let yaw = f[0].trim().parse::<f32>().ok()?;
    let pitch = f[1].trim().parse::<f32>().ok()?;
    let roll = f[2].trim().parse::<f32>().ok()?;
    let mx = f[3].trim().parse::<f32>().ok()?;
    let my = f[4].trim().parse::<f32>().ok()?;
    let mz = f[5].trim().parse::<f32>().ok()?;
    let ax = f[6].trim().parse::<f32>().ok()?;
    let ay = f[7].trim().parse::<f32>().ok()?;
    let az = f[8].trim().parse::<f32>().ok()?;
    let gx = f[9].trim().parse::<f32>().ok()?;
    let gy = f[10].trim().parse::<f32>().ok()?;
    let gz = f[11].trim().parse::<f32>().ok()?;

    state.yaw = yaw;
    state.pitch = pitch;
    state.roll = roll;
    state.mag_x = mx;
    state.mag_y = my;
    state.mag_z = mz;
    state.accel_x = ax;
    state.accel_y = ay;
    state.accel_z = az;
    state.gyro_x = gx;
    state.gyro_y = gy;
    state.gyro_z = gz;
    Some(())
}

/// `$VNYPR,yaw,pitch,roll`
fn parse_vnypr(f: &[&str], state: &mut VectorNavState) -> Option<()> {
    if f.len() < 3 {
        return None;
    }
    state.yaw = f[0].trim().parse().ok()?;
    state.pitch = f[1].trim().parse().ok()?;
    state.roll = f[2].trim().parse().ok()?;
    Some(())
}

/// `$VNIMU,magX,magY,magZ,accX,accY,accZ,gyroX,gyroY,gyroZ,temp,pres`
/// (Note: $VNIMU outputs *uncompensated* values; for parity with the rest of
/// the cached state we still write into the same fields. Switch back to
/// $VNYMR if you need bias-corrected data.)
fn parse_vnimu(f: &[&str], state: &mut VectorNavState) -> Option<()> {
    if f.len() < 9 {
        return None;
    }
    state.mag_x = f[0].trim().parse().ok()?;
    state.mag_y = f[1].trim().parse().ok()?;
    state.mag_z = f[2].trim().parse().ok()?;
    state.accel_x = f[3].trim().parse().ok()?;
    state.accel_y = f[4].trim().parse().ok()?;
    state.accel_z = f[5].trim().parse().ok()?;
    state.gyro_x = f[6].trim().parse().ok()?;
    state.gyro_y = f[7].trim().parse().ok()?;
    state.gyro_z = f[8].trim().parse().ok()?;
    Some(())
}

/// `$VNGPS,time,week,fix,numSats,lat,lon,alt,vN,vE,vD,northU,eastU,downU,speedU,timeU`
fn parse_vngps(f: &[&str], state: &mut VectorNavState) -> Option<()> {
    if f.len() < 15 {
        return None;
    }
    let gps_tow = f[0].trim().parse::<f64>().ok()?;
    let gps_week = f[1].trim().parse::<u16>().ok()?;
    let fix = f[2].trim().parse::<u8>().ok()?;
    let n_sats = f[3].trim().parse::<u8>().ok()?;
    let lat = f[4].trim().parse::<f64>().ok()?;
    let lon = f[5].trim().parse::<f64>().ok()?;
    let alt = f[6].trim().parse::<f64>().ok()?;

    state.gps_tow = gps_tow;
    state.gps_week = gps_week;
    state.gnss_fix_type = fix;
    state.gnss_num_sats = n_sats;
    state.latitude = lat;
    state.longitude = lon;
    state.altitude = alt;
    Some(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checksum_matches_manual_example() {
        // From VN-300 manual §3.5.1 example: "$VNRRG,8*4B"
        assert_eq!(checksum8(b"VNRRG,8"), 0x4B);
    }

    #[test]
    fn parse_vnins_message() {
        let mut s = VectorNavState::default();
        let line =
            "$VNINS,333811.902862,1694,0204,+009.500,-004.754,-000.225,+32.95602815,\
             -096.71424297,+00171.195,-000.840,-000.396,-000.109,07.8,01.6,0.23*XX";
        match parse_line(line, &mut s) {
            ParsedMessage::Async("VNINS") => {}
            other => panic!("unexpected: {other:?}"),
        }
        assert_eq!(s.gps_week, 1694);
        assert_eq!(s.ins_status_raw, 0x0204);
        assert_eq!(s.ins_mode, 0); // bits 0-1 = 0
        assert!(s.gnss_fix); // bit 2
        assert!(s.gnss_compass_active); // bit 9
        assert!(!s.gnss_heading_aiding); // bit 8 not set
        assert!((s.yaw - 9.5).abs() < 1e-3);
        assert!((s.altitude - 171.195).abs() < 1e-3);
    }

    #[test]
    fn malformed_line_reports_invalid() {
        let mut s = VectorNavState::default();
        match parse_line("$VNINS,not-a-number*XX", &mut s) {
            ParsedMessage::Invalid(_) => {}
            other => panic!("expected Invalid, got {other:?}"),
        }
    }
}
