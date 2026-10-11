//! The existing LVXR wire contract: little-endian, sensor-frame metres, IMU g.

pub enum Payload {
    Points(Vec<[f32; 3]>),
    Imu { gyro: [f32; 3], accel: [f32; 3] },
}

pub struct Packet {
    pub lidar_id: u8,
    pub timestamp_ns: u64,
    pub payload: Payload,
}

pub fn decode(bytes: &[u8]) -> Option<Packet> {
    if bytes.len() < 20 || &bytes[..4] != b"LVXR" || bytes[4] != 1 {
        return None;
    }
    let count = u16::from_le_bytes(bytes[8..10].try_into().ok()?) as usize;
    let timestamp_ns = u64::from_le_bytes(bytes[12..20].try_into().ok()?);
    let payload_len = match (bytes[5], bytes[7]) {
        (1, 1) => count.checked_mul(12)?,
        (2, 0) if count == 1 => 24,
        _ => return None,
    };
    if bytes.len() != 20 + payload_len {
        return None;
    }
    let values: Vec<f32> = bytes[20..]
        .chunks_exact(4)
        .map(|b| f32::from_le_bytes(b.try_into().unwrap()))
        .collect();
    if values.iter().any(|v| !v.is_finite()) {
        return None;
    }
    let payload = if bytes[5] == 1 {
        Payload::Points(values.chunks_exact(3).map(|v| [v[0], v[1], v[2]]).collect())
    } else {
        Payload::Imu {
            gyro: [values[0], values[1], values[2]],
            accel: [values[3], values[4], values[5]],
        }
    };
    Some(Packet {
        lidar_id: bytes[6],
        timestamp_ns,
        payload,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn packet(kind: u8, dtype: u8, count: u16, values: &[f32]) -> Vec<u8> {
        let mut b = b"LVXR".to_vec();
        b.extend([1, kind, 40, dtype]);
        b.extend(count.to_le_bytes());
        b.extend([0, 0]);
        b.extend(123_u64.to_le_bytes());
        for v in values {
            b.extend(v.to_le_bytes());
        }
        b
    }

    #[test]
    fn decodes_existing_points_and_imu_contract() {
        let p = decode(&packet(1, 1, 1, &[1.0, 2.0, 3.0])).unwrap();
        assert_eq!((p.lidar_id, p.timestamp_ns), (40, 123));
        assert!(matches!(p.payload, Payload::Points(v) if v == vec![[1.0, 2.0, 3.0]]));
        let p = decode(&packet(2, 0, 1, &[1.0, 2.0, 3.0, 0.0, 0.0, 1.0])).unwrap();
        assert!(matches!(p.payload, Payload::Imu { gyro, accel }
            if gyro == [1.0, 2.0, 3.0] && accel == [0.0, 0.0, 1.0]));
    }

    #[test]
    fn rejects_bad_lengths_types_versions_and_nonfinite_values() {
        let good = packet(1, 1, 1, &[1.0, 2.0, 3.0]);
        for n in 0..good.len() {
            assert!(decode(&good[..n]).is_none());
        }
        for (offset, value) in [(0, 0), (4, 2), (5, 3), (7, 0), (8, 2)] {
            let mut b = good.clone();
            b[offset] = value;
            assert!(decode(&b).is_none());
        }
        assert!(decode(&packet(1, 1, 1, &[f32::NAN, 0.0, 0.0])).is_none());
        assert!(decode(&packet(2, 0, 2, &[0.0; 6])).is_none());
    }
}
