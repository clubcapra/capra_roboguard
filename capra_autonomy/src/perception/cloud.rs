//! Livox point-cloud wire decode + reassembly (mirrors the sim's `LVX2` format
//! in `rove_sim/rove_sim/sensors/lidar.py`).
//!
//! Each scan is fragmented across datagrams:
//! ```text
//! header (52 B, little-endian):
//!   magic   : 4 bytes = b"LVX2"
//!   frame_id: u32
//!   pkt_idx : u16
//!   n_pkts  : u16
//!   total   : u32
//!   t       : f64
//!   pose    : 7 x f32  (pos xyz + quat xyzw, WORLD frame)
//! payload: f32 xyz per point (points are WORLD coordinates of the hits)
//! ```

pub const MAGIC: &[u8; 4] = b"LVX2";
pub const HDR_LEN: usize = 52;

#[derive(Debug, Clone, Copy)]
pub struct Pose {
    pub pos: [f64; 3],
    pub quat: [f64; 4], // xyzw
}

/// One decoded datagram.
pub struct Packet {
    pub frame_id: u32,
    pub pkt_idx: u16,
    pub n_pkts: u16,
    pub pose: Pose,
    pub points: Vec<[f32; 3]>,
}

fn le_u16(b: &[u8], o: usize) -> u16 {
    u16::from_le_bytes([b[o], b[o + 1]])
}
fn le_u32(b: &[u8], o: usize) -> u32 {
    u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}
fn le_f32(b: &[u8], o: usize) -> f32 {
    f32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]])
}
fn le_f64(b: &[u8], o: usize) -> f64 {
    let mut a = [0u8; 8];
    a.copy_from_slice(&b[o..o + 8]);
    f64::from_le_bytes(a)
}

/// Decode one datagram. Returns `None` on a short/garbled frame.
pub fn decode(buf: &[u8]) -> Option<Packet> {
    if buf.len() < HDR_LEN || &buf[0..4] != MAGIC {
        return None;
    }
    let frame_id = le_u32(buf, 4);
    let pkt_idx = le_u16(buf, 8);
    let n_pkts = le_u16(buf, 10);
    // total u32 @12, t f64 @16 — not needed downstream
    let pose = Pose {
        pos: [le_f32(buf, 24) as f64, le_f32(buf, 28) as f64, le_f32(buf, 32) as f64],
        quat: [
            le_f32(buf, 36) as f64,
            le_f32(buf, 40) as f64,
            le_f32(buf, 44) as f64,
            le_f32(buf, 48) as f64,
        ],
    };
    let body = &buf[HDR_LEN..];
    let n = body.len() / 12;
    let mut points = Vec::with_capacity(n);
    for i in 0..n {
        let o = i * 12;
        points.push([le_f32(body, o), le_f32(body, o + 4), le_f32(body, o + 8)]);
    }
    Some(Packet { frame_id, pkt_idx, n_pkts, pose, points })
}

/// Rebuilds a full multi-packet scan from its datagrams (loopback/LAN UDP doesn't
/// reorder, so a frame completes when its packet count is reached; a new frame id
/// drops any partial frame). Returns `(points, pose)` once complete.
#[derive(Default)]
pub struct Reassembler {
    frame_id: Option<u32>,
    parts: std::collections::HashMap<u16, Vec<[f32; 3]>>,
    n_pkts: u16,
    pose: Option<Pose>,
}

impl Reassembler {
    pub fn feed(&mut self, pkt: Packet) -> Option<(Vec<[f32; 3]>, Pose)> {
        if Some(pkt.frame_id) != self.frame_id {
            self.frame_id = Some(pkt.frame_id);
            self.parts.clear();
            self.n_pkts = pkt.n_pkts;
            self.pose = Some(pkt.pose);
        }
        self.parts.insert(pkt.pkt_idx, pkt.points);
        if self.parts.len() as u16 >= self.n_pkts {
            let mut all = Vec::new();
            let mut idxs: Vec<u16> = self.parts.keys().copied().collect();
            idxs.sort_unstable();
            for i in idxs {
                all.extend_from_slice(&self.parts[&i]);
            }
            let pose = self.pose.take().unwrap();
            self.frame_id = None;
            self.parts.clear();
            return Some((all, pose));
        }
        None
    }
}
