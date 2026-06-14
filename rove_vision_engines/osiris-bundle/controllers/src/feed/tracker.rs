//! Native ByteTrack multi-object tracker.
//!
//! Runs in the orchestrator, on top of any engine's per-frame detections, to
//! assign stable `track_id`s across frames. One tracker instance lives per
//! (engine, feed) bridge, so frames arrive in temporal order.
//!
//! This is a faithful-but-lightweight ByteTrack: two-stage IoU association
//! (high-score then low-score detections), a constant-velocity center predictor
//! (no external Kalman dependency), and a lost-track buffer for re-acquisition.
//! Association is class-aware so e.g. a `backpack` can't steal a `person` track.

use crate::models::detection::Detection;
use crate::models::manifest::TrackingSection;

/// Tunable tracker parameters (resolved from a manifest `[tracking]` section).
#[derive(Debug, Clone)]
pub struct TrackerConfig {
    /// Detections at/above this score join the high-confidence first pass.
    pub high_thresh: f64,
    /// Detections below this score are ignored entirely.
    pub low_thresh: f64,
    /// Min IoU to associate a track with a high-score detection (first pass).
    pub iou_high: f64,
    /// Min IoU to associate a track with a low-score detection (second pass).
    pub iou_low: f64,
    /// Min score for an unmatched detection to start a brand-new track.
    pub new_track_thresh: f64,
    /// Frames a lost track is retained for re-acquisition before deletion.
    pub track_buffer: u32,
    /// Detections with box area below this are not tracked.
    pub min_box_area: f64,
    /// If set, only these classes are tracked; others pass through untracked.
    pub classes: Option<Vec<String>>,
}

impl Default for TrackerConfig {
    fn default() -> Self {
        Self {
            high_thresh: 0.5,
            low_thresh: 0.1,
            iou_high: 0.2,
            iou_low: 0.5,
            new_track_thresh: 0.6,
            track_buffer: 30,
            min_box_area: 0.0,
            classes: None,
        }
    }
}

impl TrackerConfig {
    /// Build from a manifest `[tracking]` section, applying section overrides.
    pub fn from_section(s: &TrackingSection) -> Self {
        Self {
            high_thresh: s.high_thresh,
            low_thresh: s.low_thresh,
            iou_high: s.iou_high,
            iou_low: s.iou_low,
            new_track_thresh: s.new_track_thresh,
            track_buffer: s.track_buffer,
            min_box_area: s.min_box_area,
            classes: s.classes.clone(),
        }
    }
}

#[derive(PartialEq, Clone, Copy)]
enum TrackState {
    Tracked,
    Lost,
}

struct Track {
    id: u64,
    // center + size state
    cx: f64,
    cy: f64,
    w: f64,
    h: f64,
    // center velocity (constant-velocity predictor)
    vx: f64,
    vy: f64,
    class: String,
    state: TrackState,
    time_since_update: u32,
}

impl Track {
    fn new(id: u64, x: f64, y: f64, w: f64, h: f64, class: String) -> Self {
        Self {
            id,
            cx: x + w / 2.0,
            cy: y + h / 2.0,
            w,
            h,
            vx: 0.0,
            vy: 0.0,
            class,
            state: TrackState::Tracked,
            time_since_update: 0,
        }
    }

    /// Current (predicted) box as top-left x, y, width, height.
    fn tlwh(&self) -> (f64, f64, f64, f64) {
        (self.cx - self.w / 2.0, self.cy - self.h / 2.0, self.w, self.h)
    }

    /// Advance the center by its velocity (called once per frame).
    fn predict(&mut self) {
        self.cx += self.vx;
        self.cy += self.vy;
    }

    /// Correct the track from a matched detection box (tlwh).
    fn update_with(&mut self, b: (f64, f64, f64, f64), class: &str) {
        let ncx = b.0 + b.2 / 2.0;
        let ncy = b.1 + b.3 / 2.0;
        // Exponentially-smoothed velocity for stability.
        self.vx = 0.7 * self.vx + 0.3 * (ncx - self.cx);
        self.vy = 0.7 * self.vy + 0.3 * (ncy - self.cy);
        self.cx = ncx;
        self.cy = ncy;
        self.w = b.2;
        self.h = b.3;
        self.class = class.to_string();
        self.state = TrackState::Tracked;
        self.time_since_update = 0;
    }
}

struct DetItem {
    idx: usize,
    tlwh: (f64, f64, f64, f64),
    score: f64,
    class: String,
}

pub struct ByteTracker {
    cfg: TrackerConfig,
    tracks: Vec<Track>,
    next_id: u64,
}

impl ByteTracker {
    pub fn new(cfg: TrackerConfig) -> Self {
        Self {
            cfg,
            tracks: Vec::new(),
            next_id: 1,
        }
    }

    /// Assign `track_id` to each detection in place for this frame.
    pub fn update(&mut self, dets: &mut [Detection]) {
        let cfg = self.cfg.clone();

        // Advance all existing tracks by their velocity.
        for t in &mut self.tracks {
            t.predict();
        }

        // Filter detections that are eligible for tracking.
        let mut items: Vec<DetItem> = Vec::new();
        for (i, d) in dets.iter().enumerate() {
            let (x, y, w, h) = (d.bbox.x, d.bbox.y, d.bbox.width, d.bbox.height);
            if w * h < cfg.min_box_area {
                continue;
            }
            if let Some(ref allowed) = cfg.classes {
                if !allowed.iter().any(|c| c == &d.class) {
                    continue;
                }
            }
            if d.confidence < cfg.low_thresh {
                continue;
            }
            items.push(DetItem {
                idx: i,
                tlwh: (x, y, w, h),
                score: d.confidence,
                class: d.class.clone(),
            });
        }

        let high: Vec<&DetItem> = items.iter().filter(|it| it.score >= cfg.high_thresh).collect();
        let low: Vec<&DetItem> = items.iter().filter(|it| it.score < cfg.high_thresh).collect();

        // det.idx -> assigned track id
        let mut assigned: Vec<Option<u64>> = vec![None; dets.len()];
        let mut track_matched = vec![false; self.tracks.len()];

        // ── First association: all tracks vs high-score detections ──────────
        let track_boxes: Vec<_> = self.tracks.iter().map(|t| t.tlwh()).collect();
        let track_classes: Vec<_> = self.tracks.iter().map(|t| t.class.clone()).collect();
        let high_boxes: Vec<_> = high.iter().map(|d| d.tlwh).collect();
        let high_classes: Vec<_> = high.iter().map(|d| d.class.clone()).collect();

        let mut high_matched = vec![false; high.len()];
        for (ti, di) in greedy_match(&track_boxes, &track_classes, &high_boxes, &high_classes, cfg.iou_high) {
            let d = high[di];
            self.tracks[ti].update_with(d.tlwh, &d.class);
            track_matched[ti] = true;
            high_matched[di] = true;
            assigned[d.idx] = Some(self.tracks[ti].id);
        }

        // ── Second association: still-tracked unmatched tracks vs low dets ──
        let remaining: Vec<usize> = (0..self.tracks.len())
            .filter(|&i| !track_matched[i] && self.tracks[i].state == TrackState::Tracked)
            .collect();
        let rt_boxes: Vec<_> = remaining.iter().map(|&i| self.tracks[i].tlwh()).collect();
        let rt_classes: Vec<_> = remaining.iter().map(|&i| self.tracks[i].class.clone()).collect();
        let low_boxes: Vec<_> = low.iter().map(|d| d.tlwh).collect();
        let low_classes: Vec<_> = low.iter().map(|d| d.class.clone()).collect();

        for (k, di) in greedy_match(&rt_boxes, &rt_classes, &low_boxes, &low_classes, cfg.iou_low) {
            let ti = remaining[k];
            let d = low[di];
            self.tracks[ti].update_with(d.tlwh, &d.class);
            track_matched[ti] = true;
            assigned[d.idx] = Some(self.tracks[ti].id);
        }

        // ── Age out unmatched tracks ───────────────────────────────────────
        for (i, t) in self.tracks.iter_mut().enumerate() {
            if !track_matched[i] {
                t.state = TrackState::Lost;
                t.time_since_update += 1;
            }
        }
        let buffer = cfg.track_buffer;
        self.tracks.retain(|t| t.time_since_update <= buffer);

        // ── Spawn new tracks from confident unmatched high detections ───────
        for (di, d) in high.iter().enumerate() {
            if high_matched[di] || d.score < cfg.new_track_thresh {
                continue;
            }
            let id = self.next_id;
            self.next_id += 1;
            let (x, y, w, h) = d.tlwh;
            self.tracks.push(Track::new(id, x, y, w, h, d.class.clone()));
            assigned[d.idx] = Some(id);
        }

        // Write track ids back onto the detections.
        for (i, d) in dets.iter_mut().enumerate() {
            d.track_id = assigned[i];
        }
    }
}

/// Greedy IoU matching, class-aware. Returns (track_index, det_index) pairs.
fn greedy_match(
    track_boxes: &[(f64, f64, f64, f64)],
    track_classes: &[String],
    det_boxes: &[(f64, f64, f64, f64)],
    det_classes: &[String],
    iou_thresh: f64,
) -> Vec<(usize, usize)> {
    let mut pairs: Vec<(f64, usize, usize)> = Vec::new();
    for (ti, tb) in track_boxes.iter().enumerate() {
        for (di, db) in det_boxes.iter().enumerate() {
            if track_classes[ti] != det_classes[di] {
                continue;
            }
            let v = iou(*tb, *db);
            if v >= iou_thresh {
                pairs.push((v, ti, di));
            }
        }
    }
    // Highest IoU first.
    pairs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

    let mut used_t = vec![false; track_boxes.len()];
    let mut used_d = vec![false; det_boxes.len()];
    let mut matches = Vec::new();
    for (_, ti, di) in pairs {
        if !used_t[ti] && !used_d[di] {
            used_t[ti] = true;
            used_d[di] = true;
            matches.push((ti, di));
        }
    }
    matches
}

/// IoU of two top-left+size boxes.
fn iou(a: (f64, f64, f64, f64), b: (f64, f64, f64, f64)) -> f64 {
    let (ax1, ay1, ax2, ay2) = (a.0, a.1, a.0 + a.2, a.1 + a.3);
    let (bx1, by1, bx2, by2) = (b.0, b.1, b.0 + b.2, b.1 + b.3);
    let ix1 = ax1.max(bx1);
    let iy1 = ay1.max(by1);
    let ix2 = ax2.min(bx2);
    let iy2 = ay2.min(by2);
    let iw = (ix2 - ix1).max(0.0);
    let ih = (iy2 - iy1).max(0.0);
    let inter = iw * ih;
    let union = a.2 * a.3 + b.2 * b.3 - inter;
    if union > 0.0 {
        inter / union
    } else {
        0.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::models::detection::{BBox, Detection};

    fn det(class: &str, x: f64, y: f64, conf: f64) -> Detection {
        Detection {
            class: class.to_string(),
            confidence: conf,
            bbox: BBox { x, y, width: 50.0, height: 100.0 },
            track_id: None,
            keypoints: None,
        }
    }

    #[test]
    fn keeps_stable_id_for_moving_person() {
        let mut tr = ByteTracker::new(TrackerConfig::default());
        let mut f1 = vec![det("person", 100.0, 100.0, 0.9)];
        tr.update(&mut f1);
        let id = f1[0].track_id.expect("track assigned");

        // Move the box a little each frame; id must persist.
        for step in 1..6 {
            let mut f = vec![det("person", 100.0 + step as f64 * 8.0, 100.0, 0.9)];
            tr.update(&mut f);
            assert_eq!(f[0].track_id, Some(id), "id changed at step {step}");
        }
    }

    #[test]
    fn distinct_people_get_distinct_ids() {
        let mut tr = ByteTracker::new(TrackerConfig::default());
        let mut f = vec![det("person", 0.0, 0.0, 0.9), det("person", 400.0, 50.0, 0.9)];
        tr.update(&mut f);
        let a = f[0].track_id.unwrap();
        let b = f[1].track_id.unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn classes_filter_excludes_others() {
        let cfg = TrackerConfig { classes: Some(vec!["person".into()]), ..Default::default() };
        let mut tr = ByteTracker::new(cfg);
        let mut f = vec![det("person", 0.0, 0.0, 0.9), det("car", 200.0, 0.0, 0.9)];
        tr.update(&mut f);
        assert!(f[0].track_id.is_some());
        assert!(f[1].track_id.is_none(), "car should not be tracked");
    }
}
