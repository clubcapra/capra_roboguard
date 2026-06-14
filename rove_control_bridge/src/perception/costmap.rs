//! 3D traversability cost map + local planner (the go-around brain).
//!
//! Bins the lidar cloud into a world-frame 2.5-D grid, classifies each cell's
//! TRAVERSAL COST (flat=cheap, hill/stairs=costly-but-passable, wall/cliff/unknown
//! = no-go), then Dijkstra-routes from the robot toward the goal **staying on
//! known-traversable ground** — so it goes AROUND obstacles and never steers into
//! the unknown/off-road.
//!
//! [`PersistentMap`] ACCUMULATES frames into a fixed world grid (registered by the
//! pose/odometry — exact here on clean pose, lidar-odometry/SLAM later), so the
//! robot remembers terrain it has already seen: the road behind it, a trunk it
//! passed, etc. — the first step toward a real map.

use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::time::Instant;

const CELL: f64 = 0.4;

// per-frame (robot-centric) window
const HALF: f64 = 14.0;
const N_FRAME: usize = 70; // 2*HALF/CELL

// persistent (fixed world) map
const PM_HALF: f64 = 25.0;
const PM_N: usize = 125; // 2*PM_HALF/CELL

// climb envelope (tune to platform) — matches costmap_snapshot.py
const WALL_H: f32 = 0.6; // taller than this above local ground => wall (no-go)
const ASCENT_DEG: f64 = 38.0; // can CLIMB up to this (costly); steeper => no-go
const DESCENT_DEG: f64 = 26.0; // won't DROP into anything steeper than this (no-go)
const CLIFF_DROP: f32 = 0.6; // neighbour ground this much lower => edge/hole (no-go)
const STEP_CLIMB: f32 = 0.45; // climbable step height (stairs)
const NO_GO: f32 = f32::INFINITY;

// no-go classes for kind-aware inflation + the drive-ahead guard
const TRAVERSABLE: u8 = 0;
const OBSTACLE: u8 = 1; // wall/tree — a collision (tighter margin), drive may route around
const UNKNOWN: u8 = 2; // no data — no-go for planning, but NOT a drive-stop (occlusion is everywhere)
const DROP: u8 = 3; // cliff / steep descent — a FALL: full margin AND stops a forward drive

const NEIGHBORS8: [(isize, isize, f32); 8] = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414),
];

/// A classified cost grid ready for planning (per-frame or a persistent snapshot).
pub struct CostMap {
    origin: [f64; 2], // world XY of cell (0,0) corner
    cell: f64,
    n: usize,
    cost: Vec<f32>, // n*n; NO_GO = blocked/unknown, else traversal cost >= 1
    kind: Vec<u8>,  // n*n; TRAVERSABLE / OBSTACLE / HAZARD (for the drive-ahead guard)
    robot: [usize; 2],
    pub stamp: Instant,
}

/// Classify a binned grid (gmin/gmax/count) into a [`CostMap`].
fn classify(
    gmin: &[f32], gmax: &[f32], cnt: &[u32],
    origin: [f64; 2], cell: f64, n: usize, sensor: [f64; 3],
) -> CostMap {
    let mut cost = vec![NO_GO; n * n];
    let mut kind = vec![TRAVERSABLE; n * n]; // per-cell no-go class for inflation
    for i in 0..n {
        for j in 0..n {
            let idx = i * n + j;
            if cnt[idx] == 0 {
                kind[idx] = UNKNOWN; // no data / occluded
                continue;
            }
            let gr = gmin[idx];
            let obstacle_h = gmax[idx] - gr;
            // ascent = a neighbour HIGHER (climb); descent = a neighbour LOWER (drop)
            let (mut max_rise, mut max_drop) = (0f32, 0f32);
            for (di, dj) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                let a = i as isize + di;
                let b = j as isize + dj;
                if a < 0 || a >= n as isize || b < 0 || b >= n as isize {
                    continue;
                }
                let nidx = a as usize * n + b as usize;
                if cnt[nidx] == 0 {
                    continue;
                }
                let dz = gr - gmin[nidx]; // + = neighbour lower (descent)
                max_drop = max_drop.max(dz);
                max_rise = max_rise.max(-dz);
            }
            let ascent_deg = (max_rise.max(0.0) as f64 / cell).atan().to_degrees();
            let descent_deg = (max_drop.max(0.0) as f64 / cell).atan().to_degrees();
            if obstacle_h > WALL_H || ascent_deg > ASCENT_DEG {
                kind[idx] = OBSTACLE; // wall/tree / too steep to climb
                continue;
            }
            if max_drop > CLIFF_DROP || descent_deg > DESCENT_DEG {
                kind[idx] = DROP; // deep hole OR steep DESCENT -> a fall
                continue;
            }
            // traversable: base + slope penalty (descending costs a bit more)
            let mut c = 1.0f32;
            c += (ascent_deg.max(descent_deg) / ASCENT_DEG) as f32 * 3.0;
            c += (descent_deg / DESCENT_DEG) as f32 * 1.0;
            if obstacle_h > STEP_CLIMB * 0.4 {
                c += 2.0; // step/stairs penalty (still passable)
            }
            cost[idx] = c;
        }
    }
    // SAFETY INFLATION (kind-aware): keep the robot's centre clear of no-go, but
    // give HAZARDS (cliffs / holes / unknown — fall risk) a full body-radius margin
    // while OBSTACLES (trees/walls — a collision, not a fall) get a tighter margin,
    // so it can still thread between trunks. Drop-safety preserved, maneuverable.
    const HAZ_INFLATE: isize = 2; // ~0.8 m around drops/holes/unknown
    const OBST_INFLATE: isize = 1; // ~0.4 m around trees/walls
    let src_kind = kind.clone();
    for i in 0..n {
        for j in 0..n {
            if cost[i * n + j].is_infinite() {
                continue;
            }
            let mut block = false;
            'scan: for di in -HAZ_INFLATE..=HAZ_INFLATE {
                for dj in -HAZ_INFLATE..=HAZ_INFLATE {
                    let a = i as isize + di;
                    let b = j as isize + dj;
                    if a < 0 || a >= n as isize || b < 0 || b >= n as isize {
                        continue;
                    }
                    match src_kind[a as usize * n + b as usize] {
                        UNKNOWN | DROP => { block = true; break 'scan; }
                        OBSTACLE if di.abs() <= OBST_INFLATE && dj.abs() <= OBST_INFLATE => {
                            block = true;
                            break 'scan;
                        }
                        _ => {}
                    }
                }
            }
            if block {
                cost[i * n + j] = NO_GO;
            }
        }
    }

    let ri = (((sensor[0] - origin[0]) / cell) as usize).min(n - 1);
    let rj = (((sensor[1] - origin[1]) / cell) as usize).min(n - 1);
    // The robot stands on ground; force a footprint (covers the lidar near blind
    // spot) traversable AFTER inflation so it never boxes itself in.
    for di in -2isize..=2 {
        for dj in -2isize..=2 {
            let a = ri as isize + di;
            let b = rj as isize + dj;
            if a >= 0 && a < n as isize && b >= 0 && b < n as isize {
                let k = a as usize * n + b as usize;
                if cost[k].is_infinite() {
                    cost[k] = 1.0;
                }
            }
        }
    }
    CostMap { origin, cell, n, cost, kind, robot: [ri, rj], stamp: Instant::now() }
}

impl CostMap {
    fn cell_center(&self, i: usize, j: usize) -> [f64; 2] {
        [
            self.origin[0] + (i as f64 + 0.5) * self.cell,
            self.origin[1] + (j as f64 + 0.5) * self.cell,
        ]
    }

    /// Per-frame robot-centric cost map from one world-frame cloud + sensor pose.
    pub fn build(points: &[[f32; 3]], sensor: [f64; 3]) -> CostMap {
        let n = N_FRAME;
        let origin = [sensor[0] - HALF, sensor[1] - HALF];
        let (gmin, gmax, cnt) = bin(points, origin, CELL, n);
        classify(&gmin, &gmax, &cnt, origin, CELL, n, sensor)
    }

    /// Is there a no-go cell (cliff / steep descent / unknown) within `dist` m
    /// ahead along `heading`? The hard guard against driving off an edge: samples
    /// the FULL cost-map classification in the drive direction, so a steep slope or
    /// hole that the narrow lidar corridor misses still stops the robot. Off-map
    /// counts as blocked. Skips the first 0.8 m (the forced footprint / blind spot).
    pub fn blocked_ahead(&self, from: [f64; 2], heading: f64, dist: f64) -> bool {
        let (hx, hy) = (heading.cos(), heading.sin());
        let mut d = 0.8;
        while d <= dist {
            let x = from[0] + d * hx;
            let y = from[1] + d * hy;
            let i = ((x - self.origin[0]) / self.cell) as isize;
            let j = ((y - self.origin[1]) / self.cell) as isize;
            if i >= 0 && i < self.n as isize && j >= 0 && j < self.n as isize
                && self.kind[i as usize * self.n + j as usize] == DROP
            {
                return true; // cliff / steep descent / unknown ahead -> don't drive in
            }
            d += self.cell;
        }
        false
    }

    /// Plan toward `goal` (world XY): Dijkstra over traversable cells from the
    /// robot; pick the reachable cell nearest the goal (frontier), return a point
    /// ~`lookahead` m along that path. `None` if boxed in.
    pub fn plan(&self, goal: [f64; 2], lookahead: f64) -> Option<[f64; 2]> {
        let n = self.n;
        let start = self.robot[0] * n + self.robot[1];
        if self.cost[start].is_infinite() {
            return None;
        }
        let mut dist = vec![f32::INFINITY; n * n];
        let mut prev = vec![usize::MAX; n * n];
        let mut heap: BinaryHeap<Reverse<(i64, usize)>> = BinaryHeap::new();
        dist[start] = 0.0;
        heap.push(Reverse((0, start)));
        while let Some(Reverse((d, u))) = heap.pop() {
            if (d as f32) / 1000.0 > dist[u] {
                continue;
            }
            let (ui, uj) = (u / n, u % n);
            for (di, dj, w) in NEIGHBORS8 {
                let a = ui as isize + di;
                let b = uj as isize + dj;
                if a < 0 || a >= n as isize || b < 0 || b >= n as isize {
                    continue;
                }
                let v = a as usize * n + b as usize;
                let cv = self.cost[v];
                if cv.is_infinite() {
                    continue;
                }
                let nd = dist[u] + cv * w;
                if nd < dist[v] {
                    dist[v] = nd;
                    prev[v] = u;
                    heap.push(Reverse(((nd * 1000.0) as i64, v)));
                }
            }
        }
        let mut best = start;
        let mut bestd = f32::INFINITY;
        for c in 0..n * n {
            if dist[c].is_infinite() {
                continue;
            }
            let p = self.cell_center(c / n, c % n);
            let dd = ((p[0] - goal[0]).powi(2) + (p[1] - goal[1]).powi(2)) as f32;
            if dd < bestd {
                bestd = dd;
                best = c;
            }
        }
        let mut path = Vec::new();
        let mut c = best;
        while c != usize::MAX {
            path.push(c);
            if c == start {
                break;
            }
            c = prev[c];
        }
        path.reverse();
        let rob = self.cell_center(self.robot[0], self.robot[1]);
        let mut target = rob;
        for &c in &path {
            target = self.cell_center(c / n, c % n);
            if ((target[0] - rob[0]).powi(2) + (target[1] - rob[1]).powi(2)).sqrt() >= lookahead {
                break;
            }
        }
        Some(target)
    }
}

/// Bin a world-frame cloud into per-cell (ground=min z, top=max z, count).
fn bin(points: &[[f32; 3]], origin: [f64; 2], cell: f64, n: usize) -> (Vec<f32>, Vec<f32>, Vec<u32>) {
    let mut gmin = vec![f32::INFINITY; n * n];
    let mut gmax = vec![f32::NEG_INFINITY; n * n];
    let mut cnt = vec![0u32; n * n];
    for p in points {
        let gi = ((p[0] as f64 - origin[0]) / cell) as isize;
        let gj = ((p[1] as f64 - origin[1]) / cell) as isize;
        if gi < 0 || gi >= n as isize || gj < 0 || gj >= n as isize {
            continue;
        }
        let idx = gi as usize * n + gj as usize;
        gmin[idx] = gmin[idx].min(p[2]);
        gmax[idx] = gmax[idx].max(p[2]);
        cnt[idx] += 1;
    }
    (gmin, gmax, cnt)
}

/// Persistent world-frame map: accumulates ground/top/count across frames into a
/// FIXED grid (origin pinned on the first frame). Registration is by the cloud's
/// world pose (exact on clean sim pose; lidar-odometry/SLAM on the real robot).
pub struct PersistentMap {
    origin: [f64; 2],
    cell: f64,
    n: usize,
    gmin: Vec<f32>,
    gmax: Vec<f32>,
    cnt: Vec<u32>,
    init: bool,
}

impl PersistentMap {
    pub fn new() -> Self {
        let n = PM_N;
        Self {
            origin: [0.0, 0.0],
            cell: CELL,
            n,
            gmin: vec![f32::INFINITY; n * n],
            gmax: vec![f32::NEG_INFINITY; n * n],
            cnt: vec![0u32; n * n],
            init: false,
        }
    }

    /// Fold one world-frame cloud into the persistent grid.
    pub fn update(&mut self, points: &[[f32; 3]], sensor: [f64; 3]) {
        if !self.init {
            // pin the grid so the start area sits roughly centred
            self.origin = [sensor[0] - PM_HALF, sensor[1] - PM_HALF];
            self.init = true;
        }
        let (o, cell, n) = (self.origin, self.cell, self.n);
        for p in points {
            let gi = ((p[0] as f64 - o[0]) / cell) as isize;
            let gj = ((p[1] as f64 - o[1]) / cell) as isize;
            if gi < 0 || gi >= n as isize || gj < 0 || gj >= n as isize {
                continue;
            }
            let idx = gi as usize * n + gj as usize;
            self.gmin[idx] = self.gmin[idx].min(p[2]);
            self.gmax[idx] = self.gmax[idx].max(p[2]);
            self.cnt[idx] = self.cnt[idx].saturating_add(1);
        }
    }

    /// Classify the accumulated map into a planner-ready [`CostMap`].
    pub fn to_costmap(&self, sensor: [f64; 3]) -> CostMap {
        classify(&self.gmin, &self.gmax, &self.cnt, self.origin, self.cell, self.n, sensor)
    }

    /// Known (observed) cell count — how much of the world has been mapped.
    pub fn known_cells(&self) -> usize {
        self.cnt.iter().filter(|&&c| c > 0).count()
    }
}

impl Default for PersistentMap {
    fn default() -> Self {
        Self::new()
    }
}
