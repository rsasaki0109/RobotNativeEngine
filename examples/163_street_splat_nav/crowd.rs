//! The street's crowd: walkers, cyclists, dog walkers, strollers and cars.
//!
//! A port of visloc-rs `people.Crowd.step` (`ros2/visloc-ros2/gs/people.py`). Everyone moves
//! along the route (the start and goals joined by straight lines) at an arc position `s`, on
//! foot at a side offset from the walk the video was taken on, by car in a lane of the road
//! beside it. Each step, whoever has someone just ahead steps aside and slows, follows someone
//! going the same way, or stops just short of touching; no step makes two footprints (or one
//! and the robot's) overlap by more than 2 cm. People keep off the road (the pavement's edge)
//! except on the crossing, keep near the walk between kerbs, now and then stand still or drift
//! across, and turn back when they cannot get through for 8 s. Walkers who reach the crossing
//! on red mostly cross, wait, and come back. Cars keep right, brake for the light and for whoever
//! is ahead, and U-turn in a half circle at the road's ends.
//!
//! The start (who is where, how fast, which way) comes from visloc-rs `rne_crowd.py`, which
//! builds `people.Crowd` with the run's seed; the behaviour's own randomness here is seeded
//! ([`SplitMix64`]), so a run repeats exactly. Coordinates are the street bundle's (z up).

use serde::Deserialize;
use std::f64::consts::PI;

/// Kinds of agent.
pub const WALKER: u8 = 0;
/// See [`WALKER`].
pub const CYCLIST: u8 = 1;
/// See [`WALKER`].
pub const DOG_WALKER: u8 = 2;
/// See [`WALKER`].
pub const STROLLER: u8 = 3;
/// See [`WALKER`].
pub const CAR: u8 = 4;

/// A small seeded random generator (SplitMix64), for the crowd's behaviour.
#[derive(Clone, Debug)]
pub struct SplitMix64(u64);

impl SplitMix64 {
    /// A generator from a seed.
    pub fn new(seed: u64) -> Self {
        Self(seed)
    }

    /// Uniform in `[0, 1)`.
    pub fn random(&mut self) -> f64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^= z >> 31;
        (z >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Uniform in `[a, b)`.
    pub fn uniform(&mut self, a: f64, b: f64) -> f64 {
        a + (b - a) * self.random()
    }
}

/// One agent as `crowd.json` gives it.
#[derive(Clone, Debug, Deserialize)]
pub struct AgentSpec {
    /// [`WALKER`] ... [`CAR`].
    pub kind: u8,
    /// Arc position along the route (m).
    pub s: f64,
    /// +1 along the route, -1 back.
    pub dir: f64,
    /// Cruising speed (m/s).
    pub v: f64,
    /// Side offset (m, + left): from the walk on foot, from the route in a car (its lane).
    pub side: f64,
    /// Gait (or wheel) phase, 0 .. 1.
    pub phase: f64,
    /// Footprint circles (along, left, radius) in the agent's frame.
    pub foot: Vec<[f64; 3]>,
    /// The agent's looks at each phase (PLY paths in the bundle).
    pub phases: Vec<String>,
}

/// `crowd.json` (visloc-rs `rne_crowd.py`).
#[derive(Clone, Debug, Deserialize)]
pub struct CrowdSpec {
    /// Gait phases per cycle.
    pub phases: usize,
    /// A person's footprint radius (m).
    pub radius_m: f64,
    /// A lane's centre from the road's centre line (m).
    pub lane_half_m: f64,
    /// How far from the walk people go between kerbs (m).
    pub narrow_m: f64,
    /// How far ahead people on foot look (m).
    pub yield_m: f64,
    /// The route: start and goals (x, y).
    pub route: Vec<[f64; 2]>,
    /// `[s, edge, side]`: where the pavement ends on the road side.
    pub pavement: Option<Vec<[f64; 3]>>,
    /// `[s, offset]`: the walk's offset from the route.
    pub walk_offsets: Option<Vec<[f64; 2]>>,
    /// Where the walk crosses a street between kerbs (s0, s1).
    #[serde(default)]
    pub kerb_spans: Vec<[f64; 2]>,
    /// The crossing's arc position, when there is one.
    pub crossing_s: Option<f64>,
    /// The crossing's looks at each light state (`green`, `amber`, `red`).
    #[serde(default)]
    pub decor: std::collections::BTreeMap<String, String>,
    /// Who starts where.
    pub agents: Vec<AgentSpec>,
    /// The road beside the route, `[s0, s1, offset]` stretches (cars).
    #[serde(default)]
    pub road: Vec<[f64; 3]>,
    /// The cars' stretch of route (s0, s1).
    pub road_s: Option<[f64; 2]>,
    /// The road's straight centre lines `[s0, s1, px, py, ux, uy]`.
    pub road_lines: Option<Vec<[f64; 6]>>,
    /// The two lanes as polylines at the route's vertices: `"1"` and `"-1"`.
    #[serde(default)]
    pub lanes: std::collections::BTreeMap<String, Vec<[f64; 2]>>,
}

/// The traffic light the cars see.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Light {
    /// Cars go (15 s).
    Green,
    /// Cars stop if they can (3 s).
    Amber,
    /// Cars wait, walkers cross (12 s).
    Red,
}

#[derive(Clone, Debug)]
struct Agent {
    kind: u8,
    s: f64,
    dir: f64,
    v: f64,
    side: f64,
    phase: f64,
    foot: Vec<[f64; 3]>,
    cv: f64,
    uturn: f64,
    lane_mix: f64,
    cross: u8,
    cross_home: f64,
    cross_wait: f64,
    cross_tried: bool,
    cross_block: f64,
    jam: f64,
    squeeze: bool,
    pause: f64,
    side_goal: f64,
    xy: [f64; 2],
    yaw: f64,
    normal: [f64; 2],
    dodge: f64,
    waited: f64,
    stuck: f64,
    moving: f64,
}

/// Where someone stands: (x, y), heading, and the route's left normal there.
type Pose = ([f64; 2], f64, [f64; 2]);

/// The crowd, moving.
#[derive(Clone, Debug)]
pub struct Crowd {
    spec: CrowdSpec,
    cum: Vec<f64>,
    length: f64,
    agents: Vec<Agent>,
    clock_s: f64,
    rng: SplitMix64,
    /// The robot's body as people keep clear of it (m).
    pub robot_radius_m: f64,
}

fn interp(x: f64, table: &[[f64; 2]]) -> f64 {
    let n = table.len();
    if x <= table[0][0] {
        return table[0][1];
    }
    if x >= table[n - 1][0] {
        return table[n - 1][1];
    }
    let k = table.partition_point(|r| r[0] <= x).max(1) - 1;
    let (a, b) = (table[k], table[k + 1]);
    a[1] + (b[1] - a[1]) * (x - a[0]) / (b[0] - a[0]).max(1e-12)
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

impl Crowd {
    /// The crowd of `spec`, placed. `seed` seeds its behaviour.
    pub fn new(spec: CrowdSpec, seed: u64, solid: &dyn Fn(f64, f64) -> bool) -> Self {
        let mut cum = vec![0.0];
        for w in spec.route.windows(2) {
            cum.push(cum.last().unwrap() + (w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]));
        }
        let length = *cum.last().unwrap();
        let agents = spec
            .agents
            .iter()
            .map(|a| Agent {
                kind: a.kind,
                s: a.s,
                dir: a.dir,
                v: a.v,
                side: a.side,
                phase: a.phase,
                foot: a.foot.clone(),
                cv: a.v,
                uturn: 0.0,
                lane_mix: a.dir,
                cross: 0,
                cross_home: 0.0,
                cross_wait: 0.0,
                cross_tried: false,
                cross_block: 0.0,
                jam: 0.0,
                squeeze: false,
                pause: 0.0,
                side_goal: a.side,
                xy: [0.0; 2],
                yaw: 0.0,
                normal: [0.0; 2],
                dodge: 0.0,
                waited: 0.0,
                stuck: 0.0,
                moving: 1.0,
            })
            .collect();
        let mut crowd = Self {
            spec,
            cum,
            length,
            agents,
            clock_s: 0.0,
            rng: SplitMix64::new(seed.wrapping_add(2000)),
            robot_radius_m: 0.2,
        };
        crowd.place(solid);
        crowd
    }

    /// The spec it was built from.
    pub fn spec(&self) -> &CrowdSpec {
        &self.spec
    }

    /// Simulated time (s).
    pub fn clock_s(&self) -> f64 {
        self.clock_s
    }

    /// The cars' light: green 15 s, amber 3 s, red 12 s.
    pub fn light(&self) -> Light {
        let t = self.clock_s.rem_euclid(30.0);
        if t < 15.0 {
            Light::Green
        } else if t < 18.0 {
            Light::Amber
        } else {
            Light::Red
        }
    }

    /// Number of agents.
    pub fn len(&self) -> usize {
        self.agents.len()
    }

    /// Whether there is nobody.
    pub fn is_empty(&self) -> bool {
        self.agents.is_empty()
    }

    /// Agent `i`: kind, (x, y), heading (rad), phase index of `phases`.
    pub fn agent(&self, i: usize) -> (u8, [f64; 2], f64, usize) {
        let a = &self.agents[i];
        let n = self.spec.phases.max(1);
        (a.kind, a.xy, a.yaw, ((a.phase * n as f64) as usize) % n)
    }

    /// Whether agent `i` is on (or across) the crossing rather than on its pavement.
    pub fn is_crossing(&self, i: usize) -> bool {
        self.agents[i].cross != 0
    }

    /// Every footprint circle with its agent's velocity: (x, y, radius, vx, vy).
    pub fn moving_circles(&self) -> Vec<(f64, f64, f64, f64, f64)> {
        self.circles()
            .into_iter()
            .map(|(x, y, r, i)| {
                let a = &self.agents[i];
                let v = a.v * a.moving;
                (x, y, r, v * a.yaw.cos(), v * a.yaw.sin())
            })
            .collect()
    }

    /// Every footprint circle: (x, y, radius, agent).
    pub fn circles(&self) -> Vec<(f64, f64, f64, usize)> {
        let mut out = Vec::new();
        for (i, a) in self.agents.iter().enumerate() {
            let (c, r) = self.footprint(i, a.xy, a.yaw);
            out.extend(c.iter().zip(&r).map(|(p, r)| (p[0], p[1], *r, i)));
        }
        out
    }

    fn at(&self, s: f64) -> ([f64; 2], [f64; 2]) {
        let route = &self.spec.route;
        let idx = self.cum.partition_point(|&c| c < s);
        let k = (idx as isize - 1).clamp(0, route.len() as isize - 2) as usize;
        let t = (s - self.cum[k]) / (self.cum[k + 1] - self.cum[k]).max(1e-9);
        let (a, b) = (route[k], route[k + 1]);
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let len = dx.hypot(dy) + 1e-12;
        ([a[0] + t * dx, a[1] + t * dy], [dx / len, dy / len])
    }

    fn segment(&self, s: f64) -> usize {
        let idx = self.cum.partition_point(|&c| c < s);
        (idx as isize - 1).clamp(0, self.spec.route.len() as isize - 2) as usize
    }

    fn road_line(&self, s: f64) -> Option<([f64; 2], [f64; 2])> {
        let lines = self.spec.road_lines.as_ref()?;
        let key = |r: &[f64; 6]| {
            if r[0] <= s && s <= r[1] {
                0.0
            } else {
                (s - r[0]).abs().min((s - r[1]).abs())
            }
        };
        let mut best = lines.first()?;
        for r in lines {
            if key(r) < key(best) {
                best = r;
            }
        }
        Some(([best[2], best[3]], [best[4], best[5]]))
    }

    /// A car's side offset at s going this way: the road's offset there, keeping right.
    fn lane(&self, s: f64, direction: f64) -> f64 {
        let lh = self.spec.lane_half_m;
        if let Some((q, u)) = self.road_line(s) {
            let (p, d) = self.at(s);
            let n = [-d[1], d[0]];
            let den = n[0] * u[1] - n[1] * u[0];
            let den = if den.abs() > 1e-6 { den } else { 1e-6 };
            let off = ((q[0] - p[0]) * u[1] - (q[1] - p[1]) * u[0]) / den;
            return off - lh * direction;
        }
        let road = &self.spec.road;
        if road.is_empty() {
            return 0.0;
        }
        let k = road
            .iter()
            .position(|r| r[0] <= s && s <= r[1])
            .unwrap_or(if s < road[0][0] { 0 } else { road.len() - 1 });
        let mut off = road[k][2];
        for (j, edge) in [(k as isize - 1, road[k][0]), (k as isize + 1, road[k][1])] {
            if j >= 0 && (j as usize) < road.len() && (s - edge).abs() < 4.0 {
                let w = 0.5 * (1.0 - (s - edge).abs() / 4.0);
                off = (1.0 - w) * off + w * road[j as usize][2];
            }
        }
        off - lh * direction
    }

    fn road_side(&self) -> f64 {
        let c = self.spec.crossing_s.unwrap_or(0.0);
        if self.lane(c, 1.0) + self.spec.lane_half_m > 0.0 {
            1.0
        } else {
            -1.0
        }
    }

    fn lane_point(&self, i: usize, k: usize, t: f64) -> ([f64; 2], [f64; 2]) {
        let w = (self.agents[i].lane_mix + 1.0) / 2.0;
        let (lp, lm) = (&self.spec.lanes["1"], &self.spec.lanes["-1"]);
        let mix =
            |a: [f64; 2], b: [f64; 2]| [w * a[0] + (1.0 - w) * b[0], w * a[1] + (1.0 - w) * b[1]];
        let p0 = mix(lp[k], lm[k]);
        let p1 = mix(lp[k + 1], lm[k + 1]);
        let xy = [(1.0 - t) * p0[0] + t * p1[0], (1.0 - t) * p0[1] + t * p1[1]];
        (xy, [p1[0] - p0[0], p1[1] - p0[1]])
    }

    fn car_pose(&self, i: usize, s: f64) -> Pose {
        let a = &self.agents[i];
        let lh = self.spec.lane_half_m;
        let mut s = s;
        if a.uturn > 0.0 {
            s = (s - a.dir * lh * (PI * a.uturn).sin()).clamp(0.0, self.length);
        }
        let k = self.segment(s);
        let t = (s - self.cum[k]) / (self.cum[k + 1] - self.cum[k]).max(1e-9);
        let (xy, seg) = self.lane_point(i, k, t);
        let len = seg[0].hypot(seg[1]) + 1e-12;
        let seg = [seg[0] / len, seg[1] / len];
        if a.uturn > 0.0 {
            let yaw = (-seg[1] * a.dir).atan2(-seg[0] * a.dir) + PI * a.uturn;
            let yaw = yaw.sin().atan2(yaw.cos());
            return (xy, yaw, [-yaw.sin(), yaw.cos()]);
        }
        let yaw = (seg[1] * a.dir).atan2(seg[0] * a.dir);
        (xy, yaw, [-seg[1], seg[0]])
    }

    fn car_on_crossing(&self) -> bool {
        let Some(c) = self.spec.crossing_s else {
            return false;
        };
        self.agents.iter().any(|a| {
            let ahead = a.dir * (c - a.s);
            a.kind == CAR && -4.5 < ahead && ahead < 4.5 + a.cv * a.cv / (2.0 * 3.0) + 1.0
        })
    }

    fn lane_scale(&self, i: usize) -> f64 {
        let k = self.segment(self.agents[i].s);
        let (_, seg) = self.lane_point(i, k, 0.0);
        (self.cum[k + 1] - self.cum[k]) / seg[0].hypot(seg[1]).max(1e-6)
    }

    fn base(&self, s: f64) -> f64 {
        self.spec
            .walk_offsets
            .as_ref()
            .map_or(0.0, |w| interp(s, w))
    }

    fn pose(
        &self,
        i: usize,
        s: f64,
        dodge: f64,
        direction: f64,
        solid: &dyn Fn(f64, f64) -> bool,
    ) -> Pose {
        let a = &self.agents[i];
        if a.kind == CAR {
            return self.car_pose(i, s);
        }
        let (p, d) = self.at(s);
        let normal = [-d[1], d[0]];
        let base = self.base(s);
        let mut off = base + a.side + dodge;
        let narrow = self.spec.narrow_m;
        if a.cross == 0
            && self
                .spec
                .kerb_spans
                .iter()
                .any(|r| r[0] - 1.0 <= s && s <= r[1] + 1.0)
        {
            off = base + (off - base).clamp(-narrow, narrow);
        }
        if let Some(pave) = self.spec.pavement.as_ref().filter(|p| !p.is_empty()) {
            if a.cross == 0 && pave[0][0] - 0.5 <= s && s <= pave[pave.len() - 1][0] + 0.5 {
                let k = pave
                    .iter()
                    .enumerate()
                    .min_by(|x, y| (x.1[0] - s).abs().total_cmp(&(y.1[0] - s).abs()))
                    .map_or(0, |(k, _)| k);
                let table: Vec<[f64; 2]> = pave.iter().map(|r| [r[0], r[1]]).collect();
                let edge = interp(s, &table);
                let r = self.spec.radius_m;
                off = if pave[k][2] > 0.0 {
                    off.min(edge - r)
                } else {
                    off.max(edge + r)
                };
            }
        }
        let r = self.spec.radius_m;
        while a.cross == 0
            && (off - base).abs() > 0.05
            && hits(solid, p[0] + off * normal[0], p[1] + off * normal[1], r)
        {
            off = base + (off - base) * 0.7;
        }
        (
            [p[0] + off * normal[0], p[1] + off * normal[1]],
            (d[1] * direction).atan2(d[0] * direction),
            normal,
        )
    }

    fn footprint(&self, i: usize, xy: [f64; 2], yaw: f64) -> (Vec<[f64; 2]>, Vec<f64>) {
        let h = [yaw.cos(), yaw.sin()];
        let left = [-h[1], h[0]];
        let f = &self.agents[i].foot;
        (
            f.iter()
                .map(|c| {
                    [
                        xy[0] + c[0] * h[0] + c[1] * left[0],
                        xy[1] + c[0] * h[1] + c[1] * left[1],
                    ]
                })
                .collect(),
            f.iter().map(|c| c[2]).collect(),
        )
    }

    fn overlap(&self, i: usize, xy: [f64; 2], yaw: f64, robot: [f64; 2]) -> f64 {
        let (c, r) = self.footprint(i, xy, yaw);
        let mut worst = f64::NEG_INFINITY;
        for (p, ri) in c.iter().zip(&r) {
            worst = worst
                .max(ri + self.robot_radius_m + 0.05 - (p[0] - robot[0]).hypot(p[1] - robot[1]));
        }
        for (j, b) in self.agents.iter().enumerate() {
            if j == i {
                continue;
            }
            let (cj, rj) = self.footprint(j, b.xy, b.yaw);
            for (p, ri) in c.iter().zip(&r) {
                for (q, rq) in cj.iter().zip(&rj) {
                    worst = worst.max(ri + rq - (p[0] - q[0]).hypot(p[1] - q[1]));
                }
            }
        }
        worst
    }

    fn clear(&self, i: usize, xy: [f64; 2], yaw: f64, robot: [f64; 2], slack: f64) -> bool {
        let (c, r) = self.footprint(i, xy, yaw);
        if c.iter().zip(&r).any(|(p, ri)| {
            (p[0] - robot[0]).hypot(p[1] - robot[1]) < ri + self.robot_radius_m + 0.05
        }) {
            return false;
        }
        let me = &self.agents[i];
        for (j, b) in self.agents.iter().enumerate() {
            if j == i {
                continue;
            }
            let (cj, rj) = self.footprint(j, b.xy, b.yaw);
            let sl = if me.kind == CAR || b.kind == CAR || me.dir == b.dir {
                0.02
            } else {
                slack
            };
            for (p, ri) in c.iter().zip(&r) {
                for (q, rq) in cj.iter().zip(&rj) {
                    if (p[0] - q[0]).hypot(p[1] - q[1]) < ri + rq - sl {
                        return false;
                    }
                }
            }
        }
        true
    }

    fn place(&mut self, solid: &dyn Fn(f64, f64) -> bool) {
        for i in 0..self.agents.len() {
            let a = &self.agents[i];
            let (xy, yaw, normal) = if a.kind == CAR {
                self.car_pose(i, a.s)
            } else {
                let (_, d) = self.at(a.s);
                let (xy, yaw, _) = self.pose(i, a.s, a.dodge, a.dir, solid);
                (xy, yaw, [-d[1], d[0]])
            };
            let a = &mut self.agents[i];
            (a.xy, a.yaw, a.normal) = (xy, yaw, normal);
        }
    }

    /// The nearest thing ahead of agent `i`: (gap, lateral, needed clearance, speed to
    /// follow, whether it is the robot).
    fn ahead(
        &self,
        i: usize,
        robot: [f64; 2],
        half: &[f64],
        rad: &[f64],
        vel: &[[f64; 2]],
    ) -> Option<(f64, f64, f64, Option<f64>, bool)> {
        let me = &self.agents[i];
        let heading = [me.yaw.cos(), me.yaw.sin()];
        let look = if me.kind == CAR {
            12.0
        } else {
            self.spec.yield_m * if me.kind == CYCLIST { 2.5 } else { 1.0 }
        };
        let mut best = None;
        let rel = [robot[0] - me.xy[0], robot[1] - me.xy[1]];
        let (a, lat) = (dot(rel, heading) - half[i], dot(rel, me.normal));
        let need = rad[i] + 0.45;
        if 0.0 < a && a < look + 0.45 && lat.abs() < need + 0.25 {
            best = Some((a, lat, need, None, true));
        }
        for (j, b) in self.agents.iter().enumerate() {
            if j == i {
                continue;
            }
            let rel = [b.xy[0] - me.xy[0], b.xy[1] - me.xy[1]];
            let hj = [b.yaw.cos(), b.yaw.sin()];
            let a = dot(rel, heading) - half[i] - half[j] * dot(hj, heading).abs();
            let need = rad[i] + rad[j];
            if !(-rad[j] < a && a < look + rad[j]) {
                continue;
            }
            let lat = dot(rel, me.normal);
            if lat.abs() >= need + 0.25 {
                continue;
            }
            let along = dot(vel[j], heading);
            let follow = (along > 0.2).then_some(along);
            if best.is_none_or(|b: (f64, f64, f64, Option<f64>, bool)| a < b.0) {
                best = Some((a, lat, need, follow, false));
            }
        }
        best
    }

    /// One step of `dt_s`: everyone walks, rides or drives on, keeping clear of the robot at
    /// `robot` (x, y) and of each other. `solid(x, y)` is the street's solid cells (outside the
    /// map counts as solid).
    // A faithful port of people.Crowd.step: one long pass over everyone, by index.
    #[allow(
        clippy::too_many_lines,
        clippy::cognitive_complexity,
        clippy::needless_range_loop
    )]
    pub fn step(&mut self, dt: f64, robot: [f64; 2], solid: &dyn Fn(f64, f64) -> bool) {
        let n = self.agents.len();
        self.clock_s += dt;
        let light = self.spec.crossing_s.map(|_| self.light());
        let lh = self.spec.lane_half_m;
        let rad: Vec<f64> = self
            .agents
            .iter()
            .map(|a| a.foot.iter().map(|f| f[1].abs() + f[2]).fold(0.0, f64::max))
            .collect();
        let half: Vec<f64> = self
            .agents
            .iter()
            .zip(&rad)
            .map(|(a, r)| a.foot.iter().map(|f| f[0].abs() + f[2]).fold(0.0, f64::max) - r)
            .collect();
        let vel: Vec<[f64; 2]> = self
            .agents
            .iter()
            .map(|a| [a.yaw.cos() * a.v * a.moving, a.yaw.sin() * a.v * a.moving])
            .collect();
        let mut moving = vec![1.0; n];
        for i in 0..n {
            let kind = self.agents[i].kind;
            let (bike, car) = (kind == CYCLIST, kind == CAR);
            let best = self.ahead(i, robot, &half, &rad, &vel);
            let mut speed = self.agents[i].v;
            let mut hold = false;
            let mut cap = f64::INFINITY;
            let mut cross_want = false;
            let (old_dodge, old_side) = (self.agents[i].dodge, self.agents[i].side);
            if car && self.agents[i].uturn > 0.0 {
                // in a U-turn: on round the half circle
                let (u_old, mix_old, s_old) = (
                    self.agents[i].uturn,
                    self.agents[i].lane_mix,
                    self.agents[i].s,
                );
                let turn_v = self.agents[i].v.min(2.5);
                let u_new = u_old + turn_v * dt / (PI * lh);
                {
                    let a = &mut self.agents[i];
                    if u_new >= 1.0 {
                        a.uturn = 0.0;
                        a.lane_mix = a.dir;
                        a.s = s_old + a.dir * (u_new - 1.0) * PI * lh;
                    } else {
                        a.uturn = u_new;
                        a.lane_mix = -a.dir * (PI * u_new).cos();
                    }
                }
                let (xy, yaw, normal) = self.car_pose(i, self.agents[i].s);
                if self.clear(i, xy, yaw, robot, 0.02) {
                    let a = &mut self.agents[i];
                    (a.xy, a.yaw, a.normal) = (xy, yaw, normal);
                    a.cv = turn_v;
                    a.phase = (a.phase + turn_v * dt / 2.1).rem_euclid(1.0);
                    moving[i] = turn_v / a.v.max(1e-6);
                } else {
                    let a = &mut self.agents[i];
                    (a.uturn, a.lane_mix, a.s) = (u_old, mix_old, s_old);
                    moving[i] = 0.0;
                }
                continue;
            }
            if self.agents[i].cross != 0 {
                // crossing the road and back
                let side = self.road_side();
                let c = self.spec.crossing_s.unwrap_or(0.0);
                let target = if matches!(self.agents[i].cross, 1 | 2) {
                    self.lane(c, 1.0) + lh + side * (lh + 1.6) - self.base(self.agents[i].s)
                } else {
                    self.agents[i].cross_home
                };
                let step = (target - self.agents[i].side).clamp(-1.2 * dt, 1.2 * dt);
                self.agents[i].side += step;
                cross_want = step.abs() > 1e-6;
                hold = true;
                let car_on = self.car_on_crossing();
                let a = &mut self.agents[i];
                if a.cross == 1 && step == 0.0 {
                    a.cross = 2;
                    a.cross_wait = self.rng.uniform(4.0, 10.0);
                } else if a.cross == 2 {
                    a.cross_wait -= dt;
                    if a.cross_wait <= 0.0 && light == Some(Light::Red) && !car_on {
                        a.cross = 3;
                    }
                } else if a.cross == 3 && step == 0.0 {
                    a.cross = 0;
                    a.side_goal = a.side;
                }
            } else if !bike {
                // drift across now and then
                if self.rng.random() < dt / 90.0 {
                    self.agents[i].side_goal = self.rng.uniform(-1.6, 1.6);
                }
                let a = &mut self.agents[i];
                let step = (a.side_goal - a.side).clamp(-0.5 * dt, 0.5 * dt);
                a.side += step;
                if step != 0.0 {
                    speed *= 0.8;
                }
            }
            if let Some(c) = self.spec.crossing_s {
                if kind == WALKER && self.agents[i].cross == 0 {
                    let near = (self.agents[i].s - c).abs() < 0.4;
                    if near
                        && !self.agents[i].cross_tried
                        && light == Some(Light::Red)
                        && !self.car_on_crossing()
                    {
                        self.agents[i].cross_tried = true;
                        if self.rng.random() < 0.6 {
                            let a = &mut self.agents[i];
                            a.cross = 1;
                            a.cross_home = a.side;
                        }
                    } else if !near {
                        self.agents[i].cross_tried = false;
                    }
                }
                if car && matches!(light, Some(Light::Amber | Light::Red)) {
                    // brake for the line and wait there; on amber, a car too close to stop drives on
                    let a = &self.agents[i];
                    let to_line = a.dir * (c - a.s) - 2.3 - 3.0;
                    if (0.0..30.0).contains(&to_line)
                        && (light == Some(Light::Red)
                            || a.cv * a.cv / (2.0 * to_line.max(1e-3)) < 4.5)
                    {
                        cap = (2.0 * 3.0 * to_line).sqrt();
                        if to_line < 0.3 {
                            hold = true;
                        }
                    }
                }
            }
            let mut pausing = false;
            if kind == WALKER {
                // a walker stands still a while now and then
                if self.agents[i].pause > 0.0 {
                    self.agents[i].pause -= dt;
                    pausing = true;
                } else if self.rng.random() < dt / 60.0 {
                    self.agents[i].pause = self.rng.uniform(8.0, 20.0);
                }
            }
            if let Some((a, lat, need, follow, is_robot)) = best {
                let ag = &mut self.agents[i];
                let mut away = if lat > 0.0 { -1.0 } else { 1.0 };
                if ag.waited > 3.0 {
                    away = -away; // that side is blocked: try the other
                }
                if ag.waited > 6.0 {
                    ag.waited = 0.0;
                }
                let rate = if car {
                    0.0
                } else if bike {
                    1.2
                } else {
                    0.8
                };
                ag.dodge = (ag.dodge + away * rate * dt).clamp(-2.0, 2.0);
                ag.waited += if a < need + 0.3 { dt } else { 0.0 };
                let in_way = lat.abs() < need + 0.05;
                speed = if in_way && follow.is_some() {
                    ag.v.min(follow.unwrap_or(0.0))
                } else if in_way && a < need + 0.3 {
                    0.0
                } else {
                    (if is_robot { 0.35 } else { 0.6 }) * ag.v
                };
            }
            if pausing || hold {
                speed = 0.0;
            }
            if car {
                // 2 m/s^2 up, at most 6 m/s^2 down
                let [lo, hi] = self.spec.road_s.unwrap_or([0.0, self.length]);
                let a = &self.agents[i];
                let to_end = if a.dir > 0.0 { hi - a.s } else { a.s - lo };
                cap = cap.min((2.5f64.powi(2) + 2.0 * 3.0 * to_end.max(0.0)).sqrt());
                speed = speed.min(cap);
                if let Some((gap_a, lat, need, follow, _)) = best {
                    if lat.abs() < need + 0.05 {
                        let gap = gap_a - need; // brake to stop 1 m short of them
                        speed = speed
                            .min(follow.unwrap_or(0.0) + (2.0 * 4.0 * (gap - 1.0).max(0.0)).sqrt());
                    }
                }
                let cur = a.cv;
                speed = if speed > cur {
                    speed.min(cur + 2.0 * dt)
                } else {
                    speed.max(cur - 6.0 * dt)
                };
                self.agents[i].cv = speed;
            }
            if best.is_none() {
                let a = &mut self.agents[i];
                a.waited = 0.0;
                a.dodge -= a.dodge.signum() * a.dodge.abs().min(0.3 * dt);
            }
            // Never step into someone: try the move with the new dodge, then without moving
            // on, then without the new dodge; else stay put.
            let scale = if car { self.lane_scale(i) } else { 1.0 };
            let a = &self.agents[i];
            let s_new = (a.s + a.dir * speed * dt * scale).clamp(0.0, self.length);
            let now = self.overlap(i, a.xy, a.yaw, robot);
            let mut tries = vec![(s_new, a.dodge), (a.s, a.dodge), (s_new, old_dodge)];
            if a.cross != 0 {
                if let Some(c) = self.spec.crossing_s {
                    for e in [0.15, -0.15] {
                        tries.push(((a.s + e).clamp(c - 1.2, c + 1.2), a.dodge));
                    }
                }
            }
            // deadlocked head-on on a narrow pavement for 16 s: people and bikes may brush past
            if !car && a.jam > 16.0 {
                self.agents[i].squeeze = true;
            } else if now <= 0.02 {
                self.agents[i].squeeze = false;
            }
            let slack = if self.agents[i].squeeze { 0.15 } else { 0.02 };
            let mut moved = false;
            let mut placed = false;
            for (s_try, dodge_try) in tries {
                let (xy, yaw, normal) = self.pose(i, s_try, dodge_try, self.agents[i].dir, solid);
                if self.clear(i, xy, yaw, robot, slack)
                    || (now > 0.02 && self.overlap(i, xy, yaw, robot) < now - 1e-4)
                {
                    let a = &mut self.agents[i];
                    moved = s_try != a.s;
                    a.s = s_try;
                    a.dodge = dodge_try;
                    (a.xy, a.yaw, a.normal) = (xy, yaw, normal);
                    placed = true;
                    break;
                }
            }
            let a = &mut self.agents[i];
            if !placed {
                a.dodge = old_dodge;
                a.side = old_side;
                a.side_goal = old_side; // that way is taken: stay on this line
            }
            if a.cross != 0 {
                // blocked on the crossing 4 s: go back
                let blocked = cross_want && a.side == old_side;
                a.cross_block = if blocked { a.cross_block + dt } else { 0.0 };
                if a.cross_block > 4.0 {
                    a.cross = if matches!(a.cross, 1 | 2) { 3 } else { 1 };
                    a.cross_block = 0.0;
                }
            }
            a.jam = if moved || pausing || (hold && a.side != old_side) {
                0.0
            } else {
                a.jam + dt
            };
            if moved || pausing || hold {
                a.stuck = 0.0;
                // one stride per 1.3 m on foot, one crank turn per 4.5 m on a bicycle
                let stride = if car {
                    2.1
                } else if bike {
                    4.5
                } else {
                    1.3
                };
                a.phase = (a.phase + speed * dt / stride).rem_euclid(1.0);
            } else {
                a.stuck += dt;
                speed = 0.0;
                a.cv = 0.0;
            }
            moving[i] = speed / a.v.max(1e-6);
            let [lo, hi] = if car {
                self.spec.road_s.unwrap_or([0.0, self.length])
            } else {
                [0.0, self.length]
            };
            // the end of the route, or (on foot) no way through: turn back
            if (a.s <= lo && a.dir < 0.0) || (a.s >= hi && a.dir > 0.0) || (a.stuck > 8.0 && !car) {
                a.dir = -a.dir;
                if car {
                    a.uturn = 1e-6;
                }
                a.stuck = 0.0;
                a.waited = 0.0;
            }
        }
        for (a, m) in self.agents.iter_mut().zip(moving) {
            a.moving = m;
        }
    }

    /// (person-person pairs, people within `robot_radius_m` of the robot) overlapping now.
    pub fn overlaps(&self, robot: [f64; 2], robot_radius_m: f64) -> (usize, usize) {
        let c = self.circles();
        let mut pairs = std::collections::BTreeSet::new();
        let mut near = std::collections::BTreeSet::new();
        for (k, p) in c.iter().enumerate() {
            if (p.0 - robot[0]).hypot(p.1 - robot[1]) < p.2 + robot_radius_m {
                near.insert(p.3);
            }
            for q in &c[k + 1..] {
                if p.3 != q.3 && (p.0 - q.0).hypot(p.1 - q.1) < p.2 + q.2 {
                    pairs.insert((p.3.min(q.3), p.3.max(q.3)));
                }
            }
        }
        (pairs.len(), near.len())
    }
}

/// visloc-rs `Grid.hits`: whether a circle of `r` at (x, y) touches a solid cell, sampled at
/// its centre and around its rim.
pub fn hits(solid: &dyn Fn(f64, f64) -> bool, x: f64, y: f64, r: f64) -> bool {
    let n = 8usize.max((2.0 * PI * r / 0.05) as usize);
    solid(x, y)
        || (0..n).any(|k| {
            let a = 2.0 * PI * k as f64 / n as f64;
            solid(x + r * a.cos(), y + r * a.sin())
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn walker(s: f64, dir: f64, side: f64) -> AgentSpec {
        AgentSpec {
            kind: WALKER,
            s,
            dir,
            v: 1.2,
            side,
            phase: 0.0,
            foot: vec![[0.0, 0.0, 0.25]],
            phases: Vec::new(),
        }
    }

    /// A straight 40 m route along x, a road 5 m to the left with two lanes, a crossing at 20 m.
    fn spec(agents: Vec<AgentSpec>) -> CrowdSpec {
        CrowdSpec {
            phases: 12,
            radius_m: 0.25,
            lane_half_m: 1.6,
            narrow_m: 0.9,
            yield_m: 1.2,
            route: vec![[0.0, 0.0], [40.0, 0.0]],
            pavement: Some(vec![[0.0, 2.0, 1.0], [40.0, 2.0, 1.0]]),
            walk_offsets: None,
            kerb_spans: Vec::new(),
            crossing_s: Some(20.0),
            decor: Default::default(),
            agents,
            road: vec![[0.0, 40.0, 5.0]],
            road_s: Some([1.0, 39.0]),
            road_lines: Some(vec![[0.0, 40.0, 0.0, 5.0, 1.0, 0.0]]),
            lanes: [
                ("1".to_string(), vec![[0.0, 3.4], [40.0, 3.4]]),
                ("-1".to_string(), vec![[0.0, 6.6], [40.0, 6.6]]),
            ]
            .into_iter()
            .collect(),
        }
    }

    fn open(_: f64, y: f64) -> bool {
        y.abs() > 20.0
    }

    #[test]
    fn walkers_meeting_head_on_pass_without_overlapping() {
        let mut crowd = Crowd::new(
            spec(vec![walker(10.0, 1.0, 0.0), walker(14.0, -1.0, 0.0)]),
            1,
            &open,
        );
        let far = [1e6, 1e6];
        let mut deepest = f64::NEG_INFINITY;
        for _ in 0..400 {
            crowd.step(0.05, far, &open);
            let c = crowd.circles();
            deepest = deepest.max(c[0].2 + c[1].2 - (c[0].0 - c[1].0).hypot(c[0].1 - c[1].1));
        }
        assert!(deepest < 0.02 + 1e-9, "{deepest}");
        let (a, b) = (crowd.agent(0).1, crowd.agent(1).1);
        assert!(a[0] > 14.0 && b[0] < 10.0, "they passed: {a:?} {b:?}");
        // and kept to the pavement: the road side's edge is 2 m left of the route
        assert!(a[1] < 1.75 && b[1] < 1.75);
    }

    #[test]
    fn a_car_keeps_its_lane_and_waits_at_a_red_light() {
        let car = AgentSpec {
            kind: CAR,
            s: 5.0,
            dir: 1.0,
            v: 6.0,
            side: 3.4,
            phase: 0.0,
            foot: vec![[-1.4, 0.0, 0.9], [0.0, 0.0, 0.9], [1.4, 0.0, 0.9]],
            phases: Vec::new(),
        };
        let mut crowd = Crowd::new(spec(vec![car]), 1, &open);
        let far = [1e6, 1e6];
        let mut last = crowd.agent(0).1[0];
        for _ in 0..(90.0 / 0.05) as usize {
            crowd.step(0.05, far, &open);
            let (_, xy, yaw, _) = crowd.agent(0);
            if xy[0] > 2.0 && xy[0] < 37.0 {
                // in one of the two lanes, keeping right (U-turns only at the road's ends)
                let lane = if yaw.cos() > 0.0 { 3.4 } else { 6.6 };
                assert!((xy[1] - lane).abs() < 1e-6, "in its lane: {xy:?} {yaw}");
            }
            if crowd.light() == Light::Red {
                // never over the crossing's line on red
                assert!(
                    (last - 20.0).signum() == (xy[0] - 20.0).signum() || (last - 20.0).abs() < 1e-9,
                    "crossed on red: {last} -> {}",
                    xy[0]
                );
            }
            last = xy[0];
        }
    }

    #[test]
    fn a_crowd_replays_exactly() {
        let make = || {
            Crowd::new(
                spec(vec![walker(10.0, 1.0, 0.5), walker(30.0, -1.0, -0.5)]),
                7,
                &open,
            )
        };
        let (mut a, mut b) = (make(), make());
        for _ in 0..500 {
            a.step(0.05, [12.0, 0.0], &open);
            b.step(0.05, [12.0, 0.0], &open);
        }
        assert_eq!(format!("{:?}", a.circles()), format!("{:?}", b.circles()));
    }
}
