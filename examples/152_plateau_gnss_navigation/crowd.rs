//! Pedestrians on the sidewalks along the route, and the weather.

use rne_core::DeterministicRng;
use rne_gnss::{SceneBox, SurfaceMaterial};
use rne_math::Vec3;

/// Body radius of a pedestrian for the LiDAR, meters.
pub(crate) const PEDESTRIAN_RADIUS_M: f64 = 0.28;
/// Pedestrians keep at least this far from the robot's centre, meters.
pub(crate) const PERSONAL_SPACE_M: f64 = 0.9;
/// Height of a pedestrian, meters.
pub(crate) const PEDESTRIAN_HEIGHT_M: f64 = 1.7;

/// Conditions of a run.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub(crate) struct Weather {
    /// Pedestrians walking along the route.
    pub pedestrians: usize,
    /// Rain: LiDAR dropouts, noise and drop returns; wet, stronger GNSS
    /// reflectors.
    pub rain: bool,
    /// People crossing the intersection at the start of the route in waves
    /// on the walk signal, straight across and diagonally (a scramble
    /// crossing).
    pub scramble: usize,
    /// Vehicles on the street beside the route.
    pub vehicles: usize,
}

/// One pedestrian walking back and forth along a stretch of the route.
#[derive(Clone, Copy, Debug)]
struct Walker {
    start_m: f64,
    span_m: f64,
    lateral_m: f64,
    speed_m_s: f64,
    phase_m: f64,
}

/// One pedestrian of a scramble crossing: from one corner to another on each
/// walk phase, waiting at the corner otherwise.
#[derive(Clone, Copy, Debug)]
struct Crosser {
    corners: [(f64, f64); 2],
    delay_s: f64,
    speed_m_s: f64,
}

/// Signal cycle of the scramble crossing, seconds, and its walk phase.
const SIGNAL_CYCLE_S: f64 = 100.0;
const WALK_PHASE_S: f64 = 45.0;
/// Half-size of the crossing (corner to centre along each axis), meters.
const CROSSING_HALF_M: f64 = 14.0;

/// Pedestrians walking the sidewalks along a route, 1.2-3 m to either side
/// of it, at 0.9-1.6 m/s, each turning back at the ends of its 30-90 m
/// stretch, and optionally a scramble crossing at the route's start.
/// Deterministic from the seed.
#[derive(Clone, Debug)]
pub(crate) struct Crowd {
    route: Vec<(f64, f64)>,
    cumulative_m: Vec<f64>,
    walkers: Vec<Walker>,
    crossing: (f64, f64),
    crossers: Vec<Crosser>,
}

impl Crowd {
    pub(crate) fn new(route: &[(f64, f64)], count: usize, seed: u64) -> Self {
        let mut cumulative_m = vec![0.0];
        for pair in route.windows(2) {
            let length = (pair[1].0 - pair[0].0).hypot(pair[1].1 - pair[0].1);
            cumulative_m.push(cumulative_m.last().expect("start") + length);
        }
        let total = *cumulative_m.last().expect("length");
        let mut rng = DeterministicRng::new(seed);
        let walkers = (0..count)
            .map(|_| {
                let span_m = rng.uniform_f64(30.0, 90.0).min(total);
                let side = if rng.uniform_f64(0.0, 1.0) < 0.5 {
                    -1.0
                } else {
                    1.0
                };
                Walker {
                    start_m: rng.uniform_f64(0.0, (total - span_m).max(0.0)),
                    span_m,
                    lateral_m: side * rng.uniform_f64(1.2, 3.0),
                    speed_m_s: rng.uniform_f64(0.9, 1.6),
                    phase_m: rng.uniform_f64(0.0, 2.0 * span_m),
                }
            })
            .collect();
        Self {
            route: route.to_vec(),
            cumulative_m,
            walkers,
            crossing: route[0],
            crossers: Vec::new(),
        }
    }

    /// Adds `count` people crossing the intersection at the route's start:
    /// each walk phase every one walks from its corner to another one
    /// (straight across or diagonally), leaving within the first 10 s.
    pub(crate) fn with_scramble(mut self, count: usize, seed: u64) -> Self {
        let mut rng = DeterministicRng::new(seed);
        let h = CROSSING_HALF_M;
        let corners = [(h, h), (-h, h), (-h, -h), (h, -h)];
        self.crossers = (0..count)
            .map(|_| {
                let from = (rng.uniform_f64(0.0, 4.0) as usize).min(3);
                let step = 1 + (rng.uniform_f64(0.0, 3.0) as usize).min(2);
                let jitter = |rng: &mut DeterministicRng, c: (f64, f64)| {
                    (
                        c.0 + rng.uniform_f64(-2.0, 2.0),
                        c.1 + rng.uniform_f64(-2.0, 2.0),
                    )
                };
                let a = jitter(&mut rng, corners[from]);
                let b = jitter(&mut rng, corners[(from + step) % 4]);
                Crosser {
                    corners: [a, b],
                    delay_s: rng.uniform_f64(0.0, 10.0),
                    speed_m_s: rng.uniform_f64(1.0, 1.5),
                }
            })
            .collect();
        self
    }

    /// Where a crosser is at `time_s`: it goes one way per walk phase.
    fn crosser_at(&self, c: &Crosser, time_s: f64) -> (f64, f64) {
        let cycle = (time_s / SIGNAL_CYCLE_S).floor();
        let into = time_s - cycle * SIGNAL_CYCLE_S;
        // Even cycles a -> b, odd cycles b -> a.
        let [a, b] = if cycle as i64 % 2 == 0 {
            c.corners
        } else {
            [c.corners[1], c.corners[0]]
        };
        let length = (b.0 - a.0).hypot(b.1 - a.1).max(1.0e-9);
        let walked = if into < WALK_PHASE_S {
            ((into - c.delay_s) * c.speed_m_s).clamp(0.0, length)
        } else {
            length
        };
        let f = walked / length;
        (
            self.crossing.0 + a.0 + f * (b.0 - a.0),
            self.crossing.1 + a.1 + f * (b.1 - a.1),
        )
    }

    /// Pedestrian positions at `time_s`; anyone walking the sidewalk who
    /// would come closer than [`PERSONAL_SPACE_M`] to the robot at `robot`
    /// steps around it. People on the scramble crossing do not: the robot
    /// has to find its way through them.
    pub(crate) fn positions(&self, time_s: f64, robot: (f64, f64)) -> Vec<(f64, f64)> {
        let walking = self.walkers.iter().map(|w| {
            // Back and forth over the span.
            let d = (w.phase_m + w.speed_m_s * time_s).rem_euclid(2.0 * w.span_m);
            let along = if d < w.span_m { d } else { 2.0 * w.span_m - d };
            (
                self.point_beside(w.start_m + along, w.lateral_m),
                w.lateral_m,
            )
        });
        let crossing = self.crossers.iter().map(|c| self.crosser_at(c, time_s));
        walking
            .map(|((x, y), side)| {
                let (dx, dy) = (x - robot.0, y - robot.1);
                let distance = dx.hypot(dy);
                if distance >= PERSONAL_SPACE_M {
                    (x, y)
                } else if distance > 1.0e-6 {
                    let scale = PERSONAL_SPACE_M / distance;
                    (robot.0 + dx * scale, robot.1 + dy * scale)
                } else {
                    (robot.0 + side.signum() * PERSONAL_SPACE_M, robot.1)
                }
            })
            .chain(crossing)
            .collect()
    }

    fn point_beside(&self, s_m: f64, lateral_m: f64) -> (f64, f64) {
        let leg = self
            .cumulative_m
            .partition_point(|&c| c <= s_m)
            .clamp(1, self.route.len() - 1);
        let (a, b) = (self.route[leg - 1], self.route[leg]);
        let length = (b.0 - a.0).hypot(b.1 - a.1).max(1.0e-9);
        let f = ((s_m - self.cumulative_m[leg - 1]) / length).clamp(0.0, 1.0);
        let (dx, dy) = ((b.0 - a.0) / length, (b.1 - a.1) / length);
        (
            a.0 + f * (b.0 - a.0) - dy * lateral_m,
            a.1 + f * (b.1 - a.1) + dx * lateral_m,
        )
    }
}

/// Pedestrians as GNSS occluders (bodies absorb rather than mirror).
pub(crate) fn pedestrian_boxes(
    positions: &[(f64, f64)],
    ground: impl Fn(f64, f64) -> f64,
) -> Vec<SceneBox> {
    positions
        .iter()
        .map(|&(x, y)| {
            let z = ground(x, y);
            SceneBox::new(
                Vec3::new(x - 0.25, y - 0.2, z),
                Vec3::new(x + 0.25, y + 0.2, z + PEDESTRIAN_HEIGHT_M),
                SurfaceMaterial::ABSORBER,
            )
        })
        .collect()
}

/// Range along a ray from `(ox, oy)` in direction `(dx, dy)` to the nearest
/// pedestrian body, if any.
pub(crate) fn pedestrian_range(
    positions: &[(f64, f64)],
    ox: f64,
    oy: f64,
    dx: f64,
    dy: f64,
) -> Option<f64> {
    positions
        .iter()
        .filter_map(|&(px, py)| {
            let (fx, fy) = (ox - px, oy - py);
            let b = fx * dx + fy * dy;
            let c = fx * fx + fy * fy - PEDESTRIAN_RADIUS_M * PEDESTRIAN_RADIUS_M;
            let disc = b * b - c;
            (disc >= 0.0).then(|| -b - disc.sqrt()).filter(|t| *t > 0.0)
        })
        .fold(None, |best: Option<f64>, t| {
            Some(best.map_or(t, |b| b.min(t)))
        })
}

/// Wet surfaces reflect GNSS more strongly (a water film raises the
/// reflection coefficient): concrete about 2 dB, asphalt about 3 dB less
/// loss. Rain itself attenuates L-band by well under 0.1 dB over a
/// slant path, so it is not modelled.
pub(crate) fn wet(material: SurfaceMaterial) -> SurfaceMaterial {
    if material == SurfaceMaterial::CONCRETE {
        SurfaceMaterial {
            reflection_loss_db: 6.0,
        }
    } else if material == SurfaceMaterial::GROUND {
        SurfaceMaterial {
            reflection_loss_db: 9.0,
        }
    } else if material == SurfaceMaterial::GLASS {
        SurfaceMaterial {
            reflection_loss_db: 3.5,
        }
    } else {
        material
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn walkers_stay_beside_the_route_and_bounce_within_their_stretch() {
        let route = [(0.0, 0.0), (100.0, 0.0), (100.0, 50.0)];
        let crowd = Crowd::new(&route, 20, 7);
        for t in [0.0, 13.0, 77.0, 400.0] {
            for (x, y) in crowd.positions(t, (-100.0, -100.0)) {
                // Within 3 m of one of the legs.
                let near_first = (0.0..=100.0).contains(&x) && y.abs() <= 3.0 + 1e-9;
                let near_second = (97.0..=103.0).contains(&x) && (-3.0..=53.0).contains(&y);
                assert!(near_first || near_second, "({x}, {y}) at {t}");
            }
        }
    }

    #[test]
    fn pedestrians_step_around_the_robot() {
        let route = [(0.0, 0.0), (100.0, 0.0)];
        let crowd = Crowd::new(&route, 40, 9);
        for t in 0..200 {
            let robot = (f64::from(t) * 0.5, 0.0);
            for (x, y) in crowd.positions(f64::from(t), robot) {
                assert!((x - robot.0).hypot(y - robot.1) >= PERSONAL_SPACE_M - 1e-9);
            }
        }
    }

    #[test]
    fn the_scramble_crossing_moves_on_the_walk_signal() {
        let route = [(0.0, 0.0), (100.0, 0.0)];
        let crowd = Crowd::new(&route, 0, 3).with_scramble(30, 4);
        let far = (1.0e4, 1.0e4);
        let at = |t: f64| crowd.positions(t, far);
        // Everyone stays on the crossing (corners at +-14 m, jitter 2 m).
        for t in [0.0, 20.0, 44.0, 60.0, 130.0] {
            for (x, y) in at(t) {
                assert!(x.abs() <= 16.5 && y.abs() <= 16.5, "({x}, {y}) at {t}");
            }
        }
        // They walk during the walk phase and stand still after it.
        let moved = |a: f64, b: f64| {
            at(a)
                .iter()
                .zip(at(b))
                .filter(|(p, q)| (p.0 - q.0).hypot(p.1 - q.1) > 0.5)
                .count()
        };
        assert!(moved(12.0, 18.0) >= 25, "{}", moved(12.0, 18.0));
        assert_eq!(moved(60.0, 90.0), 0);
        // The next cycle takes them back.
        let start = at(0.0);
        let back = at(2.0 * SIGNAL_CYCLE_S);
        assert!(start
            .iter()
            .zip(&back)
            .all(|(p, q)| (p.0 - q.0).hypot(p.1 - q.1) < 1.0e-9));
    }

    #[test]
    fn a_ray_hits_the_nearest_body() {
        let people = [(5.0, 0.0), (3.0, 0.1), (3.0, 5.0)];
        let r = pedestrian_range(&people, 0.0, 0.0, 1.0, 0.0).unwrap();
        assert!(
            (r - (3.0 - (0.28_f64.powi(2) - 0.01).sqrt())).abs() < 1e-9,
            "{r}"
        );
        assert!(pedestrian_range(&people, 0.0, 0.0, -1.0, 0.0).is_none());
    }
}
