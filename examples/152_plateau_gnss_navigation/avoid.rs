//! `--avoid`: a local planner that walks around people instead of waiting.
//!
//! Each scan is placed at the estimated pose. Returns the prior map cannot
//! explain (in its free space, away from mapped structure) are grouped into
//! people and tracked with a constant-velocity tracker; returns off mapped
//! structure within a metre (poles, facades) become static obstacles.
//! `rne_nav::avoid_velocities` then picks the command closest to the route
//! follower's whose two-second rollout clears all of them, predicting the
//! people's motion, or stops.

use rne_math::Vec3;
use rne_nav::{
    avoid_velocities, AvoidanceConfig, CircularObstacle, Detection, LaserScan2d, ObstacleTracker,
    ObstacleTrackerConfig, OccupancyGrid, Pose2d, VelocityCommand2d,
};

/// Robot footprint radius, meters.
const ROBOT_RADIUS_M: f64 = 0.35;
/// People are tracked within this range, meters.
const TRACK_RANGE_M: f64 = 8.0;
/// Mapped returns within this range are static obstacles, meters. The route
/// is planned clear of mapped structure, so only what the robot has come
/// close to while stepping around people counts; more would keep the
/// two-second rollouts slowing it past every pole.
const STATIC_RANGE_M: f64 = 1.0;
/// A return this far (or more) from any mapped occupied cell is unmapped.
const UNMAPPED_M: f64 = 0.4;
/// Consecutive unmapped returns closer than this belong to one person.
const CLUSTER_GAP_M: f64 = 0.5;
/// Clusters are cut into pieces no longer than this (vehicles), meters.
const MAX_PIECE_M: f64 = 1.0;

/// The tracker and its clock.
pub(crate) struct Avoider {
    tracker: ObstacleTracker,
    last_scan_s: f64,
    obstacles: Vec<CircularObstacle>,
}

impl Avoider {
    pub(crate) fn new() -> Self {
        Self {
            tracker: ObstacleTracker::new(ObstacleTrackerConfig {
                gate_m: 0.8,
                max_missed: 3,
                velocity_gain: 0.4,
            })
            .expect("tracker"),
            last_scan_s: f64::NAN,
            obstacles: Vec::new(),
        }
    }

    /// Updates the obstacles from a scan taken at the estimated pose.
    pub(crate) fn sense(
        &mut self,
        scan: &LaserScan2d,
        estimate: Pose2d,
        map: &OccupancyGrid,
        t: f64,
    ) {
        let mut people: Vec<Vec<Vec3>> = Vec::new();
        let mut statics = Vec::new();
        let mut previous: Option<Vec3> = None;
        for (i, &range) in scan.ranges_m.iter().enumerate() {
            if !range.is_finite() || range > TRACK_RANGE_M {
                previous = None;
                continue;
            }
            let angle = estimate.yaw_rad + scan.angle_min_rad + i as f64 * scan.angle_increment_rad;
            let p = Vec3::new(
                estimate.x_m + range * angle.cos(),
                estimate.y_m + range * angle.sin(),
                0.0,
            );
            if unmapped(map, p) {
                match (previous, people.last_mut()) {
                    (Some(q), Some(group)) if (p - q).length() < CLUSTER_GAP_M => group.push(p),
                    _ => people.push(vec![p]),
                }
                previous = Some(p);
            } else {
                previous = None;
                if range <= STATIC_RANGE_M && i % 2 == 0 {
                    statics.push(CircularObstacle {
                        center_m: p,
                        velocity_m_s: Vec3::ZERO,
                        radius_m: 0.05,
                    });
                }
            }
        }
        // The scan wraps around: a person straddling its seam is one person.
        if people.len() >= 2 {
            let (first, last) = (
                people[0][0],
                *people[people.len() - 1].last().expect("point"),
            );
            if (first - last).length() < CLUSTER_GAP_M {
                let tail = people.pop().expect("group");
                people[0].extend(tail);
            }
        }
        // A long cluster (the side of a bus) is cut into pieces of at most
        // `MAX_PIECE_M`: one circle around it would cover the whole street.
        let detections: Vec<Detection> = people
            .iter()
            .filter(|g| g.len() >= 2)
            .flat_map(|g| {
                let mut pieces: Vec<Vec<Vec3>> = vec![Vec::new()];
                for p in g {
                    let piece = pieces.last_mut().expect("piece");
                    if piece
                        .first()
                        .is_some_and(|q| (*p - *q).length() > MAX_PIECE_M)
                    {
                        pieces.push(vec![*p]);
                    } else {
                        piece.push(*p);
                    }
                }
                pieces.into_iter().map(|piece| {
                    let center = piece.iter().fold(Vec3::ZERO, |a, p| a + *p) / piece.len() as f64;
                    let radius = piece
                        .iter()
                        .map(|p| (*p - center).length())
                        .fold(0.0, f64::max);
                    Detection {
                        position_m: center,
                        radius_m: radius.max(0.25),
                    }
                })
            })
            .collect();
        let dt = if self.last_scan_s.is_finite() {
            (t - self.last_scan_s).max(1.0e-3)
        } else {
            0.1
        };
        self.last_scan_s = t;
        self.tracker.update(&detections, dt).expect("tracker");
        self.obstacles = statics;
        self.obstacles
            .extend(self.tracker.tracks().iter().map(|track| CircularObstacle {
                center_m: track.position_m,
                velocity_m_s: track.velocity_m_s,
                radius_m: track.radius_m,
            }));
    }

    /// The command closest to `desired` that clears every obstacle for two
    /// seconds, or, in a crowd where none does, for one or half a second:
    /// standing still among people who do not give way is not safer than
    /// moving on (the "freezing robot"). Turning in place passes through
    /// unchanged.
    pub(crate) fn command(
        &self,
        desired: VelocityCommand2d,
        estimate: Pose2d,
    ) -> VelocityCommand2d {
        if desired.linear_m_s <= 0.0 {
            return desired;
        }
        for horizon_s in [2.0, 1.0, 0.5] {
            let command = avoid_velocities(
                estimate,
                ROBOT_RADIUS_M,
                &self.obstacles,
                desired,
                1.2,
                1.0,
                &AvoidanceConfig {
                    time_horizon_s: horizon_s,
                    simulation_step_s: 0.1,
                    linear_samples: 7,
                    angular_samples: 15,
                    safety_margin_m: 0.15,
                },
            );
            if command != VelocityCommand2d::ZERO {
                return command;
            }
        }
        VelocityCommand2d::ZERO
    }
}

/// Whether a point lies in the map's free space at least `UNMAPPED_M` from
/// any occupied cell (outside the map counts as mapped: nothing is known).
fn unmapped(map: &OccupancyGrid, p: Vec3) -> bool {
    let Some(coord) = map.world_to_grid(p) else {
        return false;
    };
    if map.is_occupied(coord) {
        return false;
    }
    let cells = (UNMAPPED_M / map.resolution_m()).ceil() as isize;
    for dy in -cells..=cells {
        for dx in -cells..=cells {
            let c = rne_nav::GridCoord {
                x: coord.x + dx,
                y: coord.y + dy,
            };
            if map.is_occupied(c) {
                return false;
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_nav::FrameId;
    use std::f64::consts::TAU;

    /// An open map (all free) with a wall along y = 3 m.
    fn map() -> OccupancyGrid {
        let mut grid =
            OccupancyGrid::new(200, 200, 0.1, Pose2d::new(-10.0, -10.0, 0.0)).expect("grid");
        for y in 0..200 {
            for x in 0..200 {
                let c = rne_nav::GridCoord { x, y };
                if y == 130 {
                    grid.mark_occupied(c);
                } else {
                    grid.mark_free(c);
                }
            }
        }
        grid
    }

    /// A scan from the origin seeing the wall and a person at `person`.
    fn scan(person: (f64, f64)) -> LaserScan2d {
        let beams = 360;
        let ranges_m = (0..beams)
            .map(|i| {
                let a = TAU * f64::from(i) / f64::from(beams);
                let (dx, dy) = (a.cos(), a.sin());
                let mut r = f64::INFINITY;
                if dy > 1e-9 {
                    r = 3.0 / dy;
                }
                let (fx, fy) = (-person.0, -person.1);
                let b = fx * dx + fy * dy;
                let disc = b * b - (fx * fx + fy * fy - 0.25 * 0.25);
                if disc >= 0.0 && -b - disc.sqrt() > 0.0 {
                    r = r.min(-b - disc.sqrt());
                }
                r
            })
            .collect();
        LaserScan2d {
            time_s: 0.0,
            frame: FrameId::new("laser"),
            angle_min_rad: 0.0,
            angle_increment_rad: TAU / f64::from(beams),
            range_min_m: 0.05,
            range_max_m: 40.0,
            ranges_m,
        }
    }

    #[test]
    fn a_person_ahead_is_walked_around_and_the_wall_is_not_one() {
        let map = map();
        let mut avoider = Avoider::new();
        let pose = Pose2d::new(0.0, 0.0, 0.0);
        for k in 0..3 {
            avoider.sense(&scan((1.4, 0.0)), pose, &map, 0.1 * f64::from(k));
        }
        // One tracked person; the wall is mapped, not a person.
        assert_eq!(avoider.tracker.len(), 1);
        let desired = VelocityCommand2d::new(1.2, 0.0);
        let command = avoider.command(desired, pose);
        assert!(
            command.angular_rad_s.abs() > 0.1 || command.linear_m_s < 0.6,
            "{command:?}"
        );
        // With the person gone, the route command passes unchanged.
        let mut clear = Avoider::new();
        clear.sense(&scan((50.0, 50.0)), pose, &map, 0.0);
        assert_eq!(clear.command(desired, pose), desired);
        assert!(unmapped(&map, Vec3::new(1.0, 0.0, 0.0)));
        assert!(!unmapped(&map, Vec3::new(1.0, 2.9, 0.0)));
    }
}
