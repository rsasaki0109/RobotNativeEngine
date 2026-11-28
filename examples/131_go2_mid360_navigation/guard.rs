//! A footprint guard for `--explore`: before the trot follows a command, the
//! Go2's footprint (a rectangle that turns with the body, not a disc) is
//! rolled forward under it, and a command that would bring the body too close
//! to an obstacle is replaced by the best safe one nearby.
//!
//! The planner keeps the base off obstacles by a disc radius. That cannot
//! tell a turn in place, which swings the rear legs 0.38 m behind the base
//! through an arc, from walking past along the same spot; the guard can.

use rne_math::Vec3;
use rne_nav::{GridCoord, OccupancyGrid, Pose2d};

/// The Go2's footprint in the body frame (x forward, y left): head, hips,
/// knees and feet measured over a walk reach 0.285 m ahead, 0.382 m behind
/// and 0.18 m to each side of the base; 2–3 cm more for the links' size.
pub(crate) const FOOTPRINT_BACK_M: f64 = 0.40;
pub(crate) const FOOTPRINT_FRONT_M: f64 = 0.31;
pub(crate) const FOOTPRINT_HALF_WIDTH_M: f64 = 0.20;
/// Spacing of the sample points on the footprint's outline.
const FOOTPRINT_STEP_M: f64 = 0.05;
/// The footprint must stay this far from obstacles over the horizon.
pub(crate) const MARGIN_M: f64 = 0.08;
/// Roll-out horizon and step.
const HORIZON_S: f64 = 1.0;
const ROLLOUT_DT_S: f64 = 0.1;
/// Candidate commands when the nominal one is not safe.
const SPEEDS_M_S: [f64; 5] = [-0.10, 0.0, 0.05, 0.12, 0.25];
const YAW_RATES_RAD_S: [f64; 5] = [-0.5, -0.25, 0.0, 0.25, 0.5];
/// Distances beyond this are not told apart.
const FIELD_CAP_M: f64 = 1.0;

/// Distance from each cell to the nearest occupied cell, by a two-pass
/// chamfer over the grid, capped at [`FIELD_CAP_M`].
#[derive(Clone, Debug)]
pub(crate) struct ClearanceField {
    distance_m: Vec<f64>,
    width: usize,
    height: usize,
    grid: OccupancyGrid,
}

impl ClearanceField {
    pub(crate) fn from_grid(grid: &OccupancyGrid) -> Self {
        let (width, height) = (grid.width(), grid.height());
        let cell = grid.resolution_m();
        let mut d = vec![FIELD_CAP_M; width * height];
        for y in 0..height {
            for x in 0..width {
                let coord = GridCoord {
                    x: x as isize,
                    y: y as isize,
                };
                if grid.is_occupied(coord) {
                    d[y * width + x] = 0.0;
                }
            }
        }
        let diagonal = cell * std::f64::consts::SQRT_2;
        let relax = |d: &mut Vec<f64>, x: usize, y: usize, nx: isize, ny: isize, step: f64| {
            if nx < 0 || ny < 0 || nx >= width as isize || ny >= height as isize {
                return;
            }
            let candidate = d[ny as usize * width + nx as usize] + step;
            if candidate < d[y * width + x] {
                d[y * width + x] = candidate;
            }
        };
        for y in 0..height {
            for x in 0..width {
                let (xi, yi) = (x as isize, y as isize);
                relax(&mut d, x, y, xi - 1, yi, cell);
                relax(&mut d, x, y, xi, yi - 1, cell);
                relax(&mut d, x, y, xi - 1, yi - 1, diagonal);
                relax(&mut d, x, y, xi + 1, yi - 1, diagonal);
            }
        }
        for y in (0..height).rev() {
            for x in (0..width).rev() {
                let (xi, yi) = (x as isize, y as isize);
                relax(&mut d, x, y, xi + 1, yi, cell);
                relax(&mut d, x, y, xi, yi + 1, cell);
                relax(&mut d, x, y, xi + 1, yi + 1, diagonal);
                relax(&mut d, x, y, xi - 1, yi + 1, diagonal);
            }
        }
        Self {
            distance_m: d,
            width,
            height,
            grid: grid.clone(),
        }
    }

    /// Distance from a point to the nearest occupied cell's edge; outside the
    /// map counts as an obstacle.
    fn at(&self, x_m: f64, y_m: f64) -> f64 {
        match self.grid.world_to_grid(Vec3::new(x_m, y_m, 0.0)) {
            Some(c) if (c.x as usize) < self.width && (c.y as usize) < self.height => (self
                .distance_m[c.y as usize * self.width + c.x as usize]
                - 0.5 * self.grid.resolution_m())
            .max(0.0),
            _ => 0.0,
        }
    }

    /// The footprint's smallest distance to an obstacle at `pose`.
    pub(crate) fn footprint_clearance(&self, pose: Pose2d) -> f64 {
        footprint_outline()
            .map(|(fx, fy)| {
                let p = pose.transform_point(Vec3::new(fx, fy, 0.0));
                self.at(p.x, p.y)
            })
            .fold(f64::INFINITY, f64::min)
    }

    /// The footprint's smallest distance to an obstacle over the horizon,
    /// following `(speed, yaw rate)` from `pose`, and the pose at the end.
    pub(crate) fn rollout(
        &self,
        pose: Pose2d,
        speed_m_s: f64,
        yaw_rate_rad_s: f64,
    ) -> (f64, Pose2d) {
        let mut at = pose;
        let mut clearance = self.footprint_clearance(at);
        let steps = (HORIZON_S / ROLLOUT_DT_S).round() as usize;
        for _ in 0..steps {
            at = at.compose(Pose2d::new(
                speed_m_s * ROLLOUT_DT_S,
                0.0,
                yaw_rate_rad_s * ROLLOUT_DT_S,
            ));
            clearance = clearance.min(self.footprint_clearance(at));
        }
        (clearance, at)
    }
}

/// Sample points on the footprint rectangle's outline, body frame.
fn footprint_outline() -> impl Iterator<Item = (f64, f64)> {
    let along = ((FOOTPRINT_FRONT_M + FOOTPRINT_BACK_M) / FOOTPRINT_STEP_M).ceil() as usize;
    let across = ((2.0 * FOOTPRINT_HALF_WIDTH_M) / FOOTPRINT_STEP_M).ceil() as usize;
    let sides = (0..=along).flat_map(move |i| {
        let x =
            -FOOTPRINT_BACK_M + (FOOTPRINT_FRONT_M + FOOTPRINT_BACK_M) * i as f64 / along as f64;
        [(x, FOOTPRINT_HALF_WIDTH_M), (x, -FOOTPRINT_HALF_WIDTH_M)]
    });
    let ends = (1..across).flat_map(move |j| {
        let y = -FOOTPRINT_HALF_WIDTH_M + 2.0 * FOOTPRINT_HALF_WIDTH_M * j as f64 / across as f64;
        [(FOOTPRINT_FRONT_M, y), (-FOOTPRINT_BACK_M, y)]
    });
    sides.chain(ends)
}

/// Whether a roll-out is acceptable from a start with `now` clearance: it
/// keeps the margin, or, already inside it, gets no closer.
fn acceptable(rollout_clearance: f64, now: f64) -> bool {
    rollout_clearance >= MARGIN_M.min(now) - 1e-9
}

/// The command to follow: `nominal` if its roll-out is acceptable, else the
/// acceptable candidate that best heads for `target`, then moves, keeps
/// clear and stays near `current` (the command being sent, so the trot is
/// not jerked), else the candidate that keeps the most clearance. Returns
/// the command and whether it replaced the nominal one.
pub(crate) fn choose(
    field: &ClearanceField,
    pose: Pose2d,
    nominal: (f64, f64),
    current: (f64, f64),
    target: Vec3,
) -> ((f64, f64), bool) {
    let now = field.footprint_clearance(pose);
    if acceptable(field.rollout(pose, nominal.0, nominal.1).0, now) {
        return (nominal, false);
    }
    let mut best: Option<(f64, (f64, f64))> = None;
    let mut safest: Option<(f64, (f64, f64))> = None;
    for &speed in &SPEEDS_M_S {
        for &yaw_rate in &YAW_RATES_RAD_S {
            let (clearance, end) = field.rollout(pose, speed, yaw_rate);
            if safest.is_none_or(|(c, _)| clearance > c) {
                safest = Some((clearance, (speed, yaw_rate)));
            }
            if !acceptable(clearance, now) {
                continue;
            }
            let bearing = (target.y - end.y_m).atan2(target.x - end.x_m) - end.yaw_rad;
            let change = (speed - current.0).abs() / SPEEDS_M_S[4]
                + (yaw_rate - current.1).abs() / YAW_RATES_RAD_S[4];
            let score =
                bearing.cos() + 0.5 * speed / SPEEDS_M_S[4] + 0.5 * clearance.min(0.3) / 0.3
                    - 0.3 * change;
            if best.is_none_or(|(s, _)| score > s) {
                best = Some((score, (speed, yaw_rate)));
            }
        }
    }
    let chosen = best.or(safest).map_or((0.0, 0.0), |(_, command)| command);
    (chosen, true)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A 4 m × 4 m room with a wall along y = 0.6 (cells occupied there).
    fn room_with_wall() -> ClearanceField {
        let mut grid = OccupancyGrid::new(80, 80, 0.05, Pose2d::new(-2.0, -2.0, 0.0)).unwrap();
        for x in 0..80 {
            let c = grid
                .world_to_grid(Vec3::new(-2.0 + 0.05 * x as f64 + 0.025, 0.6, 0.0))
                .unwrap();
            for _ in 0..10 {
                grid.mark_occupied(c);
            }
        }
        for y in 0..80 {
            for x in 0..80 {
                let c = GridCoord { x, y };
                if !grid.is_occupied(c) {
                    grid.mark_free(c);
                }
            }
        }
        ClearanceField::from_grid(&grid)
    }

    #[test]
    fn footprint_clearance_turns_with_the_body() {
        let field = room_with_wall();
        // Facing along the wall, the side is 0.2 m from the base: 0.6 - 0.2.
        let along = field.footprint_clearance(Pose2d::new(0.0, 0.0, 0.0));
        // Facing away from it, the rear reaches 0.4 m toward it: 0.6 - 0.4.
        let away = field.footprint_clearance(Pose2d::new(0.0, 0.0, -std::f64::consts::FRAC_PI_2));
        // Within a cell (5 cm) plus rounding at cell edges.
        assert!((along - 0.375).abs() < 0.06, "{along}");
        assert!((away - 0.175).abs() < 0.06, "{away}");
        assert!(along > away + 0.1, "{along} {away}");
    }

    #[test]
    fn a_turn_that_swings_the_rear_into_a_wall_is_replaced() {
        let field = room_with_wall();
        // Base 0.4 m from the wall, facing along it; turning to face away
        // swings the rear legs toward the wall.
        let pose = Pose2d::new(0.0, 0.2, 0.0);
        let target = Vec3::new(0.0, -2.0, 0.0);
        let turn_right = (0.0, -0.5);
        let (rollout, _) = field.rollout(pose, turn_right.0, turn_right.1);
        assert!(
            rollout < MARGIN_M,
            "the turn should come within the margin: {rollout}"
        );
        let (command, replaced) = choose(&field, pose, turn_right, turn_right, target);
        assert!(replaced);
        assert!(field.rollout(pose, command.0, command.1).0 >= MARGIN_M);
        // Walking on along the wall is safe and kept.
        let (command, replaced) = choose(
            &field,
            Pose2d::new(0.0, -0.2, 0.0),
            (0.25, 0.0),
            (0.25, 0.0),
            Vec3::new(2.0, -0.2, 0.0),
        );
        assert!(!replaced);
        assert_eq!(command, (0.25, 0.0));
    }
}
