//! Walkers crossing the robot's route, as splat actors.
//!
//! `RNE_WALKERS=DIR` loads `DIR/walker_<i>_phase_<k>.ply` (visloc-rs
//! `ros2/visloc-ros2/gs/walker_splat_plys.py`: x forward, z up, feet at z = 0)
//! for `RNE_WALKER_COUNT` walkers (default 4), scaled by `RNE_WALKER_SCALE`
//! (default 1). Each walks back and forth along its own line across the
//! route, cycling its gait phases with the distance walked.
//!
//! With `RNE_WALKER_MODE=ahead` they keep instead to the robot: walker `i`
//! stays `AHEAD_M[i]` in front of it and crosses its view from side to side,
//! as people walking just ahead of a robot do, so they hide much of what the
//! camera sees.

use std::path::{Path, PathBuf};

use rne_math::{Quat, Transform3, Vec3};
use rne_render_3dgs::{load_splat_actor, SplatActorAsset};

/// Lines (RNE world x, z) the walkers pace, crossing the rounded-rectangle route.
const LINES: &[([f64; 2], [f64; 2])] = &[
    ([-3.6, -3.8], [-0.2, -3.8]),
    ([-2.0, -5.6], [-2.0, -2.2]),
    ([-3.4, -2.6], [-0.4, -5.2]),
    ([-0.6, -2.5], [-3.2, -5.0]),
    ([-3.6, -4.4], [-0.2, -4.4]),
    ([-1.4, -5.6], [-1.4, -2.2]),
];
const SPEEDS: &[f64] = &[0.8, 0.6, 0.7, 0.9, 0.75, 0.65];
const STRIDE: f64 = 1.4;
/// `ahead` mode: distance in front of the robot, sideways amplitude and
/// period of each walker's crossing, in metres and seconds.
const AHEAD_M: &[f64] = &[0.8, 1.2, 1.6, 2.0, 1.0, 1.4];
const SWAY_M: &[f64] = &[0.6, 0.8, 1.0, 1.2, 0.7, 0.9];
const SWAY_PERIOD_S: &[f64] = &[5.0, 6.0, 7.0, 8.0, 5.5, 6.5];

pub(crate) struct Walkers {
    pub(crate) phases: Vec<Vec<SplatActorAsset>>,
    pub(crate) scale: f64,
    /// `RNE_WALKER_MODE=ahead`: keep in front of the robot.
    pub(crate) ahead: bool,
    shown: Vec<usize>,
}

impl Walkers {
    /// From the environment, or `None` without `RNE_WALKERS`.
    pub(crate) fn from_env() -> Option<Self> {
        let dir = PathBuf::from(std::env::var_os("RNE_WALKERS")?);
        let count: usize = std::env::var("RNE_WALKER_COUNT")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(4)
            .min(LINES.len());
        let scale: f64 = std::env::var("RNE_WALKER_SCALE")
            .ok()
            .and_then(|v| v.parse().ok())
            .unwrap_or(1.0);
        let phases = (0..count).map(|i| load_phases(&dir, i)).collect::<Vec<_>>();
        println!(
            "walkers: {count} x {} phases, scale {scale}",
            phases.first().map_or(0, Vec::len)
        );
        let ahead = std::env::var("RNE_WALKER_MODE").is_ok_and(|mode| mode == "ahead");
        Some(Self {
            shown: vec![0; count],
            phases,
            scale,
            ahead,
        })
    }

    /// The first phase of each walker, the actors to load.
    pub(crate) fn actors(&self) -> Vec<SplatActorAsset> {
        self.phases.iter().map(|p| p[0].clone()).collect()
    }

    /// Walker `i` at time `t_s`: its pose (RNE world, y up) and gait phase.
    pub(crate) fn state(&self, i: usize, t_s: f64) -> (Transform3, usize) {
        let (a, b) = LINES[i];
        let (dx, dz) = (b[0] - a[0], b[1] - a[1]);
        let length = dx.hypot(dz);
        // Speeds are in metres per second; the scene's unit is `scale` metres.
        let walked = SPEEDS[i] * self.scale * t_s;
        let leg = (walked / length).floor() as i64;
        let along = walked - leg as f64 * length;
        let forward = leg % 2 == 0;
        let s = if forward { along } else { length - along };
        let (x, z) = (a[0] + dx * s / length, a[1] + dz * s / length);
        let (hx, hz) = if forward { (dx, dz) } else { (-dx, -dz) };
        let yaw = (-hz).atan2(hx);
        let z_up_to_y_up = Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2);
        let pose = Transform3 {
            translation: Vec3::new(x, 0.0, z),
            rotation: (Quat::from_rotation_y(yaw) * z_up_to_y_up).normalize(),
            scale: Vec3::splat(self.scale),
        };
        let phases = self.phases[i].len();
        let phase = ((walked / (STRIDE * self.scale)) * phases as f64).floor() as usize % phases;
        (pose, phase)
    }

    /// `ahead` mode: walker `i` at time `t_s` in front of a robot at
    /// (`base_x`, `base_z`) heading `base_yaw` (its forward is the base's +x),
    /// facing the way it crosses; the gait follows the sideways distance.
    pub(crate) fn state_ahead(
        &self,
        i: usize,
        t_s: f64,
        base_x: f64,
        base_z: f64,
        base_yaw: f64,
    ) -> (Transform3, usize) {
        let s = self.scale;
        let (forward, side) = (
            (base_yaw.cos(), -base_yaw.sin()),
            (base_yaw.sin(), base_yaw.cos()),
        );
        let omega = std::f64::consts::TAU / SWAY_PERIOD_S[i];
        let start = 1.3 * i as f64;
        let angle = omega * t_s + start;
        let sway = SWAY_M[i] * s * angle.sin();
        let (x, z) = (
            base_x + forward.0 * AHEAD_M[i] * s + side.0 * sway,
            base_z + forward.1 * AHEAD_M[i] * s + side.1 * sway,
        );
        // Moving towards +side while cos > 0.
        let sign = if angle.cos() >= 0.0 { 1.0 } else { -1.0 };
        let yaw = (-(side.1 * sign)).atan2(side.0 * sign);
        let z_up_to_y_up = Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2);
        let pose = Transform3 {
            translation: Vec3::new(x, 0.0, z),
            rotation: (Quat::from_rotation_y(yaw) * z_up_to_y_up).normalize(),
            scale: Vec3::splat(s),
        };
        // Sideways distance walked: 4 amplitudes per period.
        let periods = (angle - start) / std::f64::consts::TAU;
        let walked = 4.0 * SWAY_M[i] * s * periods;
        let phases = self.phases[i].len();
        let phase = ((walked / (STRIDE * s)) * phases as f64).floor().max(0.0) as usize % phases;
        (pose, phase)
    }

    /// Whether walker `i` must swap to `phase`; records it.
    pub(crate) fn swap(&mut self, i: usize, phase: usize) -> bool {
        let changed = self.shown[i] != phase;
        self.shown[i] = phase;
        changed
    }
}

fn load_phases(dir: &Path, i: usize) -> Vec<SplatActorAsset> {
    (0..)
        .map(|k| dir.join(format!("walker_{i}_phase_{k}.ply")))
        .take_while(|p| p.exists())
        .map(|p| load_splat_actor(&p).expect("walker phase"))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn walkers(count: usize, phases: usize, scale: f64) -> Walkers {
        Walkers {
            phases: vec![vec![SplatActorAsset::new(Vec::new()); phases]; count],
            scale,
            ahead: false,
            shown: vec![0; count],
        }
    }

    #[test]
    fn ahead_walkers_stay_in_front_of_the_robot_and_cross_its_view() {
        let w = walkers(4, 12, 1.0);
        for (yaw, bx, bz) in [(0.0, -2.0, -4.0), (1.0, -1.0, -3.0), (-2.5, 0.5, 0.2)] {
            let (forward, side) = (
                (f64::cos(yaw), -f64::sin(yaw)),
                (f64::sin(yaw), f64::cos(yaw)),
            );
            for i in 0..4 {
                let mut lateral = Vec::new();
                for k in 0..80 {
                    let t = f64::from(k) * 0.1;
                    let p = w.state_ahead(i, t, bx, bz, yaw).0.translation;
                    let (dx, dz) = (p.x - bx, p.z - bz);
                    let ahead = dx * forward.0 + dz * forward.1;
                    assert!((ahead - AHEAD_M[i]).abs() < 1e-9);
                    lateral.push(dx * side.0 + dz * side.1);
                }
                // It crosses: both sides within a period.
                let max = lateral.iter().copied().fold(f64::MIN, f64::max);
                let min = lateral.iter().copied().fold(f64::MAX, f64::min);
                assert!(max > 0.5 * SWAY_M[i] && min < -0.5 * SWAY_M[i]);
            }
        }
    }

    #[test]
    fn a_walker_paces_its_line_and_turns_back_at_the_ends() {
        let w = walkers(1, 12, 1.0);
        let (a, b) = LINES[0];
        let length = (b[0] - a[0]).hypot(b[1] - a[1]);
        let start = w.state(0, 0.0).0.translation;
        assert!((start.x - a[0]).abs() < 1e-9 && (start.z - a[1]).abs() < 1e-9);
        assert!(start.y.abs() < 1e-12);
        // At the far end after length / speed, and back at the start after twice that.
        let end = w.state(0, length / SPEEDS[0]).0.translation;
        assert!((end.x - b[0]).abs() < 1e-9 && (end.z - b[1]).abs() < 1e-9);
        let back = w.state(0, 2.0 * length / SPEEDS[0]).0.translation;
        assert!((back.x - a[0]).abs() < 1e-9 && (back.z - a[1]).abs() < 1e-9);
    }

    #[test]
    fn the_gait_cycles_once_per_stride_and_scales_with_the_walker() {
        let w = walkers(1, 12, 1.0);
        assert_eq!(w.state(0, 0.0).1, 0);
        // Half a stride: half the phases.
        assert_eq!(w.state(0, 0.5 * STRIDE / SPEEDS[0] + 1e-9).1, 6);
        // A walker twice as large walks twice as far, in the same phase.
        let big = walkers(1, 12, 2.0);
        let t = 0.3 * STRIDE / SPEEDS[0];
        assert_eq!(big.state(0, t).1, w.state(0, t).1);
        let (p1, p2) = (w.state(0, t).0.translation, big.state(0, t).0.translation);
        let (a, _) = LINES[0];
        assert!(((p2.x - a[0]) - 2.0 * (p1.x - a[0])).abs() < 1e-9);
    }

    #[test]
    fn a_phase_swap_is_reported_once() {
        let mut w = walkers(2, 12, 1.0);
        assert!(!w.swap(0, 0));
        assert!(w.swap(0, 3));
        assert!(!w.swap(0, 3));
        assert!(!w.swap(1, 0));
    }
}
