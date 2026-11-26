//! Walkers crossing the robot's route, as splat actors.
//!
//! `RNE_WALKERS=DIR` loads `DIR/walker_<i>_phase_<k>.ply` (visloc-rs
//! `ros2/visloc-ros2/gs/walker_splat_plys.py`: x forward, z up, feet at z = 0)
//! for `RNE_WALKER_COUNT` walkers (default 4), scaled by `RNE_WALKER_SCALE`
//! (default 1). Each walks back and forth along its own line across the
//! route, cycling its gait phases with the distance walked.

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

pub(crate) struct Walkers {
    pub(crate) phases: Vec<Vec<SplatActorAsset>>,
    pub(crate) scale: f64,
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
        Some(Self {
            shown: vec![0; count],
            phases,
            scale,
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
            shown: vec![0; count],
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
