//! A diff-drive robot's wheel odometry, simulated from its true planar motion.
//!
//! Each step's forward distance and turn, taken from the true base pose, are
//! scaled by `1 + SCALE_ERROR` and `1 + TURN_ERROR` (a wheel radius and a
//! track width slightly off, as calibrated wheels are) plus white noise from a
//! fixed-seed generator, and integrated. The result is in an odometry frame of
//! its own, z up (x forward, y left, as ROS `odom`), starting at the identity:
//! people walking past do not move it, unlike a camera-based odometry.

/// Distance error of the wheels' radius (2 %).
const SCALE_ERROR: f64 = 0.02;
/// Turn error of the track width (3 %).
const TURN_ERROR: f64 = 0.03;
/// Per-step white noise on distance (m) and turn (rad).
const NOISE_M: f64 = 0.0005;
const NOISE_RAD: f64 = 0.0005;

#[derive(Debug, Clone)]
pub(crate) struct WheelOdometry {
    x_m: f64,
    y_m: f64,
    heading_rad: f64,
    previous: Option<(f64, f64, f64)>,
    rng: u64,
}

impl WheelOdometry {
    pub(crate) fn new(seed: u64) -> Self {
        Self {
            x_m: 0.0,
            y_m: 0.0,
            heading_rad: 0.0,
            previous: None,
            rng: seed | 1,
        }
    }

    /// A uniform value in [-1, 1) (xorshift64*).
    fn noise(&mut self) -> f64 {
        self.rng ^= self.rng >> 12;
        self.rng ^= self.rng << 25;
        self.rng ^= self.rng >> 27;
        let value = self.rng.wrapping_mul(0x2545_f491_4f6c_dd1d) >> 11;
        value as f64 / (1u64 << 52) as f64 - 1.0
    }

    /// Advances to the true base pose (RNE world: x, z on the floor, yaw about
    /// +y) and returns the odometry pose (x, y, heading) in its own z-up frame.
    pub(crate) fn step(&mut self, x_m: f64, z_m: f64, yaw_rad: f64) -> (f64, f64, f64) {
        if let Some((px, pz, pyaw)) = self.previous {
            // Forward is the base's +x: (cos yaw, -sin yaw) in RNE's (x, z).
            let (dx, dz) = (x_m - px, z_m - pz);
            let forward = dx * pyaw.cos() - dz * pyaw.sin();
            let mut turn = yaw_rad - pyaw;
            while turn > std::f64::consts::PI {
                turn -= std::f64::consts::TAU;
            }
            while turn < -std::f64::consts::PI {
                turn += std::f64::consts::TAU;
            }
            let moving = forward.abs() > 1e-9 || turn.abs() > 1e-9;
            let (noise_m, noise_rad) = if moving {
                (NOISE_M * self.noise(), NOISE_RAD * self.noise())
            } else {
                (0.0, 0.0)
            };
            let distance = forward * (1.0 + SCALE_ERROR) + noise_m;
            let turn = turn * (1.0 + TURN_ERROR) + noise_rad;
            let mid = self.heading_rad + 0.5 * turn;
            self.x_m += distance * mid.cos();
            self.y_m += distance * mid.sin();
            self.heading_rad += turn;
        }
        self.previous = Some((x_m, z_m, yaw_rad));
        (self.x_m, self.y_m, self.heading_rad)
    }
}

/// One TUM line (seconds; quaternion about z) of an odometry pose.
pub(crate) fn tum_line(timestamp_ns: u64, pose: (f64, f64, f64)) -> String {
    let (x, y, heading) = pose;
    format!(
        "{:.9} {x:.6} {y:.6} 0 0 0 {:.9} {:.9}\n",
        timestamp_ns as f64 * 1e-9,
        (0.5 * heading).sin(),
        (0.5 * heading).cos()
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_straight_drive_reads_two_percent_long_and_a_turn_three_percent_more() {
        let mut odometry = WheelOdometry::new(7);
        // 1 m forward along RNE +x at yaw 0, in 100 steps.
        let mut pose = (0.0, 0.0, 0.0);
        for k in 0..=100 {
            pose = odometry.step(f64::from(k) * 0.01, 0.0, 0.0);
        }
        assert!((pose.0 - 1.02).abs() < 0.01, "{pose:?}");
        assert!(pose.1.abs() < 0.01 && pose.2.abs() < 0.01);
        // Then a quarter turn in place, in 100 steps.
        let start = pose.2;
        for k in 1..=100 {
            pose = odometry.step(1.0, 0.0, f64::from(k) * std::f64::consts::FRAC_PI_2 / 100.0);
        }
        let turned = pose.2 - start;
        assert!(
            (turned - 1.03 * std::f64::consts::FRAC_PI_2).abs() < 0.01,
            "{turned}"
        );
    }

    #[test]
    fn standing_still_adds_no_drift_and_runs_repeat() {
        let mut still = WheelOdometry::new(3);
        for _ in 0..50 {
            assert_eq!(still.step(0.5, -2.0, 0.3), (0.0, 0.0, 0.0));
        }
        let mut a = WheelOdometry::new(3);
        let mut b = WheelOdometry::new(3);
        let path = |o: &mut WheelOdometry| {
            (0..30)
                .map(|k| o.step(f64::from(k) * 0.02, 0.0, f64::from(k) * 0.01))
                .last()
                .expect("steps")
        };
        assert_eq!(path(&mut a), path(&mut b));
    }

    #[test]
    fn a_left_turn_reads_as_positive_heading() {
        // RNE yaw about +y turning forward (+x) towards -z is a left turn
        // seen from above (+y): positive about z in the z-up frame.
        let mut odometry = WheelOdometry::new(1);
        odometry.step(0.0, 0.0, 0.0);
        let pose = odometry.step(0.1, -0.1, 0.5);
        assert!(pose.2 > 0.0);
        assert!(tum_line(1_000_000_000, pose).starts_with("1.000000000 "));
    }
}
