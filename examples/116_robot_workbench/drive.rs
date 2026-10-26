//! Fixed-step planar preview and LeKiwi wheel setpoints (RNE Y-up).
use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};

/// Transient body-frame command. A fresh/reset simulation always starts stopped.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct DriveCommand {
    pub vx_m_s: f64,
    pub vz_m_s: f64,
    pub yaw_rad_s: f64,
}
impl DriveCommand {
    pub(crate) fn validate(self) -> Result<()> {
        ensure!(
            self.vx_m_s.is_finite()
                && self.vz_m_s.is_finite()
                && self.vx_m_s.hypot(self.vz_m_s) <= 0.1
                && self.yaw_rad_s.is_finite()
                && self.yaw_rad_s.abs() <= std::f64::consts::FRAC_PI_6,
            "drive requires finite planar speed <= 0.1 m/s and yaw <= pi/6 rad/s"
        );
        Ok(())
    }
    /// Setpoints use the same vendored mount geometry as rne_ai::lekiwi_drive.
    pub(crate) fn wheel_velocities(self) -> [f64; 3] {
        let azimuth: [f64; 3] = [
            1.768_189_872_084_854_2,
            -0.280_980_865_351_562_3,
            -2.347_469_338_125_864_5,
        ];
        let radius = [0.101_984, 0.082_483, 0.082_630];
        std::array::from_fn(|i| {
            (-azimuth[i].sin() * self.vx_m_s
                + azimuth[i].cos() * self.vz_m_s
                + radius[i] * self.yaw_rad_s)
                / 0.0508
        })
    }
}
pub(crate) fn wheel_index(child: &str) -> Option<usize> {
    [
        "omni_wheel_mount-v5-2",
        "omni_wheel_mount-v5-1",
        "omni_wheel_mount-v5",
    ]
    .iter()
    .position(|name| *name == child)
}

/// Exact constant-twist integration; +Y yaw turns +X toward -Z.
pub(crate) fn advance_pose(pose: &mut [f64; 3], command: DriveCommand, dt_s: f64) {
    let angle = command.yaw_rad_s * dt_s;
    let (a, b) = if angle.abs() < 1e-8 {
        (dt_s * (1.0 - angle * angle / 6.0), dt_s * angle * 0.5)
    } else {
        (
            angle.sin() / command.yaw_rad_s,
            (1.0 - angle.cos()) / command.yaw_rad_s,
        )
    };
    let dx = a * command.vx_m_s + b * command.vz_m_s;
    let dz = -b * command.vx_m_s + a * command.vz_m_s;
    let (sin, cos) = pose[2].sin_cos();
    pose[0] += cos * dx + sin * dz;
    pose[1] += -sin * dx + cos * dz;
    pose[2] = (pose[2] + angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU)
        - std::f64::consts::PI;
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn body_frame_translation_and_turn_are_consistent() {
        let command = DriveCommand {
            vx_m_s: 0.1,
            vz_m_s: 0.0,
            yaw_rad_s: 0.5,
        };
        let mut pose = [0.0; 3];
        advance_pose(&mut pose, command, std::f64::consts::PI);
        assert!((pose[0] - 0.2).abs() < 1e-12);
        assert!((pose[1] + 0.2).abs() < 1e-12);
        advance_pose(
            &mut pose,
            DriveCommand {
                vx_m_s: 0.1,
                ..DriveCommand::default()
            },
            1.0,
        );
        assert!((pose[1] + 0.3).abs() < 1e-12);
    }
    #[test]
    fn invalid_commands_are_rejected() {
        for value in [f64::NAN, f64::INFINITY, 0.101, -0.101] {
            assert!(DriveCommand {
                vx_m_s: value,
                ..DriveCommand::default()
            }
            .validate()
            .is_err());
        }
        assert!(DriveCommand {
            vx_m_s: 0.1,
            vz_m_s: 0.1,
            yaw_rad_s: 0.0
        }
        .validate()
        .is_err());
    }
}
