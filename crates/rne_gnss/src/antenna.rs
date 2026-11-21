//! Multi-antenna rigs.
//!
//! A robot can carry several GNSS antennas (for example two along its
//! length for moving-baseline heading). Each antenna has a lever arm in the
//! body frame (x forward, y left, z up) and its own [`GnssReceiver`], with its
//! own clock and noise seed, as with separate receiver boards. The rig places
//! every antenna in the ENU frame from the body pose.

use crate::geodesy::EnuFrame;
use crate::orbit::Constellation;
use crate::receiver::{GnssFix, GnssReceiver, ReceiverConfig, ReceiverError};
use crate::scene::GnssScene;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// A planar body pose with height, in the ENU frame.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct BodyPose {
    /// East in meters.
    pub x_m: f64,
    /// North in meters.
    pub y_m: f64,
    /// Height of the body origin in meters.
    pub z_m: f64,
    /// Heading from east, counter-clockwise, in radians.
    pub yaw_rad: f64,
}

impl BodyPose {
    /// Transforms a body-frame lever arm to an ENU point.
    pub fn transform(&self, lever_arm_m: Vec3) -> Vec3 {
        let (sin, cos) = self.yaw_rad.sin_cos();
        Vec3::new(
            self.x_m + cos * lever_arm_m.x - sin * lever_arm_m.y,
            self.y_m + sin * lever_arm_m.x + cos * lever_arm_m.y,
            self.z_m + lever_arm_m.z,
        )
    }
}

/// One antenna of a rig.
#[derive(Clone, Debug)]
pub struct RigAntenna {
    /// Lever arm in the body frame in meters.
    pub lever_arm_m: Vec3,
    /// The antenna's receiver.
    pub receiver: GnssReceiver,
}

/// Several antennas rigidly mounted on one body.
#[derive(Clone, Debug)]
pub struct AntennaRig {
    antennas: Vec<RigAntenna>,
}

impl AntennaRig {
    /// Creates a rig; antenna `i` uses `config` with seed `config.seed + i`.
    pub fn new(lever_arms_m: &[Vec3], config: ReceiverConfig) -> Result<Self, ReceiverError> {
        let antennas = lever_arms_m
            .iter()
            .enumerate()
            .map(|(i, &lever_arm_m)| {
                let receiver = GnssReceiver::new(ReceiverConfig {
                    seed: config.seed.wrapping_add(i as u64 * 0x1000_0001),
                    ..config
                })?;
                Ok(RigAntenna {
                    lever_arm_m,
                    receiver,
                })
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(Self { antennas })
    }

    /// The antennas in mounting order.
    pub fn antennas(&self) -> &[RigAntenna] {
        &self.antennas
    }

    /// Mutable access to the antennas.
    pub fn antennas_mut(&mut self) -> &mut [RigAntenna] {
        &mut self.antennas
    }

    /// ENU positions of every antenna for a body pose.
    pub fn antenna_positions(&self, pose: BodyPose) -> Vec<Vec3> {
        self.antennas
            .iter()
            .map(|a| pose.transform(a.lever_arm_m))
            .collect()
    }

    /// Steps every antenna's receiver to `time_s`; returns the released fixes
    /// per antenna.
    pub fn step(
        &mut self,
        scene: &GnssScene,
        constellation: &Constellation,
        frame: &EnuFrame,
        pose: BodyPose,
        time_s: f64,
    ) -> Vec<Vec<GnssFix>> {
        self.antennas
            .iter_mut()
            .map(|antenna| {
                let position = pose.transform(antenna.lever_arm_m);
                antenna
                    .receiver
                    .step(scene, constellation, frame, position, time_s)
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geodesy::Geodetic;
    use approx::assert_relative_eq;

    #[test]
    fn lever_arms_rotate_with_the_body() {
        let pose = BodyPose {
            x_m: 10.0,
            y_m: 5.0,
            z_m: 0.5,
            yaw_rad: std::f64::consts::FRAC_PI_2,
        };
        let p = pose.transform(Vec3::new(1.0, 0.0, 0.3));
        assert_relative_eq!(p.x, 10.0, epsilon = 1e-12);
        assert_relative_eq!(p.y, 6.0, epsilon = 1e-12);
        assert_relative_eq!(p.z, 0.8, epsilon = 1e-12);
    }

    #[test]
    fn antennas_get_independent_noise() {
        let mut rig = AntennaRig::new(
            &[Vec3::new(0.5, 0.0, 0.3), Vec3::new(-0.5, 0.0, 0.3)],
            ReceiverConfig::default(),
        )
        .unwrap();
        let frame = EnuFrame::new(Geodetic::from_degrees(36.0, 140.0, 20.0));
        let pose = BodyPose {
            x_m: 0.0,
            y_m: 0.0,
            z_m: 0.5,
            yaw_rad: 0.0,
        };
        let fixes = rig.step(
            &GnssScene::open_sky(),
            &Constellation::gps_galileo_nominal(),
            &frame,
            pose,
            0.0,
        );
        assert_eq!(fixes.len(), 2);
        let positions = rig.antenna_positions(pose);
        assert_relative_eq!((positions[0] - positions[1]).length(), 1.0, epsilon = 1e-12);
        let a = rig.antennas()[0].receiver.last_epoch().unwrap();
        let b = rig.antennas()[1].receiver.last_epoch().unwrap();
        let tracked = a
            .observations
            .iter()
            .position(|o| o.path().is_tracked())
            .unwrap();
        let (pa, pb) = (
            a.observations[tracked]
                .primary()
                .unwrap()
                .pseudorange_m
                .unwrap(),
            b.observations[tracked]
                .primary()
                .unwrap()
                .pseudorange_m
                .unwrap(),
        );
        // Different clocks and noise, but the same satellite ~1 m apart.
        assert!(pa != pb);
    }
}
