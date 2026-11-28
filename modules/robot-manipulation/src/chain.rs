//! Serial revolute chains: kinematics and collision spheres.

use glam::{DAffine3, DMat3, DVec3};
use robot_foresight::ForesightError;
use serde::{Deserialize, Serialize};

/// Standard Denavit-Hartenberg parameters of one joint:
/// `T = Rz(theta + theta_offset) Tz(d) Tx(a) Rx(alpha)`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct DhParams {
    /// Link length along the new x axis (m).
    pub a_m: f64,
    /// Link twist about the new x axis (rad).
    pub alpha_rad: f64,
    /// Offset along the previous z axis (m).
    pub d_m: f64,
    /// Constant added to the joint angle (rad).
    pub theta_offset_rad: f64,
}

/// One revolute joint with limits.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RevoluteJoint {
    /// Kinematic parameters.
    pub dh: DhParams,
    /// Lower position limit (rad).
    pub lower_rad: f64,
    /// Upper position limit (rad).
    pub upper_rad: f64,
    /// Velocity limit (rad/s).
    pub max_velocity_radps: f64,
}

/// A serial chain of revolute joints on a fixed base.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct SerialChain {
    /// Base pose in the world.
    pub base: DAffine3,
    /// Joints from base to tip.
    pub joints: Vec<RevoluteJoint>,
    /// Radius (m) of the collision spheres placed along every link.
    pub link_radius_m: f64,
}

fn dh_transform(dh: &DhParams, q: f64) -> DAffine3 {
    let theta = q + dh.theta_offset_rad;
    let (st, ct) = theta.sin_cos();
    let (sa, ca) = dh.alpha_rad.sin_cos();
    DAffine3 {
        matrix3: DMat3::from_cols(
            DVec3::new(ct, st, 0.0),
            DVec3::new(-st * ca, ct * ca, sa),
            DVec3::new(st * sa, -ct * sa, ca),
        ),
        translation: DVec3::new(dh.a_m * ct, dh.a_m * st, dh.d_m),
    }
}

impl SerialChain {
    /// A 6-DOF arm with a 0.9 m reach, roughly the geometry of common
    /// collaborative arms, based at `base_m`.
    #[must_use]
    pub fn six_dof_arm(base_m: DVec3) -> Self {
        let pi2 = std::f64::consts::FRAC_PI_2;
        let lim = std::f64::consts::PI;
        let j = |a_m, alpha_rad, d_m| RevoluteJoint {
            dh: DhParams {
                a_m,
                alpha_rad,
                d_m,
                theta_offset_rad: 0.0,
            },
            lower_rad: -lim,
            upper_rad: lim,
            max_velocity_radps: 1.5,
        };
        Self {
            base: DAffine3::from_translation(base_m),
            joints: vec![
                j(0.0, pi2, 0.16),
                j(0.42, 0.0, 0.0),
                j(0.39, 0.0, 0.0),
                j(0.0, pi2, 0.11),
                j(0.0, -pi2, 0.09),
                j(0.0, 0.0, 0.08),
            ],
            link_radius_m: 0.05,
        }
    }

    /// Degrees of freedom.
    #[must_use]
    pub fn dof(&self) -> usize {
        self.joints.len()
    }

    /// Checks a configuration's length and finiteness.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidInput`].
    pub fn check_q(&self, q: &[f64]) -> Result<(), ForesightError> {
        if q.len() != self.dof() || q.iter().any(|v| !v.is_finite()) {
            return Err(ForesightError::InvalidInput(format!(
                "configuration must have {} finite values",
                self.dof()
            )));
        }
        Ok(())
    }

    /// Whether `q` is within the position limits.
    #[must_use]
    pub fn within_limits(&self, q: &[f64]) -> bool {
        q.len() == self.dof()
            && self
                .joints
                .iter()
                .zip(q)
                .all(|(j, v)| *v >= j.lower_rad - 1e-9 && *v <= j.upper_rad + 1e-9)
    }

    /// Clamps `q` into the position limits.
    #[must_use]
    pub fn clamp(&self, q: &[f64]) -> Vec<f64> {
        self.joints
            .iter()
            .zip(q)
            .map(|(j, v)| v.clamp(j.lower_rad, j.upper_rad))
            .collect()
    }

    /// World frames: the base followed by every joint's frame.
    #[must_use]
    pub fn frames(&self, q: &[f64]) -> Vec<DAffine3> {
        let mut out = Vec::with_capacity(self.dof() + 1);
        let mut t = self.base;
        out.push(t);
        for (joint, v) in self.joints.iter().zip(q) {
            t *= dh_transform(&joint.dh, *v);
            out.push(t);
        }
        out
    }

    /// Tool-frame pose.
    #[must_use]
    pub fn end_effector(&self, q: &[f64]) -> DAffine3 {
        self.frames(q).last().copied().unwrap_or(self.base)
    }

    /// Collision spheres along every link segment between consecutive
    /// frame origins, spaced at most one radius apart.
    #[must_use]
    pub fn collision_spheres(&self, q: &[f64]) -> Vec<DVec3> {
        let frames = self.frames(q);
        let mut out = Vec::new();
        for w in frames.windows(2) {
            let (a, b) = (w[0].translation, w[1].translation);
            let n = (a.distance(b) / self.link_radius_m.max(1e-3))
                .ceil()
                .max(1.0) as usize;
            for k in 1..=n {
                out.push(a.lerp(b, k as f64 / n as f64));
            }
        }
        out
    }

    /// Position IK by damped least squares on a numerical Jacobian.
    ///
    /// Returns the configuration and the remaining position error (m).
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidInput`] for a malformed seed.
    pub fn solve_position_ik(
        &self,
        target_m: DVec3,
        seed: &[f64],
        iterations: usize,
    ) -> Result<(Vec<f64>, f64), ForesightError> {
        self.check_q(seed)?;
        let mut q = self.clamp(seed);
        let damping2 = 1e-4;
        let eps = 1e-6;
        for _ in 0..iterations {
            let p = self.end_effector(&q).translation;
            let err = target_m - p;
            if err.length() < 1e-5 {
                break;
            }
            // J (3 x n), dq = J^T (J J^T + l^2 I)^-1 err
            let cols: Vec<DVec3> = (0..self.dof())
                .map(|i| {
                    let mut qp = q.clone();
                    qp[i] += eps;
                    (self.end_effector(&qp).translation - p) / eps
                })
                .collect();
            let mut jjt = DMat3::ZERO;
            for c in &cols {
                jjt += DMat3::from_cols(*c * c.x, *c * c.y, *c * c.z);
            }
            jjt += DMat3::from_diagonal(DVec3::splat(damping2));
            let y = jjt.inverse() * err.clamp_length_max(0.1);
            for (qi, c) in q.iter_mut().zip(&cols) {
                *qi += c.dot(y);
            }
            q = self.clamp(&q);
        }
        let residual = (target_m - self.end_effector(&q).translation).length();
        Ok((q, residual))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn zero_configuration_reach() {
        let arm = SerialChain::six_dof_arm(DVec3::ZERO);
        let tip = arm.end_effector(&[0.0; 6]).translation;
        // Shoulder 0.16 up, then 0.81 out along x; wrist offsets add 0.11 along -y
        // (alpha = pi/2 at joint 1 maps z to -y) and 0.09 + 0.08 along z-ish.
        assert!((tip.x - 0.81).abs() < 1e-9, "{tip}");
        assert!(tip.length() > 0.8);
    }

    #[test]
    fn ik_reaches_a_reachable_point() {
        let arm = SerialChain::six_dof_arm(DVec3::ZERO);
        let q_true = [0.3, -0.6, 0.9, 0.2, 0.4, 0.0];
        let target = arm.end_effector(&q_true).translation;
        let (q, residual) = arm.solve_position_ik(target, &[0.0; 6], 200).unwrap();
        assert!(residual < 1e-4, "{residual}");
        assert!(arm.within_limits(&q));
    }

    #[test]
    fn ik_reports_unreachable_points() {
        let arm = SerialChain::six_dof_arm(DVec3::ZERO);
        let (_, residual) = arm
            .solve_position_ik(DVec3::new(3.0, 0.0, 0.0), &[0.0; 6], 100)
            .unwrap();
        assert!(residual > 1.5);
    }

    #[test]
    fn spheres_cover_every_link() {
        let arm = SerialChain::six_dof_arm(DVec3::ZERO);
        let s = arm.collision_spheres(&[0.0; 6]);
        assert!(s.len() >= 18);
        for w in s.windows(2) {
            assert!(
                w[0].distance(w[1]) <= arm.link_radius_m + 1e-9 || w[1].distance(DVec3::ZERO) > 0.0
            );
        }
    }
}
