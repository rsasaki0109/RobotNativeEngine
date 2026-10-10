//! A finite, explicitly integrated impact; the controller cannot see this schedule.

use rne_math::Vec3;

/// A sampled half-sine force pulse normalized to a declared linear impulse.
#[derive(Debug, Clone)]
pub(super) struct Impact {
    start_step: usize,
    forces_world_n: Vec<Vec3>,
    dt_s: f64,
}

impl Impact {
    pub(super) fn new(
        start_step: usize,
        duration_steps: usize,
        dt_s: f64,
        impulse_ns: f64,
        direction_world: Vec3,
    ) -> Self {
        assert!(duration_steps > 0 && dt_s.is_finite() && dt_s > 0.0);
        assert!(impulse_ns.is_finite() && impulse_ns >= 0.0);
        assert!(direction_world.is_finite() && direction_world.length() > 0.0);
        let direction = direction_world.normalize();
        let weights: Vec<_> = (0..duration_steps)
            .map(|step| (std::f64::consts::PI * (step as f64 + 0.5) / duration_steps as f64).sin())
            .collect();
        let normalization = weights.iter().sum::<f64>() * dt_s;
        let forces_world_n = weights
            .into_iter()
            .map(|weight| direction * (impulse_ns * weight / normalization))
            .collect();
        Self {
            start_step,
            forces_world_n,
            dt_s,
        }
    }

    pub(super) fn force_world_n(&self, step: usize) -> Vec3 {
        step.checked_sub(self.start_step)
            .and_then(|index| self.forces_world_n.get(index))
            .copied()
            .unwrap_or(Vec3::ZERO)
    }

    pub(super) fn integrated_impulse_ns(&self) -> Vec3 {
        self.forces_world_n
            .iter()
            .fold(Vec3::ZERO, |sum, force| sum + *force * self.dt_s)
    }

    pub(super) fn peak_force_n(&self) -> f64 {
        self.forces_world_n
            .iter()
            .map(|force| force.length())
            .fold(0.0, f64::max)
    }

    pub(super) fn duration_s(&self) -> f64 {
        self.forces_world_n.len() as f64 * self.dt_s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn changing_physics_rate_preserves_impulse_and_duration() {
        for (steps, dt_s) in [(40, 0.002), (80, 0.001), (160, 0.0005)] {
            let impact = Impact::new(1500, steps, dt_s, 24.0, Vec3::Z);
            assert!((impact.integrated_impulse_ns() - Vec3::Z * 24.0).length() < 1e-12);
            assert_eq!(impact.duration_s(), 0.08);
            assert!(impact.peak_force_n() > 470.0 && impact.peak_force_n() < 472.0);
            assert_eq!(impact.force_world_n(1499), Vec3::ZERO);
            assert_eq!(impact.force_world_n(1500 + steps), Vec3::ZERO);
            assert!(impact.force_world_n(1500).z > 0.0);
        }
    }

    #[test]
    fn reversed_and_oblique_impacts_keep_the_declared_momentum() {
        for direction in [-Vec3::Z, Vec3::X, Vec3::new(1.0, 0.0, 1.0)] {
            let impact = Impact::new(123, 80, 0.001, 24.0, direction);
            assert!(
                (impact.integrated_impulse_ns() - direction.normalize() * 24.0).length() < 1e-12
            );
            assert_eq!(impact.force_world_n(122), Vec3::ZERO);
        }
    }
}
