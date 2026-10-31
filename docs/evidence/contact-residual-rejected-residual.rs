//! Certify the current coupled contact law, not merely a small iterate change.
use super::{ContactSolverConfig, DenseMatrix, CONE_TOLERANCE};

pub(super) fn satisfied(
    matrix: &DenseMatrix,
    free: &[f64],
    friction: &[f64],
    impulses: &[[f64; 3]],
    config: &ContactSolverConfig,
) -> bool {
    let vt = config.velocity_tolerance_m_s;
    let it = config.tolerance_n_s;
    if !vt.is_finite() || vt < 0.0 || !it.is_finite() || it < 0.0 {
        return false;
    }
    impulses.iter().enumerate().all(|(i, impulse)| {
        if !impulse.iter().all(|x| x.is_finite()) {
            return false;
        }
        let mut velocity = [0.0; 3];
        for (axis, value) in velocity.iter_mut().enumerate() {
            let row = 3 * i + axis;
            *value = free[row]
                + impulses
                    .iter()
                    .enumerate()
                    .map(|(j, p)| {
                        (0..3)
                            .map(|k| matrix.get(row, 3 * j + k) * p[k])
                            .sum::<f64>()
                    })
                    .sum::<f64>();
        }
        let start = 3 * i;
        let softness = config.regularization
            * (0..3)
                .map(|k| matrix.get(start + k, start + k))
                .sum::<f64>()
            / 3.0;
        velocity[2] += softness * impulse[2];
        point_satisfied(*impulse, velocity, friction[i], it, vt)
    })
}

fn point_satisfied(p: [f64; 3], v: [f64; 3], mu: f64, it: f64, vt: f64) -> bool {
    if !v.iter().all(|x| x.is_finite()) || p[2] < -it || v[2] < -vt {
        return false;
    }
    if p[2] > it && v[2].abs() > vt {
        return false;
    }
    let tangent = p[0].hypot(p[1]);
    let limit = mu * p[2].max(0.0);
    let slack = it.max(CONE_TOLERANCE * limit);
    if !limit.is_finite() || tangent > limit + slack {
        return false;
    }
    if mu == 0.0 || limit <= it {
        return true;
    }
    let slip = v[0].hypot(v[1]);
    let speed_tolerance = std::f64::consts::SQRT_2 * vt;
    if slip <= speed_tolerance {
        return true;
    }
    if tangent < limit - slack || tangent == 0.0 {
        return false;
    }
    // Sliding must oppose the current coupled velocity, not the velocity
    // observed when this point was visited earlier in the sweep.
    (v[0] + slip * (p[0] / tangent)).hypot(v[1] + slip * (p[1] / tangent)) <= speed_tolerance
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contact::solve_contact_impulses;

    #[test]
    fn small_relaxed_update_cannot_certify_a_closing_contact() {
        let mut g = DenseMatrix::zeros(3, 3);
        for i in 0..3 {
            g.set(i, i, 1.0);
        }
        let config = ContactSolverConfig {
            relaxation: 1e-6,
            tolerance_n_s: 1e-5,
            max_iterations: 3,
            ..Default::default()
        };
        let result = solve_contact_impulses(&g, &[0.0, 0.0, -1.0], &[0.5], &config, None).unwrap();
        assert!(result.last_change_n_s < config.tolerance_n_s);
        assert!(!result.converged);
        assert_eq!(result.iterations, 3);
        assert!(result.impulses_n_s[0][2] < 1e-5);
    }

    #[test]
    fn coupled_velocity_must_satisfy_normal_and_friction_laws() {
        let check = |p, v| point_satisfied(p, v, 0.5, 1e-10, 1e-8);
        assert!(check([0.0; 3], [1.0, 0.0, 1.0]));
        assert!(check([0.0, 0.0, 1.0], [0.0; 3]));
        assert!(check([-0.5, 0.0, 1.0], [1.0, 0.0, 0.0]));
        assert!(!check([0.5, 0.0, 1.0], [1.0, 0.0, 0.0]));
        assert!(!check([0.0, 0.0, 1.0], [1.0, 0.0, 0.0]));
        assert!(!check([0.0, 0.0, 1.0], [0.0, 0.0, 0.01]));
        assert!(!check([0.0; 3], [0.0, 0.0, -0.01]));
        assert!(!check([1.0, 0.0, 1.0], [0.0; 3]));
        assert!(!check([0.0, 0.0, 1.0], [f64::NAN, 0.0, 0.0]));
    }

    #[test]
    fn residual_includes_normal_compliance_and_rejects_nonfinite_data() {
        let mut g = DenseMatrix::zeros(3, 3);
        for i in 0..3 {
            g.set(i, i, 1.0);
        }
        let config = ContactSolverConfig {
            regularization: 0.1,
            ..Default::default()
        };
        assert!(satisfied(
            &g,
            &[0.0, 0.0, -1.1],
            &[0.5],
            &[[0.0, 0.0, 1.0]],
            &config
        ));
        assert!(!satisfied(
            &g,
            &[0.0, 0.0, -1.1],
            &[0.5],
            &[[0.0, 0.0, 1.0]],
            &ContactSolverConfig::default()
        ));
        assert!(!satisfied(
            &g,
            &[0.0; 3],
            &[0.5],
            &[[0.0, 0.0, f64::INFINITY]],
            &config
        ));
    }
}
