//! Bounded full sticking candidate with no compliance added to the equations.
use super::{
    residual, ContactSolution, ContactSolverConfig, ContactState, DenseMatrix, CONE_TOLERANCE,
};

pub(super) fn attempt(
    g: &DenseMatrix,
    free: &[f64],
    mu: &[f64],
    config: &ContactSolverConfig,
    seed: &[[f64; 3]],
) -> (Option<ContactSolution>, usize) {
    let n = mu.len();
    let tol = config.velocity_tolerance_m_s;
    if !(2..=64).contains(&n)
        || config.max_iterations == 0
        || config.regularization != 0.0
        || config.relaxation != 1.0
        || !tol.is_finite()
        || tol <= 0.0
        || !config.tolerance_n_s.is_finite()
        || config.tolerance_n_s < 0.0
        || mu.iter().any(|m| *m <= 0.0)
    {
        return (None, 0);
    }
    let mut active: Vec<bool> = (0..n)
        .map(|i| free[3 * i + 2] < 0.0 || seed[i][2] > 0.0)
        .collect();
    let budget = config.max_iterations.min(8);
    for pass in 1..=budget {
        let rows: Vec<usize> = (0..3 * n).filter(|r| active[r / 3]).collect();
        let Some(x) = cg(g, free, &rows, tol * 0.25) else {
            return (None, pass);
        };
        let mut impulse = vec![[0.0; 3]; n];
        for (&r, v) in rows.iter().zip(x) {
            impulse[r / 3][r % 3] = v;
        }
        if let Some(i) = (0..n)
            .filter(|i| impulse[*i][2] < 0.0)
            .min_by(|a, b| impulse[*a][2].total_cmp(&impulse[*b][2]))
        {
            active[i] = false;
            continue;
        }
        let mut added = false;
        for i in 0..n {
            if !active[i] {
                let v = free[3 * i + 2]
                    + (0..3 * n)
                        .map(|r| g.get(3 * i + 2, r) * impulse[r / 3][r % 3])
                        .sum::<f64>();
                if !v.is_finite() {
                    return (None, pass);
                }
                if v < -tol {
                    active[i] = true;
                    added = true;
                }
            }
        }
        if added {
            continue;
        }
        // A solution requiring sliding is not a sticking candidate. Reject
        // without changing the caller's seed or projecting an invalid solution.
        if impulse.iter().zip(mu).any(|(p, m)| {
            !p.iter().all(|v| v.is_finite()) || p[0].hypot(p[1]) > m * p[2] * (1.0 + CONE_TOLERANCE)
        }) || !residual::satisfied(g, free, mu, &impulse, config)
        {
            return (None, pass);
        }
        let last_change = impulse
            .iter()
            .zip(seed)
            .map(|(p, s)| (p[0] - s[0]).hypot(p[1] - s[1]).hypot(p[2] - s[2]))
            .fold(0.0, f64::max);
        let states = impulse
            .iter()
            .map(|p| {
                if p[2] > 0.0 {
                    ContactState::Sticking
                } else {
                    ContactState::Open
                }
            })
            .collect();
        return (
            Some(ContactSolution {
                impulses_n_s: impulse,
                states,
                iterations: 0,
                normal_block_iterations: 0,
                normal_block_solved: false,
                sticking_block_iterations: pass,
                sticking_block_solved: true,
                converged: true,
                contact_law_satisfied: true,
                last_change_n_s: last_change,
            }),
            pass,
        );
    }
    (None, budget)
}

/// Zero-start CG gives a range-space candidate for consistent singular PSD
/// blocks. Breakdown or insufficient accuracy falls back; no diagonal shift.
fn cg(g: &DenseMatrix, free: &[f64], rows: &[usize], tolerance: f64) -> Option<Vec<f64>> {
    let n = rows.len();
    let mut x = vec![0.0; n];
    let mut r: Vec<f64> = rows.iter().map(|i| -free[*i]).collect();
    let mut direction = r.clone();
    let mut rr = r.iter().map(|v| v * v).sum::<f64>();
    for _ in 0..=2 * n {
        if !rr.is_finite() {
            return None;
        }
        if r.iter().all(|v| v.abs() <= tolerance) {
            return Some(x);
        }
        let product: Vec<f64> = rows
            .iter()
            .map(|i| {
                rows.iter()
                    .zip(&direction)
                    .map(|(j, p)| g.get(*i, *j) * p)
                    .sum()
            })
            .collect();
        let curvature = direction
            .iter()
            .zip(&product)
            .map(|(p, a)| p * a)
            .sum::<f64>();
        if !curvature.is_finite() || curvature <= 0.0 {
            return None;
        }
        let alpha = rr / curvature;
        for i in 0..n {
            x[i] += alpha * direction[i];
            r[i] -= alpha * product[i];
        }
        let next = r.iter().map(|v| v * v).sum::<f64>();
        let beta = next / rr;
        for i in 0..n {
            direction[i] = r[i] + beta * direction[i];
        }
        rr = next;
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::contact::solve_contact_impulses;

    fn config() -> ContactSolverConfig {
        ContactSolverConfig {
            experimental_sticking_block: true,
            ..Default::default()
        }
    }

    fn identity() -> DenseMatrix {
        let mut g = DenseMatrix::zeros(6, 6);
        for i in 0..6 {
            g.set(i, i, 1.0);
        }
        g
    }

    fn check(g: &DenseMatrix, b: &[f64], result: &ContactSolution) {
        let x: Vec<_> = result.impulses_n_s.iter().flatten().copied().collect();
        let v = g.mul_vec(&x);
        for (i, p) in result.impulses_n_s.iter().enumerate() {
            assert!(p[2] >= 0.0);
            assert!(p[0].hypot(p[1]) <= 0.5 * p[2] * (1.0 + CONE_TOLERANCE));
            assert!(v[3 * i + 2] + b[3 * i + 2] >= -1e-8);
            if p[2] > 0.0 {
                for axis in 0..3 {
                    assert!((v[3 * i + axis] + b[3 * i + axis]).abs() <= 1e-8);
                }
            }
        }
        assert!(result.sticking_block_solved && result.contact_law_satisfied);
        assert_eq!(result.iterations, 0);
    }

    #[test]
    fn sticking_candidate_respects_the_64_contact_bound() {
        for count in [2, 16, 32, 64, 65] {
            let mut g = DenseMatrix::zeros(3 * count, 3 * count);
            for i in 0..3 * count {
                g.set(i, i, 1.0);
            }
            let b: Vec<_> = (0..count).flat_map(|_| [0.1, 0.0, -1.0]).collect();
            let result =
                solve_contact_impulses(&g, &b, &vec![0.5; count], &config(), None).unwrap();
            assert_eq!(result.sticking_block_solved, count <= 64);
            assert!(result.contact_law_satisfied);
            if count <= 64 {
                check(&g, &b, &result);
            } else {
                assert_eq!(result.sticking_block_iterations, 0);
            }
        }
    }

    #[test]
    fn induced_tangent_velocity_is_cancelled_by_valid_sticking_impulses() {
        let mut g = identity();
        g.set(0, 2, 0.25);
        g.set(2, 0, 0.25);
        let b = [0.0, 0.0, -1.0, 0.0, 0.0, -1.0];
        let solve = || solve_contact_impulses(&g, &b, &[0.5; 2], &config(), None).unwrap();
        let result = solve();
        check(&g, &b, &result);
        assert!(result.impulses_n_s[0][0] < -0.2);
        assert_eq!(result, solve());
    }

    #[test]
    fn redundant_sticking_rows_use_a_valid_range_space_solution() {
        let mut g = DenseMatrix::zeros(6, 6);
        for i in 0..6 {
            for j in 0..6 {
                if i % 3 == j % 3 {
                    g.set(i, j, 1.0);
                }
            }
        }
        let b = [0.2, 0.0, -1.0, 0.2, 0.0, -1.0];
        let result = solve_contact_impulses(&g, &b, &[0.5; 2], &config(), None).unwrap();
        check(&g, &b, &result);
        for p in result.impulses_n_s {
            assert!((p[0] + 0.1).abs() < 1e-12);
            assert!((p[2] - 0.5).abs() < 1e-12);
        }
        let inconsistent = [0.2, 0.0, -1.0, 0.3, 0.0, -1.0];
        assert!(
            attempt(&g, &inconsistent, &[0.5; 2], &config(), &[[0.0; 3]; 2])
                .0
                .is_none()
        );
    }

    #[test]
    fn active_set_adds_and_removes_normal_constraints() {
        for (coupling, first, expected) in [
            (-0.5, 0.1, ContactState::Sticking),
            (0.9, -0.1, ContactState::Open),
        ] {
            let mut g = identity();
            g.set(2, 5, coupling);
            g.set(5, 2, coupling);
            let b = [0.1, 0.0, first, 0.1, 0.0, -1.0];
            let result = solve_contact_impulses(&g, &b, &[0.5; 2], &config(), None).unwrap();
            check(&g, &b, &result);
            assert_eq!(result.states[0], expected);
            assert!(result.sticking_block_iterations >= 2);
        }
    }

    #[test]
    fn sliding_candidate_rejection_preserves_the_gs_result() {
        let g = identity();
        let b = [0.5, 0.0, -0.2, 0.0, 0.0, -0.2];
        let seed = [[0.02, 0.0, 0.1], [0.0, 0.0, 0.1]];
        let before = solve_contact_impulses(
            &g,
            &b,
            &[0.5; 2],
            &ContactSolverConfig::default(),
            Some(&seed),
        )
        .unwrap();
        let after = solve_contact_impulses(&g, &b, &[0.5; 2], &config(), Some(&seed)).unwrap();
        assert!(!after.sticking_block_solved && after.sticking_block_iterations > 0);
        assert_eq!(before.impulses_n_s, after.impulses_n_s);
        assert_eq!(before.states, after.states);
        assert_eq!(before.iterations, after.iterations);
        assert_eq!(before.contact_law_satisfied, after.contact_law_satisfied);
        assert!(!ContactSolverConfig::default().experimental_sticking_block);
        for disabled in [
            ContactSolverConfig {
                regularization: 1e-3,
                ..config()
            },
            ContactSolverConfig {
                relaxation: 0.5,
                ..config()
            },
            ContactSolverConfig {
                max_iterations: 0,
                ..config()
            },
        ] {
            assert!(attempt(&g, &b, &[0.5; 2], &disabled, &seed).0.is_none());
        }
    }
}
