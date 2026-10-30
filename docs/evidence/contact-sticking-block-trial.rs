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
