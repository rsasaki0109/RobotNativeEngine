//! Bounded normal active-set acceleration with full velocity validation.
use super::{ContactSolution, ContactSolverConfig, ContactState, DenseMatrix};

/// Frictional contacts can use a normal-only solution only when every loaded
/// frictional point has zero tangential velocity within the configured tolerance.
/// The original seed is never changed on rejection.
pub(super) fn attempt(
    matrix: &DenseMatrix,
    free: &[f64],
    friction: &[f64],
    config: &ContactSolverConfig,
    seed: &[[f64; 3]],
) -> (Option<ContactSolution>, usize) {
    let count = friction.len();
    let tolerance = config.velocity_tolerance_m_s;
    let frictionless = friction.iter().all(|mu| *mu == 0.0);
    let eligible = if frictionless {
        (2..=32).contains(&count)
    } else {
        (2..=64).contains(&count)
            && free
                .chunks_exact(3)
                .all(|v| v[0].abs() <= tolerance && v[1].abs() <= tolerance)
    };
    if !eligible
        || config.regularization != 0.0
        || config.relaxation != 1.0
        || !tolerance.is_finite()
        || tolerance <= 0.0
    {
        return (None, 0);
    }
    let (candidate, passes) = solve(matrix, free, count, seed, config.max_iterations, tolerance);
    let Some(normal) = candidate else {
        return (None, passes);
    };
    if !frictionless && !tangentially_static(matrix, free, friction, &normal, tolerance) {
        return (None, passes);
    }
    let mut result = solution(normal, seed, passes);
    for (state, mu) in result.states.iter_mut().zip(friction) {
        if *state == ContactState::Sliding && *mu > 0.0 {
            *state = ContactState::Sticking;
        }
    }
    (Some(result), passes)
}

fn tangentially_static(
    matrix: &DenseMatrix,
    free: &[f64],
    friction: &[f64],
    normal: &[f64],
    tolerance: f64,
) -> bool {
    friction.iter().enumerate().all(|(i, mu)| {
        normal[i] == 0.0
            || *mu == 0.0
            || (0..2).all(|axis| {
                let row = 3 * i + axis;
                let velocity = free[row]
                    + normal
                        .iter()
                        .enumerate()
                        .map(|(j, impulse)| matrix.get(row, 3 * j + 2) * impulse)
                        .sum::<f64>();
                velocity.is_finite() && velocity.abs() <= tolerance
            })
    })
}

/// Returns a candidate only after checking the original unilateral equations.
/// A singular/inconsistent active set or cycling leaves the caller's GS path intact.
pub(super) fn solve(
    matrix: &DenseMatrix,
    free: &[f64],
    count: usize,
    seed: &[[f64; 3]],
    budget: usize,
    tolerance: f64,
) -> (Option<Vec<f64>>, usize) {
    // Reuse an equilibrated warm start only after checking the original
    // equations. This avoids refactorizing a resting stack every timestep.
    if budget > 0
        && (0..count).all(|i| {
            let velocity = free[3 * i + 2]
                + (0..count)
                    .map(|j| matrix.get(3 * i + 2, 3 * j + 2) * seed[j][2])
                    .sum::<f64>();
            velocity.is_finite()
                && seed[i][2].is_finite()
                && seed[i][2] >= 0.0
                && if seed[i][2] > 0.0 {
                    velocity.abs() <= tolerance
                } else {
                    velocity >= -tolerance
                }
        })
    {
        return (Some(seed.iter().map(|p| p[2]).collect()), 0);
    }
    let g: Vec<Vec<f64>> = (0..count)
        .map(|i| {
            (0..count)
                .map(|j| matrix.get(3 * i + 2, 3 * j + 2))
                .collect()
        })
        .collect();
    let b: Vec<f64> = (0..count).map(|i| free[3 * i + 2]).collect();
    let mut active: Vec<bool> = b
        .iter()
        .zip(seed)
        .map(|(v, p)| *v < 0.0 || p[2] > 0.0)
        .collect();
    for pass in 1..=budget.min(2 * count + 1) {
        let indices: Vec<usize> = (0..count).filter(|i| active[*i]).collect();
        let rows: Vec<Vec<f64>> = indices
            .iter()
            .map(|i| {
                indices
                    .iter()
                    .map(|j| g[*i][*j])
                    .chain(std::iter::once(-b[*i]))
                    .collect()
            })
            .collect();
        let Some(local) = rank_solve(rows) else {
            return (None, pass);
        };
        let mut impulse = vec![0.0; count];
        for (i, value) in indices.iter().zip(local) {
            impulse[*i] = value;
        }
        if let Some(i) = (0..count)
            .filter(|i| impulse[*i] < 0.0)
            .min_by(|a, b| impulse[*a].total_cmp(&impulse[*b]))
        {
            active[i] = false;
            continue;
        }
        let velocity: Vec<f64> = (0..count)
            .map(|i| b[i] + g[i].iter().zip(&impulse).map(|(a, x)| a * x).sum::<f64>())
            .collect();
        if let Some(i) = (0..count)
            .filter(|i| !active[*i] && velocity[*i] < -tolerance)
            .min_by(|a, b| velocity[*a].total_cmp(&velocity[*b]))
        {
            active[i] = true;
            continue;
        }
        // Check complementarity as a velocity residual, not change of iterates.
        if impulse.iter().all(|x| x.is_finite() && *x >= 0.0)
            && velocity.iter().enumerate().all(|(i, v)| {
                v.is_finite()
                    && if impulse[i] > 0.0 {
                        v.abs() <= tolerance
                    } else {
                        *v >= -tolerance
                    }
            })
        {
            return (Some(impulse), pass);
        }
        return (None, pass);
    }
    (None, budget.min(2 * count + 1))
}

/// Deterministic row-pivoted elimination. Free variables of a rank-deficient
/// system are zero; no compliance or diagonal jitter is added. The caller
/// validates the candidate against the full, unmodified contact matrix.
#[allow(clippy::needless_range_loop)] // Indexed elimination avoids copying pivot rows.
fn rank_solve(mut a: Vec<Vec<f64>>) -> Option<Vec<f64>> {
    let n = a.len();
    let scale = a
        .iter()
        .flat_map(|r| &r[..n])
        .map(|x| x.abs())
        .fold(0.0, f64::max);
    let cutoff = scale * 1e-12;
    let mut pivots = Vec::new();
    for col in 0..n {
        let rank = pivots.len();
        let row = (rank..n).max_by(|i, j| a[*i][col].abs().total_cmp(&a[*j][col].abs()))?;
        if a[row][col].abs() <= cutoff {
            continue;
        }
        a.swap(rank, row);
        let pivot = a[rank][col];
        for k in col..=n {
            a[rank][k] /= pivot;
        }
        for i in 0..n {
            if i == rank {
                continue;
            }
            let factor = a[i][col];
            for k in col..=n {
                a[i][k] -= factor * a[rank][k];
            }
        }
        pivots.push(col);
    }
    let mut x = vec![0.0; n];
    for (row, col) in pivots.into_iter().enumerate() {
        x[col] = a[row][n];
    }
    x.iter().all(|v| v.is_finite()).then_some(x)
}

pub(super) fn solution(normal: Vec<f64>, seed: &[[f64; 3]], passes: usize) -> ContactSolution {
    let last_change = normal
        .iter()
        .zip(seed)
        .map(|(n, old)| old[0].hypot(old[1]).hypot(*n - old[2]))
        .fold(0.0, f64::max);
    ContactSolution {
        states: normal
            .iter()
            .map(|n| {
                if *n > 0.0 {
                    ContactState::Sliding
                } else {
                    ContactState::Open
                }
            })
            .collect(),
        impulses_n_s: normal.into_iter().map(|n| [0.0, 0.0, n]).collect(),
        iterations: 0,
        normal_block_iterations: passes,
        normal_block_solved: true,
        converged: true,
        last_change_n_s: last_change,
    }
}
