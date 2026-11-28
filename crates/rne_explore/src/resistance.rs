//! Effective resistance on a pose graph, the measure revisits use to choose
//! the loop worth closing.
//!
//! Treat each odometry step as a resistor of its length and each loop
//! closure as a resistor of a small fixed length. The effective resistance
//! between two poses is then the uncertainty of their relative pose that the
//! graph leaves (for a graph of isotropic edges, exactly so). Adding a loop
//! closure of resistance `r` between them multiplies the number of the
//! graph's weighted spanning trees, the D-optimality measure of a pose graph,
//! by `1 + R / r`, so the most informative loop to close is the one with the
//! largest effective resistance. On a chain with no closures that is the
//! trajectory length between the poses; closures already made shorten it.

use rne_nav::Pose2d;

/// Upper bound on the number of nodes the dense solve works with; longer
/// trajectories are grouped into coarser steps.
const MAX_NODES: usize = 400;

/// Effective resistance from the last pose of `history` to every pose, in
/// metres of odometry. `loops` are loop-closure edges as `(from, to)` pose
/// indices, each counted as a resistor of `loop_m`. Consecutive poses closer
/// together than `step_m` along the trajectory are merged (their
/// resistances are then exact to within `step_m`).
pub fn effective_resistance_to_end(
    history: &[Pose2d],
    loops: &[(usize, usize)],
    loop_m: f64,
    step_m: f64,
) -> Vec<f64> {
    let n = history.len();
    if n == 0 {
        return Vec::new();
    }
    let mut along = vec![0.0; n];
    for i in 1..n {
        let (a, b) = (history[i - 1], history[i]);
        along[i] = along[i - 1] + (b.x_m - a.x_m).hypot(b.y_m - a.y_m);
    }
    let total = along[n - 1];
    let step = step_m.max(total / MAX_NODES as f64).max(1e-6);
    // Group poses into chunks of trajectory length `step`; the last pose's
    // chunk is the ground.
    let chunk_of: Vec<usize> = along.iter().map(|a| (a / step).floor() as usize).collect();
    let chunks = chunk_of[n - 1] + 1;
    let mut chunk_along = vec![(0.0, 0usize); chunks];
    for (i, &c) in chunk_of.iter().enumerate() {
        chunk_along[c].0 += along[i];
        chunk_along[c].1 += 1;
    }
    // Empty chunks (a jump longer than a step) sit halfway along their span.
    let position: Vec<f64> = chunk_along
        .iter()
        .enumerate()
        .map(|(c, &(sum, count))| {
            if count > 0 {
                sum / count as f64
            } else {
                (c as f64 + 0.5) * step
            }
        })
        .collect();

    // Conductance Laplacian over chunks.
    let mut laplacian = vec![0.0; chunks * chunks];
    let mut connect = |a: usize, b: usize, resistance: f64| {
        if a == b {
            return;
        }
        let g = 1.0 / resistance.max(1e-6);
        laplacian[a * chunks + a] += g;
        laplacian[b * chunks + b] += g;
        laplacian[a * chunks + b] -= g;
        laplacian[b * chunks + a] -= g;
    };
    for c in 1..chunks {
        connect(c - 1, c, position[c] - position[c - 1]);
    }
    for &(a, b) in loops {
        if a < n && b < n {
            connect(chunk_of[a], chunk_of[b], loop_m);
        }
    }

    // Ground the last chunk and invert the rest: R(c) = (L_g^-1)_cc.
    let ground = chunks - 1;
    let size = ground;
    let mut reduced = vec![0.0; size * size];
    for r in 0..size {
        for c in 0..size {
            reduced[r * size + c] = laplacian[r * chunks + c];
        }
    }
    let diagonal = inverse_diagonal(&reduced, size);
    let mut per_chunk = diagonal;
    per_chunk.push(0.0);
    chunk_of.iter().map(|&c| per_chunk[c]).collect()
}

/// The diagonal of the inverse of a symmetric positive-definite matrix, by
/// Cholesky factorization and one solve per column.
fn inverse_diagonal(matrix: &[f64], size: usize) -> Vec<f64> {
    let mut lower = vec![0.0; size * size];
    for row in 0..size {
        for col in 0..=row {
            let mut sum = matrix[row * size + col];
            for k in 0..col {
                sum -= lower[row * size + k] * lower[col * size + k];
            }
            lower[row * size + col] = if row == col {
                sum.max(1e-12).sqrt()
            } else {
                sum / lower[col * size + col]
            };
        }
    }
    // (A^-1)_ii = sum_k (L^-1)_ki^2: solve L x = e_i for each i.
    let mut out = vec![0.0; size];
    let mut x = vec![0.0; size];
    for (i, slot) in out.iter_mut().enumerate() {
        x.iter_mut().for_each(|v| *v = 0.0);
        let mut sum_sq = 0.0;
        for row in i..size {
            let mut value = if row == i { 1.0 } else { 0.0 };
            for k in i..row {
                value -= lower[row * size + k] * x[k];
            }
            x[row] = value / lower[row * size + row];
            sum_sq += x[row] * x[row];
        }
        *slot = sum_sq;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    fn line(n: usize, spacing: f64) -> Vec<Pose2d> {
        (0..n)
            .map(|i| Pose2d::new(i as f64 * spacing, 0.0, 0.0))
            .collect()
    }

    #[test]
    fn a_chain_without_closures_gives_the_trajectory_length() {
        let history = line(41, 0.25);
        let r = effective_resistance_to_end(&history, &[], 0.5, 0.5);
        // 10 m back to the start, 5 m back to the middle, within one step.
        assert!((r[0] - 10.0).abs() < 0.5, "{}", r[0]);
        assert!((r[20] - 5.0).abs() < 0.5, "{}", r[20]);
        assert!(r[40].abs() < 1e-9);
    }

    #[test]
    fn a_closed_loop_shortens_the_resistance_behind_it() {
        // A 20 m trajectory whose end (node 80) was closed to its start.
        let history = line(81, 0.25);
        let open = effective_resistance_to_end(&history, &[], 0.5, 0.5);
        let closed = effective_resistance_to_end(&history, &[(0, 80)], 0.5, 0.5);
        // The start is now 0.5 m away through the closure, not 20 m.
        assert!(closed[0] < 1.0, "{}", closed[0]);
        // The middle is reached both ways: two 10 m paths in parallel, about
        // 5 m, against 10 m before.
        assert!((closed[40] - 5.1).abs() < 0.6, "{}", closed[40]);
        assert!(open[40] > 9.5);
    }
}
