//! Single-point positioning: iterative weighted least squares on
//! pseudoranges, dilution of precision, and RAIM fault detection and
//! exclusion (FDE) by a chi-square residual test.

use crate::geodesy::{azimuth_elevation, EnuFrame};
use crate::orbit::SatelliteId;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// One pseudorange the solver may use.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PseudorangeInput {
    /// Satellite id.
    pub id: SatelliteId,
    /// Satellite ECEF position at transmission in meters.
    pub satellite_ecef_m: Vec3,
    /// Measured pseudorange in meters.
    pub pseudorange_m: f64,
    /// A-priori one-sigma measurement error in meters (sets the weight).
    pub sigma_m: f64,
}

/// Dilution of precision.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Dop {
    /// Geometric DOP.
    pub gdop: f64,
    /// Position DOP.
    pub pdop: f64,
    /// Horizontal DOP.
    pub hdop: f64,
    /// Vertical DOP.
    pub vdop: f64,
}

/// Solver settings.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SolverConfig {
    /// Maximum Gauss-Newton iterations.
    pub max_iterations: usize,
    /// Convergence threshold on the position update in meters.
    pub convergence_m: f64,
    /// Whether to run RAIM fault detection and exclusion.
    pub raim: bool,
    /// False-alarm probability of the chi-square residual test.
    pub raim_false_alarm_probability: f64,
    /// Maximum number of satellites RAIM may exclude in one epoch.
    pub raim_max_exclusions: usize,
}

impl Default for SolverConfig {
    fn default() -> Self {
        Self {
            max_iterations: 12,
            convergence_m: 1.0e-4,
            raim: true,
            raim_false_alarm_probability: 1.0e-3,
            raim_max_exclusions: 4,
        }
    }
}

/// A position solution.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PositionSolution {
    /// Receiver ECEF position in meters.
    pub position_ecef_m: Vec3,
    /// Receiver clock bias times the speed of light, in meters.
    pub clock_bias_m: f64,
    /// Dilution of precision in the local ENU frame.
    pub dop: Dop,
    /// ENU position covariance (row-major, m²) from the a-priori sigmas.
    pub covariance_enu_m2: [[f64; 3]; 3],
    /// Satellites used, in input order.
    pub used: Vec<SatelliteId>,
    /// Satellites RAIM excluded, in exclusion order.
    pub excluded: Vec<SatelliteId>,
    /// Post-fit residuals of the used satellites in meters.
    pub residuals_m: Vec<f64>,
    /// Weighted sum of squared residuals.
    pub chi_square: f64,
    /// Whether the final residual test passed (always true without RAIM or
    /// without redundancy).
    pub consistent: bool,
}

/// Solver failures.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum SolveError {
    /// Fewer than four usable pseudoranges.
    #[error("need at least four pseudoranges")]
    TooFewSatellites,
    /// The normal matrix was singular (degenerate geometry).
    #[error("degenerate satellite geometry")]
    Degenerate,
    /// The iteration did not converge to a finite solution.
    #[error("position solution diverged")]
    Diverged,
}

/// Solves for position and clock bias, optionally with RAIM FDE.
///
/// `initial_ecef_m` seeds the iteration; `frame` sets the ENU axes of the DOP
/// and covariance.
pub fn solve_position(
    inputs: &[PseudorangeInput],
    initial_ecef_m: Vec3,
    frame: &EnuFrame,
    config: &SolverConfig,
) -> Result<PositionSolution, SolveError> {
    let mut active: Vec<PseudorangeInput> = inputs.to_vec();
    let mut excluded = Vec::new();
    loop {
        let mut solution = least_squares(&active, initial_ecef_m, frame, config)?;
        let dof = active.len().saturating_sub(4);
        if !config.raim || dof == 0 {
            solution.excluded = excluded;
            return Ok(solution);
        }
        let threshold = chi_square_quantile(dof, 1.0 - config.raim_false_alarm_probability);
        if solution.chi_square <= threshold {
            solution.excluded = excluded;
            return Ok(solution);
        }
        // Exclusion needs one spare redundancy left to verify the result.
        if excluded.len() >= config.raim_max_exclusions || dof < 2 {
            solution.consistent = false;
            solution.excluded = excluded;
            return Ok(solution);
        }
        let worst = solution
            .residuals_m
            .iter()
            .zip(&active)
            .enumerate()
            .max_by(|(_, (ra, a)), (_, (rb, b))| {
                (ra.abs() / a.sigma_m).total_cmp(&(rb.abs() / b.sigma_m))
            })
            .map(|(index, _)| index)
            .ok_or(SolveError::TooFewSatellites)?;
        excluded.push(active.remove(worst).id);
    }
}

fn least_squares(
    inputs: &[PseudorangeInput],
    initial_ecef_m: Vec3,
    frame: &EnuFrame,
    config: &SolverConfig,
) -> Result<PositionSolution, SolveError> {
    if inputs.len() < 4 {
        return Err(SolveError::TooFewSatellites);
    }
    let mut position = initial_ecef_m;
    let mut clock = 0.0;
    let mut normal_inverse = [[0.0; 4]; 4];
    for _ in 0..config.max_iterations.max(1) {
        let mut normal = [[0.0; 4]; 4];
        let mut rhs = [0.0; 4];
        for input in inputs {
            let line = input.satellite_ecef_m - position;
            let range = line.length();
            let unit = line / range;
            let row = [-unit.x, -unit.y, -unit.z, 1.0];
            let weight = 1.0 / (input.sigma_m * input.sigma_m);
            let residual = input.pseudorange_m - (range + clock);
            for i in 0..4 {
                rhs[i] += weight * row[i] * residual;
                for j in 0..4 {
                    normal[i][j] += weight * row[i] * row[j];
                }
            }
        }
        normal_inverse = invert4(normal).ok_or(SolveError::Degenerate)?;
        let mut delta = [0.0; 4];
        for i in 0..4 {
            delta[i] = (0..4).map(|j| normal_inverse[i][j] * rhs[j]).sum();
        }
        position += Vec3::new(delta[0], delta[1], delta[2]);
        clock += delta[3];
        if !position.is_finite() || !clock.is_finite() {
            return Err(SolveError::Diverged);
        }
        if Vec3::new(delta[0], delta[1], delta[2]).length() < config.convergence_m {
            break;
        }
    }

    let mut residuals_m = Vec::with_capacity(inputs.len());
    let mut chi_square = 0.0;
    let mut geometry = [[0.0; 4]; 4];
    for input in inputs {
        let line = input.satellite_ecef_m - position;
        let residual = input.pseudorange_m - (line.length() + clock);
        chi_square += (residual / input.sigma_m).powi(2);
        residuals_m.push(residual);
        // Unweighted geometry in ENU for DOP.
        let enu = frame.ecef_vector_to_enu(line).normalize();
        let row = [-enu.x, -enu.y, -enu.z, 1.0];
        for i in 0..4 {
            for j in 0..4 {
                geometry[i][j] += row[i] * row[j];
            }
        }
    }
    let dop_matrix = invert4(geometry).ok_or(SolveError::Degenerate)?;
    let dop = Dop {
        gdop: (0..4).map(|i| dop_matrix[i][i]).sum::<f64>().sqrt(),
        pdop: (dop_matrix[0][0] + dop_matrix[1][1] + dop_matrix[2][2]).sqrt(),
        hdop: (dop_matrix[0][0] + dop_matrix[1][1]).sqrt(),
        vdop: dop_matrix[2][2].sqrt(),
    };

    // Rotate the weighted ECEF position covariance into ENU.
    let axes = [
        frame.enu_vector_to_ecef(Vec3::X),
        frame.enu_vector_to_ecef(Vec3::Y),
        frame.enu_vector_to_ecef(Vec3::Z),
    ];
    let mut covariance_enu_m2 = [[0.0; 3]; 3];
    for (a, axis_a) in axes.iter().enumerate() {
        for (b, axis_b) in axes.iter().enumerate() {
            let mut sum = 0.0;
            for i in 0..3 {
                for j in 0..3 {
                    sum += axis_a[i] * normal_inverse[i][j] * axis_b[j];
                }
            }
            covariance_enu_m2[a][b] = sum;
        }
    }

    Ok(PositionSolution {
        position_ecef_m: position,
        clock_bias_m: clock,
        dop,
        covariance_enu_m2,
        used: inputs.iter().map(|i| i.id).collect(),
        excluded: Vec::new(),
        residuals_m,
        chi_square,
        consistent: true,
    })
}

/// Elevation of a satellite seen from an ECEF position, in radians.
pub fn elevation_from(frame: &EnuFrame, receiver_ecef_m: Vec3, satellite_ecef_m: Vec3) -> f64 {
    let enu = frame.ecef_vector_to_enu(satellite_ecef_m - receiver_ecef_m);
    azimuth_elevation(enu).1
}

/// Upper `probability` quantile of a chi-square distribution with `dof`
/// degrees of freedom (Wilson-Hilferty approximation).
pub fn chi_square_quantile(dof: usize, probability: f64) -> f64 {
    let k = dof.max(1) as f64;
    let z = normal_quantile(probability);
    let h = 2.0 / (9.0 * k);
    k * (1.0 - h + z * h.sqrt()).powi(3)
}

/// Standard normal quantile (Acklam's rational approximation).
fn normal_quantile(p: f64) -> f64 {
    const A: [f64; 6] = [
        -3.969_683_028_665_376e1,
        2.209_460_984_245_205e2,
        -2.759_285_104_469_687e2,
        1.383_577_518_672_69e2,
        -3.066_479_806_614_716e1,
        2.506_628_277_459_239,
    ];
    const B: [f64; 5] = [
        -5.447_609_879_822_406e1,
        1.615_858_368_580_409e2,
        -1.556_989_798_598_866e2,
        6.680_131_188_771_972e1,
        -1.328_068_155_288_572e1,
    ];
    const C: [f64; 6] = [
        -7.784_894_002_430_293e-3,
        -3.223_964_580_411_365e-1,
        -2.400_758_277_161_838,
        -2.549_732_539_343_734,
        4.374_664_141_464_968,
        2.938_163_982_698_783,
    ];
    const D: [f64; 4] = [
        7.784_695_709_041_462e-3,
        3.224_671_290_700_398e-1,
        2.445_134_137_142_996,
        3.754_408_661_907_416,
    ];
    let p = p.clamp(1.0e-12, 1.0 - 1.0e-12);
    let tail = |q: f64| {
        (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    };
    if p < 0.02425 {
        tail((-2.0 * p.ln()).sqrt())
    } else if p > 1.0 - 0.02425 {
        -tail((-2.0 * (1.0 - p).ln()).sqrt())
    } else {
        let q = p - 0.5;
        let r = q * q;
        (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
            / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
    }
}

fn invert4(m: [[f64; 4]; 4]) -> Option<[[f64; 4]; 4]> {
    let mut a = m;
    let mut inv = [[0.0; 4]; 4];
    for (i, row) in inv.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    for col in 0..4 {
        let pivot = (col..4).max_by(|&x, &y| a[x][col].abs().total_cmp(&a[y][col].abs()))?;
        if a[pivot][col].abs() < 1.0e-12 {
            return None;
        }
        a.swap(col, pivot);
        inv.swap(col, pivot);
        let scale = 1.0 / a[col][col];
        for j in 0..4 {
            a[col][j] *= scale;
            inv[col][j] *= scale;
        }
        for row in 0..4 {
            if row != col {
                let factor = a[row][col];
                if factor != 0.0 {
                    for j in 0..4 {
                        a[row][j] -= factor * a[col][j];
                        inv[row][j] -= factor * inv[col][j];
                    }
                }
            }
        }
    }
    Some(inv)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geodesy::Geodetic;
    use crate::orbit::{Constellation, GnssSystem};
    use approx::assert_relative_eq;

    fn setup(bias: Option<(usize, f64)>) -> (EnuFrame, Vec3, Vec<PseudorangeInput>) {
        let frame = EnuFrame::new(Geodetic::from_degrees(35.0, 139.0, 30.0));
        let truth = frame.enu_to_ecef(Vec3::new(12.0, -7.0, 1.0));
        let clock = 3_000.0;
        let mut inputs = Vec::new();
        for sat in Constellation::gps_galileo_nominal().satellites() {
            let pos = sat.orbit.position_ecef(0.0);
            if elevation_from(&frame, truth, pos) < 10.0_f64.to_radians() {
                continue;
            }
            inputs.push(PseudorangeInput {
                id: sat.id,
                satellite_ecef_m: pos,
                pseudorange_m: (pos - truth).length() + clock,
                sigma_m: 1.0,
            });
        }
        if let Some((index, offset)) = bias {
            inputs[index].pseudorange_m += offset;
        }
        (frame, truth, inputs)
    }

    #[test]
    fn noiseless_pseudoranges_recover_position_and_clock() {
        let (frame, truth, inputs) = setup(None);
        assert!(inputs.len() >= 8, "{}", inputs.len());
        let solution = solve_position(
            &inputs,
            frame.origin_ecef_m,
            &frame,
            &SolverConfig::default(),
        )
        .unwrap();
        assert!((solution.position_ecef_m - truth).length() < 1e-4);
        assert_relative_eq!(solution.clock_bias_m, 3_000.0, epsilon = 1e-4);
        assert!(solution.consistent);
        assert!(solution.dop.hdop > 0.3 && solution.dop.hdop < 3.0);
        assert!(solution.dop.vdop > solution.dop.hdop);
    }

    #[test]
    fn raim_excludes_an_nlos_like_bias() {
        let (frame, truth, inputs) = setup(Some((2, 60.0)));
        let bad = inputs[2].id;
        let solution = solve_position(
            &inputs,
            frame.origin_ecef_m,
            &frame,
            &SolverConfig::default(),
        )
        .unwrap();
        assert_eq!(solution.excluded, vec![bad]);
        assert!((solution.position_ecef_m - truth).length() < 1e-3);

        let no_raim = SolverConfig {
            raim: false,
            ..SolverConfig::default()
        };
        let biased = solve_position(&inputs, frame.origin_ecef_m, &frame, &no_raim).unwrap();
        assert!((biased.position_ecef_m - truth).length() > 1.0);
    }

    #[test]
    fn rejects_too_few_satellites() {
        let (frame, _, inputs) = setup(None);
        assert_eq!(
            solve_position(
                &inputs[..3],
                frame.origin_ecef_m,
                &frame,
                &SolverConfig::default()
            ),
            Err(SolveError::TooFewSatellites)
        );
        assert_eq!(inputs[0].id.system, GnssSystem::Gps);
    }

    #[test]
    fn chi_square_quantile_matches_tables() {
        // Tabulated 0.999 quantiles: dof 1 -> 10.83, 5 -> 20.52, 10 -> 29.59.
        assert_relative_eq!(chi_square_quantile(5, 0.999), 20.52, epsilon = 0.3);
        assert_relative_eq!(chi_square_quantile(10, 0.999), 29.59, epsilon = 0.3);
    }
}
