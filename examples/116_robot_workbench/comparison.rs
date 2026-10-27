//! Offline comparison of evaluation evidence; never runs or modifies a recording.
use anyhow::{ensure, Context, Result};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeSet, io::Read, path::Path};

const TOLERANCE_M: f64 = 0.001;
// The v1 catalog observes both grasp pads for 20 batches over two seconds.
const MIN_HOLD_DURATION_S: f64 = 2.0;
const MIN_HOLD_CONTACT_SAMPLES: usize = 20;
const MAX_REPORT_BYTES: u64 = 1024 * 1024;

#[derive(Debug, Deserialize)]
struct Report {
    schema_version: u32,
    catalog: String,
    passed: bool,
    cases: Vec<Case>,
}

#[derive(Debug, Deserialize)]
struct Case {
    name: String,
    expected: String,
    expected_reason: Option<String>,
    expected_phase: Option<String>,
    outcome: String,
    expectation_met: bool,
    reason: Option<String>,
    failed_phase: Option<String>,
    replay_verified: Option<bool>,
    placement: Option<Placement>,
}

#[derive(Debug, Deserialize)]
struct Placement {
    grasp: Option<Grasp>,
    planned_destination_m: [f64; 3],
    placement_error_x_z_m: [f64; 2],
    maximum_settled_drift_m: f64,
}

#[derive(Debug, Deserialize)]
struct Grasp {
    lift_m: f64,
    minimum_hold_lift_m: f64,
    hold_duration_s: Option<f64>,
    contact_samples: Option<usize>,
}

impl Grasp {
    fn hold_complete(&self) -> bool {
        self.hold_duration_s
            .is_some_and(|s| s >= MIN_HOLD_DURATION_S)
            && self
                .contact_samples
                .is_some_and(|n| n >= MIN_HOLD_CONTACT_SAMPLES)
    }
}

#[derive(Debug, Serialize)]
pub(crate) struct Comparison {
    pub passed: bool,
    tolerance_m: f64,
    regressions: Vec<String>,
}

fn read(path: &Path) -> Result<Report> {
    let mut bytes = Vec::new();
    std::fs::File::open(path)?
        .take(MAX_REPORT_BYTES + 1)
        .read_to_end(&mut bytes)?;
    ensure!(
        bytes.len() as u64 <= MAX_REPORT_BYTES,
        "report exceeds 1 MiB"
    );
    serde_json::from_slice(&bytes).context("invalid evaluation report")
}

fn validate(report: &Report) -> Result<()> {
    ensure!(report.schema_version == 1, "unsupported report schema");
    ensure!(
        report.catalog == "lekiwi-pick-place-v1",
        "unsupported catalog"
    );
    let names: BTreeSet<_> = report.cases.iter().map(|c| c.name.as_str()).collect();
    ensure!(
        report.cases.len() == 6
            && names
                == BTreeSet::from([
                    "nominal",
                    "diagonal",
                    "reverse",
                    "missed-block",
                    "unequal-height",
                    "out-of-range"
                ]),
        "catalog must contain exactly six unique cases"
    );
    for case in &report.cases {
        ensure!(
            ["completed", "failed", "rejected", "error"].contains(&case.outcome.as_str()),
            "invalid outcome"
        );
        if let Some(p) = &case.placement {
            if let Some(grasp) = &p.grasp {
                ensure!(
                    [grasp.lift_m, grasp.minimum_hold_lift_m]
                        .iter()
                        .all(|v| v.is_finite() && *v >= 0.0),
                    "invalid grasp metrics"
                );
                ensure!(
                    grasp
                        .hold_duration_s
                        .is_none_or(|s| s.is_finite() && s >= 0.0),
                    "invalid hold duration"
                );
            }
            ensure!(
                p.planned_destination_m
                    .iter()
                    .chain(&p.placement_error_x_z_m)
                    .all(|v| v.is_finite())
                    && p.maximum_settled_drift_m.is_finite()
                    && p.maximum_settled_drift_m >= 0.0,
                "invalid placement metrics"
            );
        }
    }
    Ok(())
}

fn healthy(case: &Case) -> bool {
    case.expectation_met
        && case.outcome == case.expected
        && case.outcome != "error"
        && case.expected_reason.as_ref().is_none_or(|r| {
            case.reason
                .as_ref()
                .is_some_and(|actual| actual.contains(r))
        })
        && case
            .expected_phase
            .as_ref()
            .is_none_or(|p| case.failed_phase.as_ref() == Some(p))
        && match case.outcome.as_str() {
            "completed" => {
                case.replay_verified == Some(true)
                    && case
                        .placement
                        .as_ref()
                        .is_some_and(|p| p.grasp.as_ref().is_some_and(Grasp::hold_complete))
            }
            "failed" => case.replay_verified == Some(true),
            "rejected" => case.replay_verified.is_none(),
            _ => false,
        }
}

fn compare(baseline: &Report, candidate: &Report) -> Result<Comparison> {
    validate(baseline)?;
    validate(candidate)?;
    ensure!(
        baseline.passed && baseline.cases.iter().all(healthy),
        "baseline must be a passing evaluation with valid evidence"
    );
    let mut regressions = Vec::new();
    if !candidate.passed {
        regressions.push("candidate evaluation did not pass".into());
    }
    for old in &baseline.cases {
        let new = candidate.cases.iter().find(|c| c.name == old.name).unwrap();
        ensure!(
            old.expected == new.expected
                && old.expected_reason == new.expected_reason
                && old.expected_phase == new.expected_phase,
            "{}: changed case expectations",
            old.name
        );
        if !healthy(new) {
            let reason = if new.outcome == "completed"
                && new
                    .placement
                    .as_ref()
                    .and_then(|p| p.grasp.as_ref())
                    .is_some_and(|g| !g.hold_complete())
            {
                "hold evidence incomplete: requires at least 2 s and 20 contact samples"
            } else {
                "outcome, expectation, replay, or grasp evidence failed"
            };
            regressions.push(format!("{}: {reason}", old.name));
        }
        if let (Some(a), Some(b)) = (&old.placement, &new.placement) {
            ensure!(
                a.planned_destination_m == b.planned_destination_m,
                "{}: changed destination",
                old.name
            );
            let error =
                |p: &Placement| p.placement_error_x_z_m[0].hypot(p.placement_error_x_z_m[1]);
            for (label, before, after) in [
                ("placement error", error(a), error(b)),
                (
                    "settled drift",
                    a.maximum_settled_drift_m,
                    b.maximum_settled_drift_m,
                ),
            ] {
                if after > before + TOLERANCE_M {
                    regressions.push(format!(
                        "{}: {label} increased from {before:.6} m to {after:.6} m",
                        old.name
                    ));
                }
            }
            if let (Some(a), Some(b)) = (&a.grasp, &b.grasp) {
                for (label, before, after) in [
                    ("grasp lift", a.lift_m, b.lift_m),
                    (
                        "minimum hold lift",
                        a.minimum_hold_lift_m,
                        b.minimum_hold_lift_m,
                    ),
                ] {
                    if after < before - TOLERANCE_M {
                        regressions.push(format!(
                            "{}: {label} decreased from {before:.6} m to {after:.6} m",
                            old.name
                        ));
                    }
                }
            }
        }
    }
    Ok(Comparison {
        passed: regressions.is_empty(),
        tolerance_m: TOLERANCE_M,
        regressions,
    })
}

/// Compares reports with a 1 mm allowance for error/drift increases or lift decreases.
pub(crate) fn run(baseline: &Path, candidate: &Path) -> Result<Comparison> {
    compare(&read(baseline)?, &read(candidate)?)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture() -> Report {
        let cases = [
            "nominal",
            "diagonal",
            "reverse",
            "missed-block",
            "unequal-height",
            "out-of-range",
        ]
        .into_iter()
        .map(|name| Case {
            name: name.into(),
            expected: "completed".into(),
            expected_reason: None,
            expected_phase: None,
            outcome: "completed".into(),
            expectation_met: true,
            reason: None,
            failed_phase: None,
            replay_verified: Some(true),
            placement: Some(Placement {
                grasp: Some(Grasp {
                    lift_m: 0.09,
                    minimum_hold_lift_m: 0.088,
                    hold_duration_s: Some(2.000_000_16),
                    contact_samples: Some(20),
                }),
                planned_destination_m: [0.0; 3],
                placement_error_x_z_m: [0.001, 0.0],
                maximum_settled_drift_m: 0.0,
            }),
        })
        .collect();
        Report {
            schema_version: 1,
            catalog: "lekiwi-pick-place-v1".into(),
            passed: true,
            cases,
        }
    }
    #[test]
    fn order_is_irrelevant_and_improvement_passes() {
        let mut candidate = fixture();
        candidate.cases.reverse();
        candidate.cases[0]
            .placement
            .as_mut()
            .unwrap()
            .placement_error_x_z_m = [0.0; 2];
        assert!(compare(&fixture(), &candidate).unwrap().passed);
    }
    #[test]
    fn detects_metrics_and_replay_even_with_stale_pass_flag() {
        let mut candidate = fixture();
        candidate.cases[0]
            .placement
            .as_mut()
            .unwrap()
            .placement_error_x_z_m = [0.003, 0.0];
        candidate.cases[1]
            .placement
            .as_mut()
            .unwrap()
            .maximum_settled_drift_m = 0.002;
        candidate.cases[2].replay_verified = Some(false);
        assert_eq!(
            compare(&fixture(), &candidate).unwrap().regressions.len(),
            3
        );
        candidate.cases[3].placement = None;
        assert_eq!(
            compare(&fixture(), &candidate).unwrap().regressions.len(),
            4
        );
    }
    #[test]
    fn lift_decreases_are_detected_even_when_placement_and_pass_flag_are_unchanged() {
        let baseline = fixture();
        let mut candidate = fixture();
        let grasp = candidate.cases[0]
            .placement
            .as_mut()
            .unwrap()
            .grasp
            .as_mut()
            .unwrap();
        grasp.lift_m -= 0.002;
        grasp.minimum_hold_lift_m -= 0.002;
        let result = compare(&baseline, &candidate).unwrap();
        assert!(!result.passed);
        assert_eq!(result.regressions.len(), 2);
        assert!(result.regressions[0].contains("nominal: grasp lift decreased"));
        assert!(result.regressions[1].contains("nominal: minimum hold lift decreased"));
    }

    #[test]
    fn lift_allowance_boundary_and_improvement_pass_but_tiny_exceedance_fails() {
        for minimum_hold in [false, true] {
            let mut candidate = fixture();
            let before = if minimum_hold { 0.088 } else { 0.09 };
            for after in [
                before + 0.01,
                before - TOLERANCE_M,
                before - TOLERANCE_M - 1e-9,
            ] {
                let grasp = candidate.cases[0]
                    .placement
                    .as_mut()
                    .unwrap()
                    .grasp
                    .as_mut()
                    .unwrap();
                if minimum_hold {
                    grasp.minimum_hold_lift_m = after;
                } else {
                    grasp.lift_m = after;
                }
                assert_eq!(
                    compare(&fixture(), &candidate).unwrap().passed,
                    after >= before - TOLERANCE_M
                );
            }
        }
    }

    #[test]
    fn missing_or_invalid_grasp_evidence_cannot_pass() {
        let mut candidate = fixture();
        candidate.cases[0].placement.as_mut().unwrap().grasp = None;
        assert!(!compare(&fixture(), &candidate).unwrap().passed);
        assert!(compare(&candidate, &fixture()).is_err());
        for value in [-0.001, f64::NAN, f64::INFINITY] {
            for minimum_hold in [false, true] {
                let mut candidate = fixture();
                let grasp = candidate.cases[0]
                    .placement
                    .as_mut()
                    .unwrap()
                    .grasp
                    .as_mut()
                    .unwrap();
                if minimum_hold {
                    grasp.minimum_hold_lift_m = value;
                } else {
                    grasp.lift_m = value;
                }
                assert!(compare(&fixture(), &candidate).is_err());
            }
        }
    }

    #[test]
    fn incomplete_hold_evidence_fails_despite_successful_placement() {
        for (duration, samples, passes) in [
            (Some(2.0), Some(20), true),
            (Some(3.0), Some(30), true),
            (Some(2.0 - 1e-9), Some(20), false),
            (Some(2.0), Some(19), false),
            (Some(0.0), Some(0), false),
            (None, Some(20), false),
            (Some(2.0), None, false),
        ] {
            let mut candidate = fixture();
            let grasp = candidate.cases[0]
                .placement
                .as_mut()
                .unwrap()
                .grasp
                .as_mut()
                .unwrap();
            grasp.hold_duration_s = duration;
            grasp.contact_samples = samples;
            let result = compare(&fixture(), &candidate).unwrap();
            assert_eq!(result.passed, passes);
            if !passes {
                assert_eq!(result.regressions.len(), 1);
                assert!(result.regressions[0].contains("nominal: hold evidence incomplete"));
                assert!(compare(&candidate, &fixture()).is_err());
            }
        }
    }

    #[test]
    fn invalid_hold_duration_is_rejected() {
        for duration in [-1.0, f64::NAN, f64::INFINITY] {
            let mut candidate = fixture();
            candidate.cases[0]
                .placement
                .as_mut()
                .unwrap()
                .grasp
                .as_mut()
                .unwrap()
                .hold_duration_s = Some(duration);
            assert!(compare(&fixture(), &candidate).is_err());
        }
    }

    #[test]
    fn incompatible_or_unhealthy_reports_are_rejected() {
        let mut candidate = fixture();
        candidate.cases[1].name = "nominal".into();
        assert!(compare(&fixture(), &candidate).is_err());
        candidate = fixture();
        candidate.schema_version = 2;
        assert!(compare(&fixture(), &candidate).is_err());
        candidate = fixture();
        candidate.cases[0]
            .placement
            .as_mut()
            .unwrap()
            .planned_destination_m[0] = 1.0;
        assert!(compare(&fixture(), &candidate).is_err());
        candidate = fixture();
        candidate.passed = false;
        assert!(compare(&candidate, &fixture()).is_err());
    }
}
