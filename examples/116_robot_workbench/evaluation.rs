//! Fixed, hardware-free regression catalog with replayable failure evidence.
use crate::{
    grasp::PickPlaceReport, presets, project::Project, simulation::Simulation, task, transfer,
};
use anyhow::{ensure, Result};
use serde::Serialize;
use std::path::Path;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
enum Outcome {
    Completed,
    Failed,
    Rejected,
    Error,
}

#[derive(Debug, Serialize)]
struct CaseReport {
    name: &'static str,
    expected: Outcome,
    expected_reason: Option<&'static str>,
    expected_phase: Option<task::Phase>,
    outcome: Outcome,
    expectation_met: bool,
    reason: Option<String>,
    failed_phase: Option<task::Phase>,
    completed_batches: u32,
    replay_verified: Option<bool>,
    replay_error: Option<String>,
    recording: Option<String>,
    placement: Option<PickPlaceReport>,
}

#[derive(Debug, Serialize)]
pub(crate) struct EvaluationReport {
    schema_version: u32,
    catalog: &'static str,
    engine_version: &'static str,
    pub passed: bool,
    completed_cases: usize,
    executed_cases: usize,
    rejected_cases: usize,
    error_cases: usize,
    completion_rate: f64,
    cases: Vec<CaseReport>,
}

fn summarize(cases: Vec<CaseReport>) -> EvaluationReport {
    let count = |outcome| cases.iter().filter(|case| case.outcome == outcome).count();
    let completed_cases = count(Outcome::Completed);
    let executed_cases = completed_cases + count(Outcome::Failed);
    EvaluationReport {
        schema_version: 1,
        catalog: "lekiwi-pick-place-v1",
        engine_version: env!("CARGO_PKG_VERSION"),
        passed: !cases.is_empty() && cases.iter().all(|case| case.expectation_met),
        completed_cases,
        executed_cases,
        rejected_cases: count(Outcome::Rejected),
        error_cases: count(Outcome::Error),
        completion_rate: if executed_cases == 0 {
            0.0
        } else {
            completed_cases as f64 / executed_cases as f64
        },
        cases,
    }
}

fn evaluate_case(
    name: &'static str,
    expected: Outcome,
    project: Project,
    root: &Path,
    output: &Path,
) -> Result<CaseReport> {
    let mut result = CaseReport {
        name,
        expected,
        expected_reason: None,
        expected_phase: None,
        outcome: Outcome::Error,
        expectation_met: false,
        reason: None,
        failed_phase: None,
        completed_batches: 0,
        replay_verified: None,
        replay_error: None,
        recording: None,
        placement: None,
    };
    // Invalid project data and asset/setup failures are never expected rejections.
    project.validate()?;
    if let Err(error) = transfer::plan(&project) {
        result.outcome = Outcome::Rejected;
        result.reason = Some(format!("{error:#}"));
        result.expectation_met = expected == Outcome::Rejected;
        return Ok(result);
    }
    let mut sim = Simulation::new(project, Some(root))?;
    let mut task = task::Task::new(&sim)?;
    while task.status == task::Status::Running {
        ensure!(
            task.completed_batches < task.total_batches,
            "task exceeded its batch budget"
        );
        task.advance(&mut sim)?;
    }
    result.outcome = if task.status == task::Status::Completed {
        Outcome::Completed
    } else {
        Outcome::Failed
    };
    result.completed_batches = task.completed_batches;
    result.failed_phase = (task.status == task::Status::Failed).then_some(task.phase);
    result.reason = task.error;
    result.placement = task.report;
    let filename = format!("{name}.recording.json");
    let bytes = serde_json::to_vec_pretty(sim.recording())?;
    std::fs::write(output.join(&filename), &bytes)?;
    result.recording = Some(filename);
    // Verify the serialized evidence, including recordings of failed tasks.
    match crate::replay::Recording::from_json(&bytes)?.verify(Some(root)) {
        Ok(_) => result.replay_verified = Some(true),
        Err(error) => {
            result.replay_verified = Some(false);
            result.replay_error = Some(format!("{error:#}"));
        }
    }
    result.expectation_met = result.outcome == expected && result.replay_verified == Some(true);
    Ok(result)
}

/// Writes a bounded six-case catalog into a new directory; never overwrites evidence.
pub(crate) fn run(output: &Path, root: &Path) -> Result<EvaluationReport> {
    std::fs::create_dir(output)?;
    let mut cases = Vec::new();
    for (name, destination, missed_block, expected) in [
        ("nominal", [0.60, 0.1845, 0.154], false, Outcome::Completed),
        ("diagonal", [0.68, 0.1845, 0.254], false, Outcome::Completed),
        ("reverse", [0.54, 0.1845, -0.246], false, Outcome::Completed),
        ("missed-block", [0.60, 0.1845, 0.154], true, Outcome::Failed),
        (
            "unequal-height",
            [0.60, 0.20, 0.154],
            false,
            Outcome::Rejected,
        ),
        (
            "out-of-range",
            [0.60, 0.1845, 0.60],
            false,
            Outcome::Rejected,
        ),
    ] {
        let mut project = presets::preset("lekiwi_pick_place")?;
        for object in &mut project.objects {
            if object.name == "place-table" {
                object.position_m = destination;
            }
            if missed_block && object.name == "target-block" {
                object.position_m[2] += 0.15;
            }
        }
        let mut result =
            evaluate_case(name, expected, project, root, output).unwrap_or_else(|error| {
                CaseReport {
                    name,
                    expected,
                    expected_reason: None,
                    expected_phase: None,
                    outcome: Outcome::Error,
                    expectation_met: false,
                    reason: Some(format!("{error:#}")),
                    failed_phase: None,
                    completed_batches: 0,
                    replay_verified: None,
                    replay_error: None,
                    recording: None,
                    placement: None,
                }
            });
        result.expected_reason = match name {
            "missed-block" => Some("block must contact both grasp pads"),
            "unequal-height" => Some("tabletop heights must match"),
            "out-of-range" => Some("distance 0.12–0.40 m"),
            _ => None,
        };
        if missed_block {
            result.expected_phase = Some(task::Phase::Close);
        }
        result.expectation_met &= result.expected_reason.is_none_or(|reason| {
            result
                .reason
                .as_deref()
                .is_some_and(|actual| actual.contains(reason))
        }) && result
            .expected_phase
            .is_none_or(|phase| result.failed_phase == Some(phase));
        cases.push(result);
    }
    let report = summarize(cases);
    std::fs::write(
        output.join("report.json"),
        serde_json::to_vec_pretty(&report)?,
    )?;
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn existing_output_directory_is_never_reused() {
        assert!(run(
            Path::new(env!("CARGO_MANIFEST_DIR")),
            Path::new("/missing-assets")
        )
        .is_err());
    }

    #[test]
    fn rejection_is_not_a_physical_success_or_a_replay() {
        let mut project = presets::preset("lekiwi_pick_place").unwrap();
        project
            .objects
            .iter_mut()
            .find(|o| o.name == "place-table")
            .unwrap()
            .position_m[1] += 0.1;
        // No world or output directory is needed for preflight rejection.
        let case = evaluate_case(
            "height",
            Outcome::Rejected,
            project,
            Path::new("/missing-assets"),
            Path::new("/missing-output"),
        )
        .unwrap();
        assert!(case.expectation_met);
        assert!(case.recording.is_none());
        assert!(case.replay_verified.is_none());
        let report = summarize(vec![case]);
        assert!(report.passed);
        assert_eq!(report.completed_cases, 0);
        assert_eq!(report.executed_cases, 0);
        assert_eq!(report.rejected_cases, 1);
        assert_eq!(report.completion_rate, 0.0);
        assert!(!summarize(Vec::new()).passed);
    }
}
