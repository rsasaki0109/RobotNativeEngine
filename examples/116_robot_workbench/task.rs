//! Resumable, fixed-step pick/place execution. No wall-clock scheduling lives here.
use crate::{
    drive::DriveCommand,
    grasp::{self, GraspReport, PickPlaceReport},
    simulation::Simulation,
};
use anyhow::{ensure, Context, Result};
use rne_core::SimDuration;
use rne_math::Vec3;
use serde::Serialize;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum Phase {
    Open,
    Roll,
    Approach,
    Close,
    Lift,
    Hold,
    Carry,
    Lower,
    Release,
    Retreat,
    Settle,
    Verify,
}
const PHASES: [Phase; 12] = [
    Phase::Open,
    Phase::Roll,
    Phase::Approach,
    Phase::Close,
    Phase::Lift,
    Phase::Hold,
    Phase::Carry,
    Phase::Lower,
    Phase::Release,
    Phase::Retreat,
    Phase::Settle,
    Phase::Verify,
];
impl Phase {
    fn batches(self) -> u32 {
        match self {
            Self::Approach | Self::Retreat => 40,
            Self::Carry => 80,
            Self::Settle => 10,
            _ => 20,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub(crate) enum Status {
    Running,
    Completed,
    Failed,
}

/// Progress and failure are retained until reset/cancel, even when execution stops.
#[derive(Debug, Serialize)]
pub(crate) struct Task {
    pub phase: Phase,
    pub status: Status,
    pub phase_batch: u32,
    pub phase_batches: u32,
    pub completed_batches: u32,
    pub total_batches: u32,
    pub transfer: crate::transfer::TransferPlan,
    pub error: Option<String>,
    pub report: Option<PickPlaceReport>,
    #[serde(skip)]
    initial_y: f64,
    #[serde(skip)]
    minimum_lift_m: f64,
    #[serde(skip)]
    hold_ticks: u64,
    #[serde(skip)]
    hold_contact_samples: usize,
    #[serde(skip)]
    grasp: Option<GraspReport>,
    #[serde(skip)]
    settled: Vec3,
    #[serde(skip)]
    maximum_drift_m: f64,
}
impl Task {
    pub(crate) fn new(sim: &Simulation) -> Result<Self> {
        ensure!(
            sim.sim_time_ticks() == 0,
            "pick/place requires a fresh scene"
        );
        ensure!(
            sim.project.planar_base.is_some(),
            "pick/place requires a mobile base"
        );
        let table = sim
            .project
            .objects
            .iter()
            .find(|o| o.name == "place-table")
            .context("pick/place requires place-table")?;
        ensure!(table.dynamics.is_none(), "place-table must be fixed");
        ensure!(
            sim.project
                .objects
                .iter()
                .any(|o| o.name == "target-block" && o.dynamics.is_some()),
            "pick/place requires a dynamic target-block"
        );
        for name in ["gripper", "wrist_roll", "shoulder_lift"] {
            ensure!(
                sim.project.targets.contains_key(name),
                "missing task joint {name}"
            );
        }
        let transfer = crate::transfer::plan(&sim.project)?;
        Ok(Self {
            phase: Phase::Open,
            status: Status::Running,
            phase_batch: 0,
            phase_batches: 20,
            completed_batches: 0,
            total_batches: 250 + transfer.batches,
            transfer,
            error: None,
            report: None,
            initial_y: 0.0,
            minimum_lift_m: f64::INFINITY,
            hold_ticks: 0,
            hold_contact_samples: 0,
            grasp: None,
            settled: Vec3::ZERO,
            maximum_drift_m: 0.0,
        })
    }

    /// Advances at most 24 physics steps (about 0.1 s); failed tasks retain evidence.
    pub(crate) fn advance(&mut self, sim: &mut Simulation) -> Result<()> {
        ensure!(
            self.status == Status::Running,
            "task is terminal; reset or cancel it"
        );
        if let Err(error) = self.advance_inner(sim) {
            sim.set_drive(DriveCommand::default())?;
            self.status = Status::Failed;
            self.error = Some(format!("{:?}: {error:#}", self.phase));
        }
        Ok(())
    }

    fn enter(&self, sim: &mut Simulation) -> Result<()> {
        match self.phase {
            Phase::Open | Phase::Release => grasp::target(sim, "gripper", 0.8)?,
            Phase::Roll => grasp::target(sim, "wrist_roll", 1.62)?,
            Phase::Close => grasp::target(sim, "gripper", -0.1)?,
            Phase::Lift => grasp::target(sim, "shoulder_lift", -0.3)?,
            Phase::Lower => grasp::target(sim, "shoulder_lift", 0.0)?,
            Phase::Approach => sim.set_drive(DriveCommand {
                vx_m_s: 0.05,
                ..DriveCommand::default()
            })?,
            Phase::Carry => sim.set_drive(self.transfer.velocity)?,
            Phase::Retreat => sim.set_drive(DriveCommand {
                vx_m_s: -0.025,
                ..DriveCommand::default()
            })?,
            _ => {}
        }
        Ok(())
    }

    fn advance_inner(&mut self, sim: &mut Simulation) -> Result<()> {
        if self.phase_batch == 0 {
            self.enter(sim)?;
        }
        let before_ticks = sim.sim_time_ticks();
        sim.step(24)?;
        self.phase_batch += 1;
        self.completed_batches += 1;
        let end = self.phase_batch == self.phase_batches;
        match self.phase {
            Phase::Open if end => self.initial_y = sim.object_position("target-block")?.y,
            Phase::Close if end => grasp::check_pads(sim)?,
            Phase::Hold => {
                grasp::check_pads(sim)?;
                let lift = sim.object_position("target-block")?.y - self.initial_y;
                ensure!(lift >= 0.05, "block lift below 50 mm: {lift}");
                self.minimum_lift_m = self.minimum_lift_m.min(lift);
                self.hold_ticks += sim.sim_time_ticks() - before_ticks;
                self.hold_contact_samples += 1;
                if end {
                    self.grasp = Some(GraspReport {
                        lift_m: lift,
                        minimum_hold_lift_m: self.minimum_lift_m,
                        hold_duration_s: SimDuration::from_ticks(self.hold_ticks)
                            .as_seconds()
                            .value(),
                        contact_samples: self.hold_contact_samples,
                    });
                }
            }
            Phase::Carry => grasp::check_pads(sim).context("lost grasp during transport")?,
            Phase::Lower if end => {
                ensure!(
                    sim.objects_in_contact("target-block", "place-table")?,
                    "block is not supported by place-table before release"
                );
                grasp::check_pads(sim).context("lost grasp while lowering")?;
            }
            Phase::Settle if end => self.settled = sim.object_position("target-block")?,
            Phase::Verify => {
                grasp::check_placement(sim)?;
                let position = sim.object_position("target-block")?;
                ensure!(
                    (position.x - self.transfer.destination_m[0])
                        .hypot(position.z - self.transfer.destination_m[2])
                        <= 0.015,
                    "placed block is more than 15 mm from the destination center"
                );
                let drift = (sim.object_position("target-block")? - self.settled).length();
                self.maximum_drift_m = self.maximum_drift_m.max(drift);
                ensure!(drift < 0.001, "placed block is not stable: drift {drift} m");
            }
            _ => {}
        }
        if end {
            sim.set_drive(DriveCommand::default())?;
            if self.phase == Phase::Verify {
                self.report = Some(PickPlaceReport {
                    grasp: self.grasp.take().context("missing grasp result")?,
                    final_position_m: sim.object_position("target-block")?.to_array(),
                    transport_contact_samples: self.transfer.batches as usize,
                    planned_destination_m: self.transfer.destination_m,
                    placement_error_x_z_m: {
                        let position = sim.object_position("target-block")?;
                        [
                            position.x - self.transfer.destination_m[0],
                            position.z - self.transfer.destination_m[2],
                        ]
                    },
                    stable_duration_s: 2.000_000_16,
                    placement_samples: 20,
                    maximum_settled_drift_m: self.maximum_drift_m,
                });
                self.status = Status::Completed;
            } else {
                let index = PHASES
                    .iter()
                    .position(|p| *p == self.phase)
                    .context("unknown phase")?;
                self.phase = PHASES[index + 1];
                self.phase_batch = 0;
                self.phase_batches = if self.phase == Phase::Carry {
                    self.transfer.batches
                } else {
                    self.phase.batches()
                };
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    #[test]
    fn progress_can_be_observed_and_resumed_without_advancing_time() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
        let mut sim = Simulation::new(
            crate::presets::preset("lekiwi_pick_place").unwrap(),
            Some(&root),
        )
        .unwrap();
        let mut task = Task::new(&sim).unwrap();
        let mut visited = Vec::new();
        while task.status == Status::Running {
            if visited.last() != Some(&task.phase) {
                visited.push(task.phase);
            }
            task.advance(&mut sim).unwrap();
            if task.completed_batches == 57 {
                let ticks = sim.sim_time_ticks();
                let hash = sim.state_hash();
                for _ in 0..3 {
                    serde_json::to_string(&task).unwrap();
                    sim.object_readings().unwrap();
                }
                assert_eq!(sim.sim_time_ticks(), ticks);
                assert_eq!(sim.state_hash(), hash);
                assert_eq!(task.phase, Phase::Approach);
            }
        }
        assert_eq!(visited, PHASES);
        assert_eq!(task.status, Status::Completed);
        assert_eq!(task.completed_batches, 330);
        assert!(task.report.is_some());
        let grasp = &task.report.as_ref().unwrap().grasp;
        assert_eq!(grasp.contact_samples, 20);
        assert_eq!(grasp.hold_duration_s, 2.000_000_16);
        let ticks = sim.sim_time_ticks();
        assert!(task.advance(&mut sim).is_err());
        assert_eq!(sim.sim_time_ticks(), ticks);
        let replay = sim.recording().verify(Some(&root)).unwrap();
        assert_eq!(replay.state_hash(), sim.state_hash());
    }

    #[test]
    fn shortened_hold_reports_only_observed_batches_and_simulation_time() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
        let mut sim = Simulation::new(
            crate::presets::preset("lekiwi_pick_place").unwrap(),
            Some(&root),
        )
        .unwrap();
        let mut task = Task::new(&sim).unwrap();
        while task.phase != Phase::Hold {
            task.advance(&mut sim).unwrap();
            assert_eq!(task.status, Status::Running, "{:?}", task.error);
        }
        // A shortened observation window must not claim the standard 2 s / 20 samples.
        task.phase_batches = 5;
        let start_ticks = sim.sim_time_ticks();
        for sample in 1..=5 {
            task.advance(&mut sim).unwrap();
            assert_eq!(task.status, Status::Running, "{:?}", task.error);
            assert_eq!(task.hold_contact_samples, sample);
            let ticks = sim.sim_time_ticks();
            let held_ticks = task.hold_ticks;
            serde_json::to_string(&task).unwrap();
            sim.object_readings().unwrap();
            assert_eq!(sim.sim_time_ticks(), ticks);
            assert_eq!(task.hold_ticks, held_ticks);
        }
        assert_eq!(task.phase, Phase::Carry);
        let report = task.grasp.as_ref().unwrap();
        assert_eq!(report.contact_samples, 5);
        assert_eq!(
            report.hold_duration_s,
            SimDuration::from_ticks(sim.sim_time_ticks() - start_ticks)
                .as_seconds()
                .value()
        );
        assert!(report.hold_duration_s < 1.0);
    }

    #[test]
    fn failed_hold_observation_does_not_count_as_successful_evidence() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
        let mut sim = Simulation::new(
            crate::presets::preset("lekiwi_pick_place").unwrap(),
            Some(&root),
        )
        .unwrap();
        let mut task = Task::new(&sim).unwrap();
        while task.phase != Phase::Hold {
            task.advance(&mut sim).unwrap();
            assert_eq!(task.status, Status::Running, "{:?}", task.error);
        }
        task.advance(&mut sim).unwrap();
        assert_eq!(task.hold_contact_samples, 1);
        let held_ticks = task.hold_ticks;
        // Force the next measured lift below the threshold without changing physics.
        task.initial_y += 1.0;
        task.advance(&mut sim).unwrap();
        assert_eq!(task.status, Status::Failed);
        assert!(task.error.as_ref().unwrap().contains("below 50 mm"));
        assert_eq!(task.hold_contact_samples, 1);
        assert_eq!(task.hold_ticks, held_ticks);
        assert!(task.grasp.is_none());
        assert!(task.report.is_none());
    }

    #[test]
    fn failed_phase_is_retained_and_cannot_continue() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
        let mut project = crate::presets::preset("lekiwi_pick_place").unwrap();
        project
            .objects
            .iter_mut()
            .find(|o| o.name == "target-block")
            .unwrap()
            .position_m[2] = -0.25;
        let mut sim = Simulation::new(project, Some(&root)).unwrap();
        let mut task = Task::new(&sim).unwrap();
        while task.status == Status::Running {
            task.advance(&mut sim).unwrap();
        }
        assert_eq!(task.status, Status::Failed);
        assert_eq!(task.phase, Phase::Close);
        assert!(task.error.as_ref().unwrap().contains("both grasp pads"));
        assert_eq!(sim.base_reading().1, DriveCommand::default());
        let ticks = sim.sim_time_ticks();
        assert!(task.advance(&mut sim).is_err());
        assert_eq!(sim.sim_time_ticks(), ticks);
        let replay = sim.recording().verify(Some(&root)).unwrap();
        assert_eq!(replay.state_hash(), sim.state_hash());
    }
    #[test]
    fn edited_destinations_are_placed_and_replayed_in_both_directions() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
        for (x, z) in [(0.68, 0.254), (0.54, -0.246)] {
            let mut project = crate::presets::preset("lekiwi_pick_place").unwrap();
            let table = project
                .objects
                .iter_mut()
                .find(|o| o.name == "place-table")
                .unwrap();
            table.position_m[0] = x;
            table.position_m[2] = z;
            let mut sim = Simulation::new(project, Some(&root)).unwrap();
            let mut task = Task::new(&sim).unwrap();
            assert_ne!(task.total_batches, 330);
            while task.status == Status::Running {
                task.advance(&mut sim).unwrap();
            }
            assert_eq!(task.status, Status::Completed, "{:?}", task.error);
            assert_eq!(task.completed_batches, task.total_batches);
            let report = task.report.unwrap();
            assert_eq!(
                report.transport_contact_samples,
                task.transfer.batches as usize
            );
            assert!((report.final_position_m[0] - x).hypot(report.final_position_m[2] - z) < 0.015);
            let replay = sim.recording().verify(Some(&root)).unwrap();
            assert_eq!(replay.state_hash(), sim.state_hash());
        }
    }
}
