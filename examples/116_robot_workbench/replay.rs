//! Bounded workbench input recording with per-batch state verification.
use crate::{
    drive::DriveCommand,
    project::{JointPosition, Project},
    simulation::Simulation,
};
use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, path::Path};

pub(crate) const MAX_RECORDING_BYTES: usize = 4 * 1024 * 1024 - 1024;
pub(crate) const MAX_FRAMES: usize = 2400;

#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct ReplayFrame {
    pub steps: u32,
    pub targets: BTreeMap<String, JointPosition>,
    pub drive: DriveCommand,
    pub sim_time_ticks: String,
    pub state_hash: String,
}

/// Replays commands, not rendered poses. Exact hashes apply to a compatible runtime.
#[derive(Clone, Debug, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Recording {
    pub schema_version: u32,
    pub project: Project,
    pub frames: Vec<ReplayFrame>,
}
impl Recording {
    pub(crate) fn from_json(bytes: &[u8]) -> Result<Self> {
        ensure!(
            bytes.len() <= MAX_RECORDING_BYTES,
            "recording exceeds 4 MiB"
        );
        let recording: Self = serde_json::from_slice(bytes)?;
        ensure!(
            recording.schema_version == 1,
            "unsupported recording schema"
        );
        ensure!(
            recording.frames.len() <= MAX_FRAMES,
            "too many recording frames"
        );
        recording.project.validate()?;
        Ok(recording)
    }
    pub(crate) fn verify(&self, asset_root: Option<&Path>) -> Result<Simulation> {
        // Also validate objects received via the HTTP enum, not only file readers.
        let checked = Self::from_json(&serde_json::to_vec(self)?)?;
        let mut simulation = Simulation::new(checked.project, asset_root)?;
        for (index, frame) in checked.frames.iter().enumerate() {
            simulation.set_targets(frame.targets.clone())?;
            if simulation.project.planar_base.is_some() {
                simulation.set_drive(frame.drive)?;
            } else {
                ensure!(
                    frame.drive == DriveCommand::default(),
                    "fixed-base replay has a drive command"
                );
            }
            simulation.step(frame.steps)?;
            ensure!(
                simulation.sim_time_ticks().to_string() == frame.sim_time_ticks
                    && format!("{:016x}", simulation.state_hash()) == frame.state_hash,
                "replay diverged at frame {index}"
            );
        }
        // Loading a recording is paused; continuing must explicitly set a drive command.
        if simulation.project.planar_base.is_some() {
            simulation.set_drive(DriveCommand::default())?;
        }
        Ok(simulation)
    }
}
