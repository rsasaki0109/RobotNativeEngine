//! Scripted SO-101 contact grasp on a freely simulated box.
use crate::{drive::DriveCommand, project::JointPosition, simulation::Simulation};
use anyhow::{ensure, Context, Result};
use serde::Serialize;

#[derive(Debug, Serialize)]
pub(crate) struct GraspReport {
    pub lift_m: f64,
    pub minimum_hold_lift_m: f64,
    pub hold_duration_s: f64,
    pub contact_samples: usize,
}

pub(crate) fn target(sim: &mut Simulation, name: &str, position_rad: f64) -> Result<()> {
    let mut targets = sim.project.targets.clone();
    targets.insert(name.into(), JointPosition::Revolute { position_rad });
    sim.set_targets(targets)
}
fn advance(sim: &mut Simulation, seconds: usize) -> Result<()> {
    for _ in 0..seconds {
        sim.step(240)?;
    }
    Ok(())
}
pub(crate) fn check_pads(sim: &Simulation) -> Result<()> {
    let contacts = sim.object_contacts("target-block")?;
    ensure!(
        ["gripper_link", "moving_jaw_so101_v1_link"]
            .iter()
            .all(|name| contacts.iter().any(|contact| contact == name)),
        "block must contact both grasp pads; actual contacts: {contacts:?}"
    );
    Ok(())
}

/// Drive, close, lift, then measure two seconds of sustained contact support.
/// Only motor targets and base velocity are changed; no object pose is prescribed.
pub(crate) fn run(sim: &mut Simulation) -> Result<GraspReport> {
    target(sim, "gripper", 0.8)?;
    advance(sim, 2)?;
    let initial_y = sim.object_position("target-block")?.y;
    target(sim, "wrist_roll", 1.62)?;
    advance(sim, 2)?;
    sim.set_drive(DriveCommand {
        vx_m_s: 0.05,
        ..DriveCommand::default()
    })?;
    advance(sim, 4)?;
    sim.set_drive(DriveCommand::default())?;
    target(sim, "gripper", -0.1)?;
    advance(sim, 2)?;
    check_pads(sim)?;
    target(sim, "shoulder_lift", -0.3)?;
    advance(sim, 2)?;
    let mut minimum_hold_lift_m = f64::INFINITY;
    for _ in 0..20 {
        sim.step(24)?;
        check_pads(sim)?;
        let lift = sim.object_position("target-block")?.y - initial_y;
        ensure!(lift >= 0.05, "block lift below 50 mm: {lift}");
        minimum_hold_lift_m = minimum_hold_lift_m.min(lift);
    }
    Ok(GraspReport {
        lift_m: sim.object_position("target-block")?.y - initial_y,
        minimum_hold_lift_m,
        hold_duration_s: 480.0 * 0.004_166_667,
        contact_samples: 20,
    })
}

/// Measured result of the fixed-scene transfer and supported release.
#[derive(Debug, Serialize)]
pub(crate) struct PickPlaceReport {
    pub grasp: GraspReport,
    pub final_position_m: [f64; 3],
    pub transport_contact_samples: usize,
    pub planned_destination_m: [f64; 3],
    pub placement_error_x_z_m: [f64; 2],
    pub stable_duration_s: f64,
    pub placement_samples: usize,
    pub maximum_settled_drift_m: f64,
}

/// Fixed motor/base script; placement success is measured from simulated contacts.
pub(crate) fn pick_place(sim: &mut Simulation) -> Result<PickPlaceReport> {
    let mut task = crate::task::Task::new(sim)?;
    while task.status == crate::task::Status::Running {
        task.advance(sim)?;
    }
    task.report
        .context(task.error.unwrap_or_else(|| "missing task result".into()))
}

pub(crate) fn check_placement(sim: &Simulation) -> Result<()> {
    ensure!(
        sim.objects_in_contact("target-block", "place-table")?,
        "block is not supported by place-table"
    );
    ensure!(
        sim.object_contacts("target-block")?.is_empty(),
        "robot still contacts placed block"
    );
    let (block_min, block_max) = sim.object_bounds("target-block")?;
    let (table_min, table_max) = sim.object_bounds("place-table")?;
    ensure!(
        block_min.x >= table_min.x
            && block_max.x <= table_max.x
            && block_min.z >= table_min.z
            && block_max.z <= table_max.z
            && (block_min.y - table_max.y).abs() <= 0.005,
        "block is outside the destination tabletop"
    );
    let (linear_m_s, angular_rad_s) = sim.object_speeds("target-block")?;
    ensure!(
        linear_m_s < 0.005 && angular_rad_s < 0.05,
        "placed block is moving: {linear_m_s} m/s, {angular_rad_s} rad/s"
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    #[test]
    fn contact_grasp_holds_replays_and_releases() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
        let mut sim =
            Simulation::new(crate::presets::preset("lekiwi_grasp").unwrap(), Some(&root)).unwrap();
        let report = run(&mut sim).unwrap();
        assert!(report.minimum_hold_lift_m > 0.05);
        let readings = sim.object_readings().unwrap();
        assert!(readings
            .iter()
            .any(|o| o.name == "target-block" && o.dynamic && o.height_gain_m > 0.05));
        let replay = sim.recording().verify(Some(&root)).unwrap();
        assert_eq!(replay.state_hash(), sim.state_hash());
        let held_y = sim.object_position("target-block").unwrap().y;
        target(&mut sim, "gripper", 0.8).unwrap();
        advance(&mut sim, 2).unwrap();
        assert!(sim.object_position("target-block").unwrap().y < held_y - 0.05);
        assert!(sim.object_contacts("target-block").unwrap().is_empty());
    }

    #[test]
    fn missed_block_is_not_reported_as_grasped() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
        let mut project = crate::presets::preset("lekiwi_grasp").unwrap();
        project
            .objects
            .iter_mut()
            .find(|o| o.name == "target-block")
            .unwrap()
            .position_m[2] = -0.25;
        let mut sim = Simulation::new(project, Some(&root)).unwrap();
        assert!(run(&mut sim)
            .unwrap_err()
            .to_string()
            .contains("both grasp pads"));
    }
    #[test]
    fn pick_place_releases_on_destination_and_replays() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
        let mut sim = Simulation::new(
            crate::presets::preset("lekiwi_pick_place").unwrap(),
            Some(&root),
        )
        .unwrap();
        let report = pick_place(&mut sim).unwrap();
        assert_eq!(report.transport_contact_samples, 80);
        assert_eq!(report.placement_samples, 20);
        assert!((report.final_position_m[2] - 0.154).abs() < 0.01);
        assert_eq!(sim.base_reading().1, DriveCommand::default());
        assert!((sim.base_reading().0[0] - 0.1).abs() < 1e-5);
        let replay = sim.recording().verify(Some(&root)).unwrap();
        assert_eq!(replay.state_hash(), sim.state_hash());
        check_placement(&replay).unwrap();
        advance(&mut sim, 3).unwrap();
        check_placement(&sim).unwrap();
    }

    #[test]
    fn displaced_destination_is_not_reported_as_placed() {
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
        let mut project = crate::presets::preset("lekiwi_pick_place").unwrap();
        project
            .objects
            .iter_mut()
            .find(|o| o.name == "place-table")
            .unwrap()
            .position_m[2] += 0.3;
        let mut sim = Simulation::new(project, Some(&root)).unwrap();
        assert!(pick_place(&mut sim)
            .unwrap_err()
            .to_string()
            .contains("not supported"));
    }
}
