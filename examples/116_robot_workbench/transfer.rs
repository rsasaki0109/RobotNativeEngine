//! Bounded straight-line base transfer between equal-height tables.
use crate::{drive::DriveCommand, project::Project};
use anyhow::{ensure, Context, Result};
use serde::Serialize;

const BATCH_DURATION_S: f64 = 24.0 * 0.004_166_667;

#[derive(Clone, Copy, Debug, Serialize)]
pub(crate) struct TransferPlan {
    pub delta_x_z_m: [f64; 2],
    pub destination_m: [f64; 3],
    pub velocity: DriveCommand,
    pub batches: u32,
    pub duration_s: f64,
}

/// Computes only the carry translation; grasp, lowering and retreat stay scripted.
pub(crate) fn plan(project: &Project) -> Result<TransferPlan> {
    let pickup = project
        .objects
        .iter()
        .find(|o| o.name == "pickup-table")
        .context("pickup-table is required")?;
    let place = project
        .objects
        .iter()
        .find(|o| o.name == "place-table")
        .context("place-table is required")?;
    let block = project
        .objects
        .iter()
        .find(|o| o.name == "target-block")
        .context("target-block is required")?;
    ensure!(
        pickup.dynamics.is_none() && place.dynamics.is_none(),
        "transfer tables must be fixed"
    );
    let pickup_top = pickup.position_m[1] + pickup.size_m[1] * 0.5;
    let place_top = place.position_m[1] + place.size_m[1] * 0.5;
    ensure!(
        (pickup_top - place_top).abs() <= 1e-6,
        "destination not supported: tabletop heights must match"
    );
    let dx = place.position_m[0] - pickup.position_m[0];
    let dz = place.position_m[2] - pickup.position_m[2];
    let distance = dx.hypot(dz);
    ensure!(distance.is_finite() && (0.12..=0.4).contains(&distance)
        && dx.abs() <= 0.12 && dz.abs() >= 0.12,
        "destination not supported: distance 0.12–0.40 m, |X offset| ≤ 0.12 m, |Z offset| ≥ 0.12 m required");
    ensure!(
        dx.abs() >= (pickup.size_m[0] + place.size_m[0]) * 0.5 + 0.02
            || dz.abs() >= (pickup.size_m[2] + place.size_m[2]) * 0.5 + 0.02,
        "destination not supported: tables need at least 20 mm separation"
    );
    let footprint = block.size_m.iter().map(|v| v * v).sum::<f64>().sqrt() + 0.02;
    ensure!(
        place.size_m[0] >= footprint && place.size_m[2] >= footprint,
        "destination not supported: tabletop is too small for the block"
    );
    // Round duration up to whole simulation batches, then reduce speed so the
    // prescribed base endpoint is exact and never exceeds 0.025 m/s.
    let batches = (distance / (0.025 * BATCH_DURATION_S)).ceil() as u32;
    let duration_s = f64::from(batches) * BATCH_DURATION_S;
    let velocity = DriveCommand {
        vx_m_s: dx / duration_s,
        vz_m_s: dz / duration_s,
        ..DriveCommand::default()
    };
    velocity.validate()?;
    Ok(TransferPlan {
        delta_x_z_m: [dx, dz],
        destination_m: [
            place.position_m[0],
            place_top + block.size_m[1] * 0.5,
            place.position_m[2],
        ],
        velocity,
        batches,
        duration_s,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn duration_quantization_respects_speed_and_endpoint_in_both_directions() {
        for (x, z) in [(0.60, 0.154), (0.68, 0.254), (0.54, -0.246)] {
            let mut project = crate::presets::preset("lekiwi_pick_place").unwrap();
            let table = project
                .objects
                .iter_mut()
                .find(|o| o.name == "place-table")
                .unwrap();
            table.position_m[0] = x;
            table.position_m[2] = z;
            let plan = plan(&project).unwrap();
            assert!(plan.velocity.vx_m_s.hypot(plan.velocity.vz_m_s) <= 0.025 + 1e-12);
            assert!((plan.velocity.vx_m_s * plan.duration_s - plan.delta_x_z_m[0]).abs() < 1e-12);
            assert!((plan.velocity.vz_m_s * plan.duration_s - plan.delta_x_z_m[1]).abs() < 1e-12);
        }
    }
    #[test]
    fn unsupported_destination_is_rejected_before_motion() {
        for position in [
            [0.6, 0.2, 0.154],
            [0.6, 0.1845, 0.6],
            [0.6, 0.1845, -0.046],
            [0.9, 0.1845, 0.154],
        ] {
            let mut project = crate::presets::preset("lekiwi_pick_place").unwrap();
            project
                .objects
                .iter_mut()
                .find(|o| o.name == "place-table")
                .unwrap()
                .position_m = position;
            assert!(plan(&project).is_err());
        }
    }
}
