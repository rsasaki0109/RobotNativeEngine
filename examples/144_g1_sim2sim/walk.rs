//! Runs the unitree_rl_gym G1 walking policy on RNE through `rne_ai`.
//!
//! [`rne_ai::UnitreeRlGymWalker`] implements the upstream MuJoCo deployment:
//! PD torque recomputed every 2 ms physics step and clamped to the URDF effort
//! limits, with the policy every tenth step. This module maps the example's
//! plant options onto it and computes the walking metrics shared with
//! `metrics.py`. Poses are in the policy's Z-up frame.

use rne_ai::{
    unitree_g1_12dof_walk_scene_path, LstmPolicy, PolicyWalkPlant, PolicyWalkStep,
    UnitreeRlGymWalker, UrdfSceneSim,
};
use serde_json::{json, Value};

use crate::policy::Result;

/// Pelvis height below which a run counts as a fall, in metres.
pub(crate) const FALL_HEIGHT_M: f64 = 0.55;

/// One 50 Hz policy step in the policy's Z-up world.
pub(crate) type StepRecord = PolicyWalkStep;

/// Plant settings that make RNE's model match unitree_rl_gym's MJCF.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct PlantConfig {
    /// Constraint-solver iterations per physics step.
    pub(crate) solver_iterations: usize,
    /// Reflected rotor inertia per joint, MJCF `armature`.
    pub(crate) armature_kg_m2: f64,
    /// Joint viscous damping, MJCF `damping`.
    pub(crate) joint_damping_nm_s_per_rad: f64,
    /// Joint dry friction, MJCF `frictionloss` (regularized in RNE).
    pub(crate) joint_friction_nm: f64,
    /// Floor and foot friction coefficient (MuJoCo default 1).
    pub(crate) floor_friction: f64,
}

impl Default for PlantConfig {
    fn default() -> Self {
        let matched = PolicyWalkPlant::default();
        Self {
            solver_iterations: 4,
            armature_kg_m2: matched.armature_kg_m2,
            joint_damping_nm_s_per_rad: matched.joint_damping_nm_s_per_rad,
            joint_friction_nm: matched.joint_friction_nm,
            floor_friction: matched.floor_friction,
        }
    }
}

/// Loads the 12-DoF scene and configures the walker for `plant`.
pub(crate) fn walker(policy: LstmPolicy, plant: PlantConfig) -> Result<UnitreeRlGymWalker> {
    let sim = UrdfSceneSim::from_scene_path_with_solver_iterations(
        &unitree_g1_12dof_walk_scene_path(),
        plant.solver_iterations,
    )?;
    Ok(UnitreeRlGymWalker::new(
        sim,
        policy,
        PolicyWalkPlant {
            armature_kg_m2: plant.armature_kg_m2,
            joint_damping_nm_s_per_rad: plant.joint_damping_nm_s_per_rad,
            joint_friction_nm: plant.joint_friction_nm,
            floor_friction: plant.floor_friction,
            ..PolicyWalkPlant::default()
        },
    )?)
}

fn yaw(q: [f64; 4]) -> f64 {
    let [w, x, y, z] = q;
    (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z))
}

fn round4(value: f64) -> f64 {
    (value * 1.0e4).round() / 1.0e4
}

/// Walking metrics; keep identical to `metrics.py`.
pub(crate) fn summarize(records: &[StepRecord], command: [f64; 3]) -> Value {
    let heights: Vec<f64> = records.iter().map(|r| r.base_position_m[2]).collect();
    let fall_time_s = records
        .iter()
        .zip(&heights)
        .find(|(_, height)| **height < FALL_HEIGHT_M)
        .map(|(record, _)| record.time_s);
    let half = records.len() / 2;
    let (mut forward, mut lateral, mut turn) = (Vec::new(), Vec::new(), Vec::new());
    for index in half.max(1)..records.len() {
        let (previous, current) = (&records[index - 1], &records[index]);
        let dt = current.time_s - previous.time_s;
        let dx = current.base_position_m[0] - previous.base_position_m[0];
        let dy = current.base_position_m[1] - previous.base_position_m[1];
        let heading = yaw(previous.base_quaternion_wxyz);
        forward.push((heading.cos() * dx + heading.sin() * dy) / dt);
        lateral.push((-heading.sin() * dx + heading.cos() * dy) / dt);
        let delta = yaw(current.base_quaternion_wxyz) - heading;
        let wrapped = delta - std::f64::consts::TAU * (delta / std::f64::consts::TAU).round();
        turn.push(wrapped / dt);
    }
    let mean = |values: &[f64]| values.iter().sum::<f64>() / values.len().max(1) as f64;
    let last = records
        .last()
        .map(|r| r.base_position_m)
        .unwrap_or_default();
    json!({
        "fell": fall_time_s.is_some(), "fall_time_s": fall_time_s,
        "final_base_position_m": last.map(round4),
        "min_base_height_m": round4(heights.iter().copied().fold(f64::INFINITY, f64::min)),
        "mean_base_height_m": round4(mean(&heights)),
        "second_half_forward_speed_m_s": round4(mean(&forward)),
        "second_half_lateral_speed_m_s": round4(mean(&lateral)),
        "second_half_yaw_rate_rad_s": round4(mean(&turn)),
        "forward_speed_error_m_s": round4(mean(&forward) - command[0]),
        "yaw_rate_error_rad_s": round4(mean(&turn) - command[2]),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn walker() -> UnitreeRlGymWalker {
        super::walker(
            crate::policy::load().expect("policy"),
            PlantConfig::default(),
        )
        .expect("walker")
    }

    fn walk(seconds: f64, command: [f64; 3]) -> Vec<StepRecord> {
        let mut walker = walker();
        (0..(seconds / 0.02).round() as usize)
            .map(|_| walker.step(command).expect("step"))
            .collect()
    }

    #[test]
    fn plant_mass_and_effort_limits_match_the_mjcf() {
        let walker = walker();
        // The MJCF's 12-DoF G1 weighs 32.107 kg. Merging the upper body into
        // the pelvis must keep it, and no frame may take the importer's 1 kg
        // default for links without <inertial>.
        let (_, mass_kg) = walker.sim().dynamic_center_of_mass_m().expect("mass");
        assert!((mass_kg - 32.107).abs() < 1.0e-3, "mass {mass_kg}");
        assert_eq!(
            walker.effort_limits_nm(),
            [88.0, 139.0, 88.0, 139.0, 50.0, 50.0, 88.0, 139.0, 88.0, 139.0, 50.0, 50.0]
        );
    }

    #[test]
    fn replay_is_bit_exact() {
        let first = walk(1.0, [0.5, 0.0, 0.0]);
        let second = walk(1.0, [0.5, 0.0, 0.0]);
        assert_eq!(first.len(), 50);
        for (a, b) in first.iter().zip(&second) {
            assert_eq!(a.time_s.to_bits(), b.time_s.to_bits());
            let bits = |action: &[f32]| action.iter().map(|v| v.to_bits()).collect::<Vec<_>>();
            assert_eq!(bits(&a.action), bits(&b.action));
            assert_eq!(
                a.base_position_m.map(f64::to_bits),
                b.base_position_m.map(f64::to_bits)
            );
            assert_eq!(
                a.base_quaternion_wxyz.map(f64::to_bits),
                b.base_quaternion_wxyz.map(f64::to_bits)
            );
        }
    }

    #[test]
    fn walks_like_the_mujoco_reference() {
        let reference: Value =
            serde_json::from_str(include_str!("fixtures/mujoco_reference.json")).expect("fixture");
        let mujoco = &reference["runs"][0];
        let command: [f64; 3] = serde_json::from_value(mujoco["command"].clone()).unwrap();
        assert_eq!(command, [0.5, 0.0, 0.0]);
        let summary = summarize(&walk(10.0, command), command);
        let field = |value: &Value, key: &str| value[key].as_f64().expect(key);
        assert_eq!(summary["fell"], false);
        for (key, tolerance) in [
            ("second_half_forward_speed_m_s", 0.04),
            ("second_half_yaw_rate_rad_s", 0.03),
            ("mean_base_height_m", 0.005),
        ] {
            let (rne, reference) = (field(&summary, key), field(&mujoco["summary"], key));
            assert!(
                (rne - reference).abs() < tolerance,
                "{key}: RNE {rne}, MuJoCo {reference}"
            );
        }
    }
}
