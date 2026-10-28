//! Explicit sampled wheel-speed feedback producing direct joint torques.
use crate::arena::{Action, Arena, Config, Observation};
use anyhow::{ensure, Context, Result};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, path::Path};

pub(crate) const WHEELS: [&str; 3] = [
    "ST3215_Servo_Motor-v1-2_Revolute-60",
    "ST3215_Servo_Motor-v1-1_Revolute-62",
    "ST3215_Servo_Motor-v1_Revolute-64",
];
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Episode {
    pub config: Config,
    pub gain_nm_s_per_rad: f64,
    #[serde(default)]
    pub integral_gain_nm_per_rad: f64,
    #[serde(default)]
    pub arm_hold: BTreeMap<String, BTreeMap<String, ArmHold>>,
    pub phases: Vec<Phase>,
}
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Phase {
    pub steps: u32,
    /// Missing agents coast at zero torque. Explicit zero speeds request braking.
    pub wheel_speeds_rad_s: BTreeMap<String, [f64; 3]>,
}
/// Optional persistent arm posture control; outputs direct torque, not a servo motor.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct ArmHold {
    pub position_rad: f64,
    pub stiffness_nm_per_rad: f64,
    pub damping_nm_s_per_rad: f64,
    pub effort_limit_nm: f64,
}
#[derive(Debug, Serialize)]
pub(crate) struct Sample {
    pub phase: usize,
    pub phase_step: u32,
    pub observation: Observation,
}

/// Run a bounded shared-world episode using per-step PI wheel torque feedback.
pub(crate) fn run(input: &Episode, root: &Path) -> Result<Vec<Sample>> {
    ensure!(
        input.gain_nm_s_per_rad.is_finite() && (0.0..=1.0).contains(&input.gain_nm_s_per_rad),
        "gain must be finite and in 0..=1 N m s/rad"
    );
    ensure!(
        !input.phases.is_empty() && input.phases.len() <= 1000,
        "requires 1..=1000 phases"
    );
    ensure!(
        input.phases.iter().all(|p| (1..=2400).contains(&p.steps))
            && input.phases.iter().map(|p| u64::from(p.steps)).sum::<u64>() <= 24000,
        "invalid phase duration or episode exceeds 24000 steps"
    );
    ensure!(
        input.integral_gain_nm_per_rad.is_finite()
            && (0.0..=1.0).contains(&input.integral_gain_nm_per_rad),
        "integral gain must be finite and in 0..=1 N m/rad"
    );
    let mut sim = Arena::new(&input.config, root)?;
    // Validate all IDs and setpoints before any phase advances the world.
    for phase in &input.phases {
        for (id, speeds) in &phase.wheel_speeds_rad_s {
            ensure!(
                speeds.iter().all(|s| s.is_finite() && s.abs() <= 20.0),
                "wheel speed must be finite and within 20 rad/s"
            );
            for name in WHEELS {
                sim.joint_feedback(id, name)?;
            }
        }
    }
    let catalog = crate::presets::preset("lekiwi_grasp")?.joint_catalog()?;
    for (id, joints) in &input.arm_hold {
        sim.joint_feedback(id, WHEELS[0])?;
        for (name, hold) in joints {
            ensure!(
                !WHEELS.contains(&name.as_str()),
                "arm hold cannot control a drive wheel"
            );
            let joint = catalog
                .iter()
                .find(|j| &j.name == name)
                .context("unknown arm joint")?;
            let (_, _, cap) = sim.joint_feedback(id, name)?;
            ensure!(
                hold.position_rad.is_finite()
                    && (joint.lower..=joint.upper).contains(&hold.position_rad),
                "arm target outside joint limits"
            );
            ensure!(
                hold.stiffness_nm_per_rad.is_finite()
                    && (0.0..=20.0).contains(&hold.stiffness_nm_per_rad)
                    && hold.damping_nm_s_per_rad.is_finite()
                    && (0.0..=1.0).contains(&hold.damping_nm_s_per_rad)
                    && hold.effort_limit_nm.is_finite()
                    && (0.0..=cap).contains(&hold.effort_limit_nm),
                "invalid arm gains or effort limit"
            );
        }
    }
    let mut samples = Vec::new();
    for (index, phase) in input.phases.iter().enumerate() {
        // Explicit phase changes reset integral memory, including drive-to-brake.
        let mut integral = BTreeMap::<(String, String), f64>::new();
        // Emit 24-step observation windows, retaining every contact impulse.
        let mut contacts = BTreeMap::<(String, String), f64>::new();
        for step in 1..=phase.steps {
            let mut torques_nm = BTreeMap::new();
            for (id, targets) in &phase.wheel_speeds_rad_s {
                let mut joints = BTreeMap::new();
                for (name, target) in WHEELS.into_iter().zip(targets) {
                    let (_, measured, cap) = sim.joint_feedback(id, name)?;
                    let error = target - measured;
                    let stored = integral.entry((id.clone(), name.into())).or_default();
                    let candidate = (*stored
                        + input.integral_gain_nm_per_rad
                            * error
                            * (crate::arena::STEP_TICKS as f64 / 1e9))
                        .clamp(-cap, cap);
                    let raw = input.gain_nm_s_per_rad * error + candidate;
                    // Conditional integration permits unwinding but prevents growth
                    // when error would drive an already saturated actuator harder.
                    if (raw <= cap || error < 0.0) && (raw >= -cap || error > 0.0) {
                        *stored = candidate;
                    }
                    joints.insert(
                        name.into(),
                        (input.gain_nm_s_per_rad * error + *stored).clamp(-cap, cap),
                    );
                }
                torques_nm.insert(id.clone(), joints);
            }
            for (id, holds) in &input.arm_hold {
                let joints = torques_nm.entry(id.clone()).or_default();
                for (name, hold) in holds {
                    let (position, velocity, _) = sim.joint_feedback(id, name)?;
                    let effort = hold.stiffness_nm_per_rad * (hold.position_rad - position)
                        - hold.damping_nm_s_per_rad * velocity;
                    joints.insert(
                        name.clone(),
                        effort.clamp(-hold.effort_limit_nm, hold.effort_limit_nm),
                    );
                }
            }
            let mut observation = sim.step(&Action {
                steps: 1,
                torques_nm,
            })?;
            for c in &observation.contacts {
                *contacts
                    .entry((c.first.clone(), c.second.clone()))
                    .or_default() += c.normal_impulse_ns;
            }
            if step % 24 == 0 || step == phase.steps {
                observation.contacts = std::mem::take(&mut contacts)
                    .into_iter()
                    .map(
                        |((first, second), normal_impulse_ns)| crate::arena::Contact {
                            first,
                            second,
                            normal_impulse_ns,
                        },
                    )
                    .collect();
                samples.push(Sample {
                    phase: index,
                    phase_step: step,
                    observation,
                });
            }
        }
    }
    Ok(samples)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn root() -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots")
    }
    #[test]
    fn held_arm_removes_floor_support_during_driving_and_braking() {
        let mut input: Episode =
            serde_json::from_str(include_str!("scenarios/wheel_arm_hold.json")).unwrap();
        let held = run(&input, &root()).unwrap();
        let is_arm_floor = |c: &crate::arena::Contact| {
            let link = if c.first == "ground" {
                &c.second
            } else if c.second == "ground" {
                &c.first
            } else {
                return false;
            };
            link.starts_with("alice/")
                && !link.contains(crate::omni::PREFIX)
                && c.normal_impulse_ns > 0.0
        };
        assert!(!held
            .iter()
            .flat_map(|s| &s.observation.contacts)
            .any(is_arm_floor));
        for sample in &held {
            let agent = &sample.observation.agents["alice"];
            for (name, target) in &input.arm_hold["alice"] {
                let joint = &agent.joints[name];
                assert!(
                    (joint.position_rad - target.position_rad).abs() < 0.2,
                    "{name}: {}",
                    joint.position_rad
                );
                assert!(joint.commanded_effort_nm.abs() <= target.effort_limit_nm);
            }
        }
        let moving = &held
            .iter()
            .find(|s| s.phase == 1 && s.phase_step == 240)
            .unwrap()
            .observation
            .agents["alice"];
        assert!(moving.linear_velocity_m_s[0].hypot(moving.linear_velocity_m_s[2]) > 0.05);
        let stopped = &held.last().unwrap().observation.agents["alice"];
        assert!(stopped.linear_velocity_m_s[0].hypot(stopped.linear_velocity_m_s[2]) < 0.01);
        assert!(stopped.joints["shoulder_lift"].commanded_effort_nm.abs() > 0.1);
        // Same initial model, gravity, and wheel commands, but no arm controller.
        input.arm_hold.clear();
        input.phases.truncate(1);
        let unpowered = run(&input, &root()).unwrap();
        assert!(unpowered
            .iter()
            .flat_map(|s| &s.observation.contacts)
            .any(is_arm_floor));
    }

    #[test]
    fn arm_hold_rejects_wheel_conflicts_and_out_of_range_targets() {
        let mut input: Episode =
            serde_json::from_str(include_str!("scenarios/wheel_arm_hold.json")).unwrap();
        input
            .arm_hold
            .get_mut("alice")
            .unwrap()
            .get_mut("shoulder_lift")
            .unwrap()
            .position_rad = 10.0;
        assert!(run(&input, &root())
            .unwrap_err()
            .to_string()
            .contains("outside joint limits"));
        let hold = input
            .arm_hold
            .get_mut("alice")
            .unwrap()
            .remove("shoulder_lift")
            .unwrap();
        input.arm_hold.get_mut("alice").unwrap().clear();
        input
            .arm_hold
            .get_mut("alice")
            .unwrap()
            .insert(WHEELS[0].into(), hold);
        assert!(run(&input, &root())
            .unwrap_err()
            .to_string()
            .contains("drive wheel"));
    }
    #[test]
    fn independently_commanded_agents_share_clock_and_replay() {
        let config: crate::arena::Episode =
            serde_json::from_str(include_str!("scenarios/contact_arena.json")).unwrap();
        let input = Episode {
            config: config.config,
            gain_nm_s_per_rad: 0.03,
            integral_gain_nm_per_rad: 0.1,
            arm_hold: [("alice", 0.1), ("bob", -0.1)]
                .into_iter()
                .map(|(id, position_rad)| {
                    (
                        id.into(),
                        BTreeMap::from([(
                            "shoulder_pan".into(),
                            ArmHold {
                                position_rad,
                                stiffness_nm_per_rad: 1.0,
                                damping_nm_s_per_rad: 0.02,
                                effort_limit_nm: 0.3,
                            },
                        )]),
                    )
                })
                .collect(),
            phases: vec![
                Phase {
                    steps: 1,
                    wheel_speeds_rad_s: BTreeMap::from([
                        ("alice".into(), [2.0, -1.0, -1.0]),
                        ("bob".into(), [-2.0, 1.0, 1.0]),
                    ]),
                },
                Phase {
                    steps: 25,
                    wheel_speeds_rad_s: BTreeMap::new(),
                },
            ],
        };
        let first = run(&input, &root()).unwrap();
        let replay = run(&input, &root()).unwrap();
        assert_eq!(
            serde_json::to_value(&first).unwrap(),
            serde_json::to_value(replay).unwrap()
        );
        assert_eq!(first.len(), 3);
        assert_eq!(first[2].observation.sim_time_ticks, 26 * 4_166_667);
        assert!(first[0].observation.agents["alice"].joints[WHEELS[0]].commanded_effort_nm > 0.0);
        assert!(first[0].observation.agents["bob"].joints[WHEELS[0]].commanded_effort_nm < 0.0);
        assert!(
            first[0].observation.agents["alice"].joints["shoulder_pan"].commanded_effort_nm > 0.0
        );
        assert!(
            first[0].observation.agents["bob"].joints["shoulder_pan"].commanded_effort_nm < 0.0
        );
        assert!(first[2].observation.agents.values().all(|a| WHEELS
            .iter()
            .all(|w| a.joints[*w].commanded_effort_nm == 0.0)));
        assert!(first[2]
            .observation
            .agents
            .values()
            .all(|a| a.joints["shoulder_pan"].commanded_effort_nm.abs() > 0.001));
    }

    #[test]
    fn free_space_drive_and_brake_use_feedback_without_obstacle_support() {
        let mut input: Episode =
            serde_json::from_str(include_str!("scenarios/wheel_feedback.json")).unwrap();
        let braking = run(&input, &root()).unwrap();
        input.phases[2].wheel_speeds_rad_s.clear();
        let coasting = run(&input, &root()).unwrap();
        // The counterfactual is the identical drive, changing only the stop command.
        for (a, b) in braking.iter().zip(&coasting).filter(|(a, _)| a.phase < 2) {
            assert_eq!(a.observation.state_hash, b.observation.state_hash);
        }
        let start = &braking
            .iter()
            .find(|s| s.phase == 0 && s.phase_step == 120)
            .unwrap()
            .observation
            .agents["alice"];
        let moving = &braking
            .iter()
            .find(|s| s.phase == 1 && s.phase_step == 240)
            .unwrap()
            .observation
            .agents["alice"];
        assert!(moving.position_m[0] - start.position_m[0] > 0.08);
        assert!(moving.linear_velocity_m_s[0].hypot(moving.linear_velocity_m_s[2]) > 0.05);
        let last = &braking.last().unwrap().observation;
        assert_eq!(last.sim_time_ticks, 600 * crate::arena::STEP_TICKS);
        let stopped = &last.agents["alice"];
        assert!(stopped.linear_velocity_m_s[0].hypot(stopped.linear_velocity_m_s[2]) < 0.01);
        assert!(WHEELS
            .iter()
            .all(|w| stopped.joints[*w].velocity_rad_s.abs() < 0.1));
        for sample in braking.iter().chain(&coasting) {
            assert!(
                !sample.observation.contacts.iter().any(|c| c.first == "wall"
                    || c.second == "wall"
                    || (c.first.starts_with("alice/") && c.second == "shared_box"))
            );
            assert!(WHEELS
                .iter()
                .all(|w| sample.observation.agents["alice"].joints[*w]
                    .commanded_effort_nm
                    .abs()
                    <= 1.0));
        }
        assert!(braking
            .iter()
            .filter(|s| s.phase == 2)
            .any(|s| WHEELS
                .iter()
                .any(|w| s.observation.agents["alice"].joints[*w]
                    .commanded_effort_nm
                    .abs()
                    > 0.001)));
        assert!(coasting
            .iter()
            .filter(|s| s.phase == 2)
            .all(|s| WHEELS
                .iter()
                .all(|w| s.observation.agents["alice"].joints[*w].commanded_effort_nm == 0.0)));
        // Ground/arm friction also stops the coast case; do not claim a general
        // stopping-distance advantage from this high-friction experiment.
    }
}
