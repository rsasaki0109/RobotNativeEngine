//! Shared-world, direct-torque LeKiwi experiments. All robot poses are physics owned.
use anyhow::{ensure, Context, Result};
use rne_core::SimDuration;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    hash_physics_state_v2, Collider, ExternalBodyWrench, JointActuation, JointEffortMeasurement,
    JointMotor, PhysicsBackend, PhysicsOwnedPose, PhysicsWorldDesc, PhysicsWorldId, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_urdf_import::{
    attach_urdf_document_articulation, spawn_urdf_document_with_owned_collisions,
    UrdfArticulationConfig, UrdfJointType, UrdfSpawnConfig,
};
use rne_world::{world_transform_of, Transform3};
use serde::{Deserialize, Serialize};
use std::{collections::BTreeMap, path::Path};

pub(crate) const STEP_TICKS: u64 = 4_166_667;
/// Each array entry owns one robot; IDs remain stable through an episode.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Config {
    pub agents: Vec<Spawn>,
    /// Explicit self-contact policy; CAD collision proxies can overlap internally.
    pub self_collisions: bool,
    /// Opt-in chassis proxy and passive roller experiment; not calibrated hardware.
    #[serde(default)]
    pub idealized_omni: bool,
}
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Spawn {
    pub id: String,
    pub position_m: [f64; 3],
    pub yaw_rad: f64,
}
/// Missing agents/joints receive zero torque for this frame, never a held command.
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Action {
    pub steps: u32,
    pub torques_nm: BTreeMap<String, BTreeMap<String, f64>>,
    /// World-frame forces at actuated child-link origins, repeated each physics tick.
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    pub forces_n: BTreeMap<String, BTreeMap<String, [f64; 3]>>,
}
#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub(crate) struct Episode {
    pub config: Config,
    pub actions: Vec<Action>,
}
#[derive(Debug, Serialize)]
pub(crate) struct JointObservation {
    pub position_rad: f64,
    pub velocity_rad_s: f64,
    pub commanded_effort_nm: f64,
    /// Direct effort accepted by the backend, not a physical torque sensor model.
    pub applied_effort_nm: Option<f64>,
    pub effort_limit_nm: f64,
}
#[derive(Debug, Serialize)]
pub(crate) struct AgentObservation {
    pub position_m: [f64; 3],
    pub rotation_xyzw: [f64; 4],
    pub linear_velocity_m_s: [f64; 3],
    pub joints: BTreeMap<String, JointObservation>,
}
#[derive(Debug, Serialize)]
pub(crate) struct Contact {
    pub first: String,
    pub second: String,
    pub normal_impulse_ns: f64,
}
#[derive(Debug, Serialize)]
pub(crate) struct Observation {
    pub sim_time_ticks: u64,
    pub state_hash: String,
    pub agents: BTreeMap<String, AgentObservation>,
    pub object_positions_m: BTreeMap<String, [f64; 3]>,
    /// Positive solver impulses summed across the action's physics steps.
    pub contacts: Vec<Contact>,
}
#[derive(Debug)]
struct Agent {
    base: Entity,
    joints: BTreeMap<String, (Entity, f64)>,
}
pub(crate) struct Arena {
    world: World,
    backend: RapierBackend,
    physics_world: PhysicsWorldId,
    agents: BTreeMap<String, Agent>,
    names: BTreeMap<Entity, String>,
    ticks: u64,
}
impl std::fmt::Debug for Arena {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Arena")
            .field("agents", &self.agents)
            .field("ticks", &self.ticks)
            .finish_non_exhaustive()
    }
}
impl Arena {
    pub(crate) fn new(config: &Config, root: &Path) -> Result<Self> {
        ensure!(
            (1..=8).contains(&config.agents.len()),
            "arena requires 1..=8 agents"
        );
        let mut ids = std::collections::BTreeSet::new();
        for spawn in &config.agents {
            ensure!(
                !spawn.id.is_empty()
                    && spawn.id.len() <= 32
                    && spawn
                        .id
                        .bytes()
                        .all(|b| b.is_ascii_alphanumeric() || b == b'_'),
                "invalid agent ID"
            );
            ensure!(ids.insert(&spawn.id), "duplicate agent ID");
            ensure!(
                spawn
                    .position_m
                    .iter()
                    .all(|v| v.is_finite() && v.abs() <= 5.0)
                    && spawn.position_m[1] >= 0.06
                    && spawn.yaw_rad.is_finite(),
                "invalid spawn pose"
            );
        }
        let mut document = crate::presets::preset("lekiwi_grasp")?.robot.document()?;
        if config.idealized_omni {
            crate::omni::add_rollers(&mut document)?;
        }
        crate::simulation::resolve_meshes(&mut document, Some(root))?;
        let mut world = World::new();
        let mut agents = BTreeMap::new();
        let mut names = BTreeMap::new();
        // Stable ID order, independent of input JSON ordering.
        let mut spawns = config.agents.iter().collect::<Vec<_>>();
        spawns.sort_by_key(|s| &s.id);
        for spawn in spawns {
            let robot = spawn_urdf_document_with_owned_collisions(
                &mut world,
                &document,
                UrdfSpawnConfig {
                    base_body_type: RigidBodyType::Dynamic,
                    self_collisions: config.self_collisions,
                    use_declared_inertial_masses: true,
                    mesh_assets_root: Some(root.to_path_buf()),
                    ..Default::default()
                },
            )?;
            world.entity_mut(robot.base_link).insert(Transform3 {
                translation: Vec3::from_array(spawn.position_m),
                rotation: Quat::from_rotation_y(spawn.yaw_rad)
                    * Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
                ..Transform3::IDENTITY
            });
            attach_urdf_document_articulation(
                &mut world,
                &document,
                &robot,
                UrdfArticulationConfig {
                    base_body_type: RigidBodyType::Dynamic,
                    multibody: true,
                    use_joint_origin_rpy: true,
                    weld_fixed_children: true,
                    ..Default::default()
                },
            )?;
            let mut poses = robot
                .links
                .values()
                .map(|e| (*e, world_transform_of(&world, *e)))
                .collect::<Vec<_>>();
            poses.sort_by_key(|(e, _)| e.index());
            for (entity, pose) in poses {
                world
                    .entity_mut(entity)
                    .remove::<rne_ecs::Parent>()
                    .insert((pose, PhysicsOwnedPose));
            }
            for (name, entity) in &robot.links {
                names.insert(*entity, format!("{}/{name}", spawn.id));
            }
            let mut joints = BTreeMap::new();
            for joint in &document.robot.joints {
                if matches!(
                    joint.joint_type,
                    UrdfJointType::Revolute | UrdfJointType::Continuous
                ) {
                    let entity = robot.links[&joint.child];
                    if joint.child.starts_with(crate::omni::PREFIX) {
                        world
                            .entity_mut(entity)
                            .remove::<JointMotor>()
                            .insert(JointActuation::Disabled);
                        continue;
                    }
                    // Unspecified wheel ratings use an explicit experimental 1 N m ceiling.
                    let cap = joint.limit.map_or(1.0, |l| l.max_effort_nm);
                    world.entity_mut(entity).remove::<JointMotor>().insert(
                        JointActuation::RevoluteEffort {
                            effort_nm: 0.0,
                            max_effort_nm: cap,
                        },
                    );
                    joints.insert(joint.name.clone(), (entity, cap));
                }
            }
            agents.insert(
                spawn.id.clone(),
                Agent {
                    base: robot.base_link,
                    joints,
                },
            );
        }
        for (name, position, half, dynamic) in [
            ("ground", [0.0, -0.05, 0.0], [6.0, 0.05, 6.0], false),
            ("wall", [1.5, 0.4, 0.0], [0.05, 0.4, 2.0], false),
            ("shared_box", [0.0, 0.15, 0.7], [0.1, 0.1, 0.1], true),
        ] {
            let entity = spawn_named(&mut world, name);
            world.entity_mut(entity).insert((
                Transform3::from_translation_rotation(Vec3::from_array(position), Quat::IDENTITY),
                RigidBody {
                    body_type: if dynamic {
                        RigidBodyType::Dynamic
                    } else {
                        RigidBodyType::Fixed
                    },
                    mass_kg: 0.2,
                    ..Default::default()
                },
                Collider::cuboid(Vec3::from_array(half)),
            ));
            names.insert(entity, name.into());
        }
        let mut backend = RapierBackend::new();
        let physics_world = backend.create_world(PhysicsWorldDesc {
            solver_iterations: 12,
            ..Default::default()
        })?;
        backend.sync_from_ecs(&mut world, physics_world)?;
        Ok(Self {
            world,
            backend,
            physics_world,
            agents,
            names,
            ticks: 0,
        })
    }

    /// Read completed joint position, velocity, and effort ceiling without advancing time.
    pub(crate) fn joint_feedback(&self, id: &str, name: &str) -> Result<(f64, f64, f64)> {
        let agent = self.agents.get(id).context("unknown agent")?;
        let (entity, cap) = agent.joints.get(name).context("unknown joint")?;
        let (position, velocity) = self
            .backend
            .multibody_joint_state(self.physics_world, *entity)
            .context("missing joint feedback")?;
        ensure!(
            position.is_finite() && velocity.is_finite(),
            "non-finite joint feedback"
        );
        Ok((position, velocity, *cap))
    }

    /// Validate a complete action without mutating the world.
    pub(crate) fn validate_action(&self, action: &Action) -> Result<()> {
        ensure!(
            (1..=240).contains(&action.steps),
            "steps must be in 1..=240"
        );
        for (id, torques) in &action.torques_nm {
            let agent = self.agents.get(id).context("unknown agent")?;
            for (joint, effort) in torques {
                ensure!(
                    agent.joints.contains_key(joint),
                    "unknown joint {id}/{joint}"
                );
                ensure!(effort.is_finite(), "non-finite joint torque");
            }
        }
        for (id, forces) in &action.forces_n {
            let agent = self.agents.get(id).context("unknown force agent")?;
            for (joint, force) in forces {
                ensure!(
                    agent.joints.contains_key(joint),
                    "unknown force target {id}/{joint}"
                );
                let force = Vec3::from_array(*force);
                ensure!(
                    force.is_finite() && force.length() <= 20.0,
                    "force must be finite and at most 20 N"
                );
            }
        }
        Ok(())
    }

    /// Advance a validated complete action in this world only.
    pub(crate) fn step(&mut self, action: &Action) -> Result<Observation> {
        self.validate_action(action)?;
        for (id, agent) in &self.agents {
            for (name, (entity, cap)) in &agent.joints {
                let effort = action
                    .torques_nm
                    .get(id)
                    .and_then(|a| a.get(name))
                    .copied()
                    .unwrap_or(0.0)
                    .clamp(-cap, *cap);
                self.world
                    .entity_mut(*entity)
                    .insert(JointActuation::RevoluteEffort {
                        effort_nm: effort,
                        max_effort_nm: *cap,
                    });
            }
        }
        let mut impulses = BTreeMap::<(String, String), f64>::new();
        for _ in 0..action.steps {
            self.backend
                .sync_from_ecs(&mut self.world, self.physics_world)?;
            for (id, forces) in &action.forces_n {
                for (joint, force) in forces {
                    let entity = self.agents[id].joints[joint].0;
                    self.backend.apply_external_body_wrench(
                        self.physics_world,
                        ExternalBodyWrench {
                            entity,
                            point_world_m: world_transform_of(&self.world, entity).translation,
                            force_world_n: Vec3::from_array(*force),
                            torque_world_nm: Vec3::ZERO,
                        },
                    )?;
                }
            }
            self.backend
                .step(self.physics_world, SimDuration::from_ticks(STEP_TICKS))?;
            self.backend
                .sync_to_ecs(&mut self.world, self.physics_world)?;
            self.ticks += STEP_TICKS;
            for contact in self.backend.contacts(self.physics_world)? {
                if contact.impulse > 0.0 {
                    let mut pair = [
                        self.names[&contact.entity_a].clone(),
                        self.names[&contact.entity_b].clone(),
                    ];
                    pair.sort();
                    *impulses
                        .entry((pair[0].clone(), pair[1].clone()))
                        .or_default() += f64::from(contact.impulse);
                }
            }
        }
        self.observe_with_contacts(
            impulses
                .into_iter()
                .map(|((first, second), normal_impulse_ns)| Contact {
                    first,
                    second,
                    normal_impulse_ns,
                })
                .collect(),
        )
    }

    /// Observe without stepping; no new contact interval is represented.
    pub(crate) fn observe(&self) -> Result<Observation> {
        self.observe_with_contacts(Vec::new())
    }

    fn observe_with_contacts(&self, contacts: Vec<Contact>) -> Result<Observation> {
        let mut agents = BTreeMap::new();
        for (id, agent) in &self.agents {
            let pose = world_transform_of(&self.world, agent.base);
            ensure!(
                pose.translation.is_finite() && pose.rotation.is_finite(),
                "non-finite base pose"
            );
            let body = self
                .world
                .get::<RigidBody>(agent.base)
                .context("missing base")?;
            ensure!(
                body.linear_velocity_m_s.is_finite(),
                "non-finite base velocity"
            );
            let mut joints = BTreeMap::new();
            for (name, (entity, cap)) in &agent.joints {
                let (q, dq) = self
                    .backend
                    .multibody_joint_state(self.physics_world, *entity)
                    .context("missing joint feedback")?;
                ensure!(q.is_finite() && dq.is_finite(), "non-finite joint feedback");
                let Some(JointActuation::RevoluteEffort { effort_nm, .. }) =
                    self.world.get::<JointActuation>(*entity)
                else {
                    anyhow::bail!("missing effort command")
                };
                joints.insert(
                    name.clone(),
                    JointObservation {
                        position_rad: q,
                        velocity_rad_s: dq,
                        commanded_effort_nm: *effort_nm,
                        applied_effort_nm: match self.world.get::<JointEffortMeasurement>(*entity) {
                            Some(JointEffortMeasurement::Revolute { measured_effort_nm }) => {
                                Some(*measured_effort_nm)
                            }
                            _ => None,
                        },
                        effort_limit_nm: *cap,
                    },
                );
            }
            agents.insert(
                id.clone(),
                AgentObservation {
                    position_m: pose.translation.to_array(),
                    rotation_xyzw: pose.rotation.to_array(),
                    linear_velocity_m_s: body.linear_velocity_m_s.to_array(),
                    joints,
                },
            );
        }
        Ok(Observation {
            sim_time_ticks: self.ticks,
            state_hash: format!("{:016x}", hash_physics_state_v2(&self.world)),
            agents,
            object_positions_m: self
                .names
                .iter()
                .filter(|(_, name)| !name.contains('/'))
                .map(|(entity, name)| {
                    (
                        name.clone(),
                        world_transform_of(&self.world, *entity)
                            .translation
                            .to_array(),
                    )
                })
                .collect(),
            contacts,
        })
    }
}

pub(crate) fn run(episode: &Episode, root: &Path) -> Result<Vec<Observation>> {
    ensure!(
        !episode.actions.is_empty() && episode.actions.len() <= 1000,
        "episode needs 1..=1000 action frames"
    );
    ensure!(
        episode
            .actions
            .iter()
            .map(|a| u64::from(a.steps))
            .sum::<u64>()
            <= 24_000,
        "episode exceeds 100 simulation seconds"
    );
    let mut arena = Arena::new(&episode.config, root)?;
    episode.actions.iter().map(|a| arena.step(a)).collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    fn root() -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots")
    }
    fn episode() -> Episode {
        serde_json::from_str(include_str!("scenarios/contact_arena.json")).unwrap()
    }
    #[test]
    fn external_force_is_scoped_repeated_and_does_not_replace_motor_torque() {
        let mut pushed = Arena::new(&episode().config, &root()).unwrap();
        let mut repeated = Arena::new(&episode().config, &root()).unwrap();
        let mut passive = Arena::new(&episode().config, &root()).unwrap();
        let mut action = Action {
            steps: 12,
            torques_nm: BTreeMap::new(),
            forces_n: BTreeMap::from([(
                "alice".into(),
                BTreeMap::from([("wrist_flex".into(), [0.0, 0.0, 1.0])]),
            )]),
        };
        let forced = pushed.step(&action).unwrap();
        let mut one = action.clone();
        one.steps = 1;
        for _ in 0..12 {
            repeated.step(&one).unwrap();
        }
        assert_eq!(forced.state_hash, repeated.observe().unwrap().state_hash);
        action.forces_n.clear();
        let zero = passive.step(&action).unwrap();
        assert_ne!(forced.state_hash, zero.state_hash);
        assert_eq!(
            serde_json::to_value(&forced.agents["bob"]).unwrap(),
            serde_json::to_value(&zero.agents["bob"]).unwrap()
        );
        assert!(forced.agents["alice"]
            .joints
            .values()
            .all(|j| j.commanded_effort_nm == 0.0));
        // Removing the force matches an explicit zero force: no stale wrench.
        let cleared = pushed.step(&action).unwrap();
        one.steps = 12;
        one.forces_n
            .get_mut("alice")
            .unwrap()
            .insert("wrist_flex".into(), [0.0; 3]);
        assert_eq!(cleared.state_hash, repeated.step(&one).unwrap().state_hash);
        let before = pushed.observe().unwrap().state_hash;
        for force in [[f64::NAN, 0.0, 0.0], [21.0, 0.0, 0.0]] {
            one.forces_n
                .get_mut("alice")
                .unwrap()
                .insert("wrist_flex".into(), force);
            assert!(pushed.step(&one).is_err());
            assert_eq!(before, pushed.observe().unwrap().state_hash);
        }
    }

    #[test]
    fn shared_world_is_dynamic_and_replays_in_agent_id_order() {
        let mut input = episode();
        let first = run(&input, &root()).unwrap();
        input.config.agents.reverse();
        let replay = run(&input, &root()).unwrap();
        assert_eq!(
            serde_json::to_value(&first).unwrap(),
            serde_json::to_value(&replay).unwrap()
        );
        assert_eq!(first.last().unwrap().sim_time_ticks, 264 * STEP_TICKS);
        assert!(first
            .iter()
            .flat_map(|o| &o.contacts)
            .any(|c| c.first.starts_with("alice/") && c.second.starts_with("alice/")));
        for id in ["alice", "bob"] {
            assert_eq!(first[0].agents[id].joints.len(), 9);
            assert!((first[0].agents[id].position_m[1] - 0.12).abs() > 0.005);
            assert!(first
                .iter()
                .flat_map(|o| &o.contacts)
                .any(|c| (c.first == "ground" && c.second.starts_with(id))
                    || (c.second == "ground" && c.first.starts_with(id))));
        }
    }
    #[test]
    fn torque_is_scoped_capped_and_cleared_and_invalid_batches_are_atomic() {
        let mut sim = Arena::new(&episode().config, &root()).unwrap();
        let mut passive = Arena::new(&episode().config, &root()).unwrap();
        let zero = passive
            .step(&Action {
                steps: 1,
                forces_n: BTreeMap::new(),
                torques_nm: BTreeMap::new(),
            })
            .unwrap();
        let mut action = Action {
            steps: 1,
            forces_n: BTreeMap::new(),
            torques_nm: BTreeMap::from([(
                "alice".into(),
                BTreeMap::from([("shoulder_pan".into(), 0.02), ("gripper".into(), 2.0)]),
            )]),
        };
        let obs = sim.step(&action).unwrap();
        assert_ne!(
            obs.agents["alice"].joints["shoulder_pan"].velocity_rad_s,
            zero.agents["alice"].joints["shoulder_pan"].velocity_rad_s
        );
        assert_eq!(
            serde_json::to_value(&obs.agents["bob"]).unwrap(),
            serde_json::to_value(&zero.agents["bob"]).unwrap()
        );
        assert!(
            (obs.agents["alice"].joints["shoulder_pan"]
                .applied_effort_nm
                .unwrap()
                - 0.02)
                .abs()
                < 1e-6
        );
        assert_eq!(
            obs.agents["alice"].joints["gripper"].commanded_effort_nm,
            0.06
        );
        assert_eq!(
            obs.agents["alice"].joints["shoulder_pan"].commanded_effort_nm,
            0.02
        );
        assert_eq!(
            obs.agents["bob"].joints["shoulder_pan"].commanded_effort_nm,
            0.0
        );
        let hash = hash_physics_state_v2(&sim.world);
        action
            .torques_nm
            .insert("bob".into(), BTreeMap::from([("missing".into(), 1.0)]));
        assert!(sim.step(&action).is_err());
        assert_eq!(sim.ticks, STEP_TICKS);
        assert_eq!(hash_physics_state_v2(&sim.world), hash);
        action.torques_nm.clear();
        let obs = sim.step(&action).unwrap();
        assert!(obs
            .agents
            .values()
            .flat_map(|a| a.joints.values())
            .all(|j| j.commanded_effort_nm == 0.0));
        assert!(sim
            .world
            .iter_entities()
            .all(|e| !e.contains::<JointMotor>()));
        for agent in sim.agents.values() {
            assert_eq!(
                sim.world.get::<RigidBody>(agent.base).unwrap().body_type,
                RigidBodyType::Dynamic
            );
        }
    }
    #[test]
    fn wheel_torque_drives_passive_rollers_into_wall_and_settles() {
        let episode: Episode =
            serde_json::from_str(include_str!("scenarios/contact_drive.json")).unwrap();
        let mut sim = Arena::new(&episode.config, &root()).unwrap();
        let rollers = sim
            .names
            .iter()
            .filter(|(_, n)| n.contains(crate::omni::PREFIX))
            .map(|(e, _)| *e)
            .collect::<Vec<_>>();
        assert_eq!(rollers.len(), 48);
        assert!(rollers.iter().all(|e| matches!(
            sim.world.get::<JointActuation>(*e),
            Some(JointActuation::Disabled)
        )));
        let mut observations = Vec::new();
        let mut peak_roller_speed: f64 = 0.0;
        for action in &episode.actions {
            observations.push(sim.step(action).unwrap());
            for entity in &rollers {
                peak_roller_speed = peak_roller_speed.max(
                    sim.backend
                        .multibody_joint_state(sim.physics_world, *entity)
                        .unwrap()
                        .1
                        .abs(),
                );
            }
        }
        let first = &observations[0];
        assert!(!first
            .contacts
            .iter()
            .any(|c| c.first == "wall" || c.second == "wall"));
        let last = observations.last().unwrap();
        let moved = last.agents["alice"].position_m[0] - first.agents["alice"].position_m[0];
        assert!(moved > 0.1 && moved < 0.3, "travel {moved}");
        assert!(observations
            .iter()
            .flat_map(|o| &o.contacts)
            .any(|c| (c.first == "wall" || c.second == "wall") && c.normal_impulse_ns > 0.0));
        assert!(last.agents["alice"].position_m[0] < 1.4);
        let speed = last.agents["alice"].linear_velocity_m_s;
        assert!(speed[0].hypot(speed[2]) < 0.01);
        assert!(
            peak_roller_speed > 0.1,
            "passive roller speed {peak_roller_speed}"
        );
        assert_eq!(last.agents["alice"].joints.len(), 9);
    }

    #[test]
    fn passive_roller_world_replays_exactly() {
        let mut input: Episode =
            serde_json::from_str(include_str!("scenarios/contact_drive.json")).unwrap();
        input.actions.truncate(1);
        input.actions[0].steps = 24;
        let first = run(&input, &root()).unwrap();
        let replay = run(&input, &root()).unwrap();
        assert_eq!(
            serde_json::to_value(first).unwrap(),
            serde_json::to_value(replay).unwrap()
        );
    }

    #[test]
    fn robot_drop_produces_inter_agent_solver_contacts() {
        let mut config = episode().config;
        // Disabling same-robot contacts must still retain cross-robot contacts.
        config.self_collisions = false;
        config.agents[0].position_m = [0.0, 0.12, 0.0];
        config.agents[1].position_m = [0.0, 0.8, 0.0];
        let mut sim = Arena::new(&config, &root()).unwrap();
        let obs = sim
            .step(&Action {
                steps: 240,
                forces_n: BTreeMap::new(),
                torques_nm: BTreeMap::new(),
            })
            .unwrap();
        assert!(
            obs.contacts.iter().any(|c| c.first.starts_with("alice/")
                && c.second.starts_with("bob/")
                && c.normal_impulse_ns > 0.0),
            "{:?}",
            obs.contacts
        );
    }
}
