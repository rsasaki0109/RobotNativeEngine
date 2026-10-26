//! Native fixed-step workbench simulation; no window or GPU is required.

use crate::drive::{advance_pose, wheel_index, DriveCommand};
use crate::project::{JointInfo, JointPosition, Project};
use anyhow::{ensure, Context, Result};
use rne_core::{SimClock, SimDuration};
use rne_ecs::{spawn_named, Entity, World};
use rne_math::Vec3;
use rne_physics::{
    hash_physics_state_v2, Collider, CommandedKinematicPose, JointMotor, PhysicsBackend,
    PhysicsOwnedPose, PhysicsWorldDesc, PhysicsWorldId, RaycastQuery, RigidBody, RigidBodyInertia,
    RigidBodyType,
};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_render::{LinkVisuals, RenderScene, Visual, VisualShape};
use rne_urdf_import::{
    attach_urdf_document_articulation, spawn_urdf_document_with_config, UrdfArticulationConfig,
    UrdfSpawnConfig,
};
use rne_world::{world_transform_of, Transform3};
use serde::Serialize;
use std::{collections::BTreeMap, path::Path};

const STEP_TICKS: u64 = 4_166_667;

/// One actual measured joint coordinate alongside its requested target.
#[derive(Debug, Serialize)]
pub(crate) struct JointReading {
    pub info: JointInfo,
    pub measured: f64,
    pub velocity: f64,
    pub target: JointPosition,
}

/// A timestamped, instantaneous noiseless scan from an explicit world mount.
#[derive(Debug, Serialize)]
pub(crate) struct LidarReading {
    pub sim_time_ticks: u64,
    pub origin_m: [f64; 3],
    pub max_range_m: f64,
    pub ranges_m: Vec<Option<f64>>,
    pub points_m: Vec<[f64; 3]>,
}

/// Measured box state and positive-impulse robot contacts at the displayed step.
#[cfg_attr(not(feature = "gpu"), allow(dead_code))]
#[derive(Debug, Serialize)]
pub(crate) struct ObjectReading {
    pub name: String,
    pub position_m: [f64; 3],
    pub dynamic: bool,
    pub height_gain_m: f64,
    pub robot_contacts: Vec<String>,
}

/// Owns all backend handles at the application boundary.
pub(crate) struct Simulation {
    pub project: Project,
    world: World,
    backend: RapierBackend,
    physics_world: PhysicsWorldId,
    clock: SimClock,
    joints: Vec<(JointInfo, Entity)>,
    robot_links: Vec<Entity>,
    base_link: Entity,
    base_pose: [f64; 3],
    drive: DriveCommand,
    wheels: Vec<(usize, Entity)>,
    recording: crate::replay::Recording,
    recording_bytes: usize,
    objects: BTreeMap<String, Entity>,
    links: BTreeMap<String, Entity>,
}

impl std::fmt::Debug for Simulation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Simulation")
            .field("clock", &self.clock)
            .field("joints", &self.joints)
            .finish_non_exhaustive()
    }
}

fn resolve_meshes(
    document: &mut rne_urdf_import::UrdfDocument,
    asset_root: Option<&Path>,
) -> Result<()> {
    for element in document
        .robot
        .links
        .iter_mut()
        .flat_map(|link| link.visuals.iter_mut().chain(&mut link.collisions))
    {
        if let rne_urdf_import::UrdfGeometry::Mesh { path, .. } = &mut element.geometry {
            let root = asset_root
                .context("mesh model requires --asset-root")?
                .canonicalize()?;
            let mut candidate = rne_render::resolve_package_uri(path, &root);
            // A bundled multi-package root also accepts historical package://
            // sources without changing their saved-project identity.
            if !candidate.exists() {
                if let Some(relative) = path.strip_prefix("package://") {
                    candidate = root.join(relative);
                }
            }
            let resolved = candidate
                .canonicalize()
                .with_context(|| format!("missing mesh {path}; check --asset-root"))?;
            ensure!(
                resolved.starts_with(&root),
                "mesh lies outside --asset-root: {path}"
            );
            ensure!(
                resolved.metadata()?.len() <= 32 * 1024 * 1024,
                "mesh exceeds 32 MiB: {path}"
            );
            *path = resolved
                .to_str()
                .context("mesh path is not UTF-8")?
                .to_owned();
        }
    }
    Ok(())
}

fn configure_planar_base(
    world: &mut World,
    document: &rne_urdf_import::UrdfDocument,
    spawned: &rne_urdf_import::SpawnedUrdfRobot,
) -> Result<Vec<(usize, Entity)>> {
    let mut wheels = Vec::new();
    // Freeze initial world transforms before removing the imported hierarchy.
    // The multibody solver then owns every child pose; only its root is commanded.
    let mut poses = spawned
        .links
        .values()
        .map(|entity| (*entity, world_transform_of(world, *entity)))
        .collect::<Vec<_>>();
    poses.sort_by_key(|(entity, _)| entity.index());
    for (entity, transform) in poses {
        world
            .entity_mut(entity)
            .remove::<rne_ecs::Parent>()
            .insert(transform);
        if entity != spawned.base_link {
            world.entity_mut(entity).insert(PhysicsOwnedPose);
        }
    }
    world
        .entity_mut(spawned.base_link)
        .insert(CommandedKinematicPose);
    for joint in &document.robot.joints {
        if let Some(index) = wheel_index(&joint.child) {
            let entity = spawned.links[&joint.child];
            let mut motor = world
                .get_mut::<JointMotor>(entity)
                .context("missing wheel motor")?;
            motor.stiffness = 0.0;
            motor.gain = 28.0;
            motor.max_force = 10.0;
            wheels.push((index, entity));
        }
    }
    Ok(wheels)
}

fn spawn_objects(world: &mut World, project: &Project) -> BTreeMap<String, Entity> {
    let mut objects = BTreeMap::new();
    for object in &project.objects {
        let entity = spawn_named(world, format!("workbench/{}", object.name));
        let mut collider = Collider::cuboid(Vec3::from_array(object.size_m) * 0.5);
        let mut body = RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        };
        if let Some(dynamics) = &object.dynamics {
            body.body_type = RigidBodyType::Dynamic;
            body.mass_kg = dynamics.mass_kg;
            collider.material.friction = dynamics.friction;
            let [x, y, z] = object.size_m;
            let factor = dynamics.mass_kg / 12.0;
            world.entity_mut(entity).insert(RigidBodyInertia {
                center_of_mass_local_m: Vec3::ZERO,
                ixx_kg_m2: factor * (y * y + z * z),
                iyy_kg_m2: factor * (x * x + z * z),
                izz_kg_m2: factor * (x * x + y * y),
                ixy_kg_m2: 0.0,
                ixz_kg_m2: 0.0,
                iyz_kg_m2: 0.0,
            });
        }
        world.entity_mut(entity).insert((
            Transform3::from_translation_rotation(
                Vec3::from_array(object.position_m),
                rne_math::Quat::IDENTITY,
            ),
            body,
            collider,
            Visual::new(
                VisualShape::Box {
                    size_m: Vec3::from_array(object.size_m),
                },
                object.color_rgba,
            ),
        ));
        objects.insert(object.name.clone(), entity);
    }
    objects
}

impl Simulation {
    pub(crate) fn new(project: Project, asset_root: Option<&Path>) -> Result<Self> {
        project.validate()?;
        let mut document = project.robot.document()?;
        ensure!(
            document.robot.links.len() <= 512 && document.robot.joints.len() <= 512,
            "workbench supports at most 512 links and joints"
        );
        resolve_meshes(&mut document, asset_root)?;
        ensure!(
            document.robot.joints.iter().all(|j| j.mimic.is_none()),
            "physics preview does not yet support mimic joints"
        );
        let body_type = if project.planar_base.is_some() {
            RigidBodyType::Kinematic
        } else {
            RigidBodyType::Fixed
        };
        let mut world = World::new();
        let spawned = spawn_urdf_document_with_config(
            &mut world,
            &document,
            UrdfSpawnConfig {
                base_body_type: body_type,
                self_collisions: false,
                mesh_assets_root: asset_root.map(Path::to_path_buf),
                use_declared_inertial_masses: project.use_declared_inertias,
                ..UrdfSpawnConfig::default()
            },
        )?;
        world.entity_mut(spawned.base_link).insert(Transform3 {
            translation: Vec3::new(
                0.0,
                project
                    .planar_base
                    .as_ref()
                    .map_or(0.0, |base| base.height_m),
                0.0,
            ),
            rotation: rne_urdf_import::rpy_to_quat(Vec3::from_array(
                project.robot_rotation_rpy_rad,
            )),
            ..Transform3::IDENTITY
        });
        attach_urdf_document_articulation(
            &mut world,
            &document,
            &spawned,
            UrdfArticulationConfig {
                base_body_type: body_type,
                multibody: true,
                use_joint_origin_rpy: true,
                weld_fixed_children: true,
                ..UrdfArticulationConfig::default()
            },
        )?;
        let wheels = if project.planar_base.is_some() {
            configure_planar_base(&mut world, &document, &spawned)?
        } else {
            Vec::new()
        };
        let mut joints = Vec::new();
        for info in project.joint_catalog()? {
            let joint = document
                .robot
                .joints
                .iter()
                .find(|joint| joint.name == info.name)
                .context("joint disappeared from robot document")?;
            let entity = spawned.links[&joint.child];
            let mut motor = world
                .get_mut::<JointMotor>(entity)
                .context("missing joint motor")?;
            motor.stiffness = 200.0;
            motor.gain = 28.0;
            motor.max_force = info.max_effort;
            motor.target_position = 0.0;
            motor.velocity_rad_s = 0.0;
            joints.push((info, entity));
        }
        let objects = spawn_objects(&mut world, &project);
        let mut backend = RapierBackend::new();
        let physics_world = backend.create_world(PhysicsWorldDesc {
            solver_iterations: 12,
            ..PhysicsWorldDesc::default()
        })?;
        backend.sync_from_ecs(&mut world, physics_world)?;
        let mut robot_links = spawned.links.values().copied().collect::<Vec<_>>();
        robot_links.sort_by_key(|entity| entity.index());
        let recording = crate::replay::Recording {
            schema_version: 1,
            project: project.clone(),
            frames: Vec::new(),
        };
        let recording_bytes = serde_json::to_vec(&recording)?.len();
        Ok(Self {
            recording,
            recording_bytes,
            objects,
            links: spawned
                .links
                .iter()
                .map(|(name, entity)| (name.clone(), *entity))
                .collect(),
            project,
            world,
            backend,
            physics_world,
            clock: SimClock::new(SimDuration::from_ticks(STEP_TICKS)),
            joints,
            robot_links,
            base_link: spawned.base_link,
            base_pose: [0.0; 3],
            drive: DriveCommand::default(),
            wheels,
        })
    }

    pub(crate) fn camera_fit(&self) -> (Vec3, f64) {
        let mut minimum = Vec3::splat(f64::INFINITY);
        let mut maximum = Vec3::splat(f64::NEG_INFINITY);
        for entity in &self.robot_links {
            let point = world_transform_of(&self.world, *entity).translation;
            minimum = minimum.min(point);
            maximum = maximum.max(point);
        }
        let focus = (minimum + maximum) * 0.5;
        let distance_m = ((maximum - minimum).length() * 1.6 + 0.1).clamp(0.4, 30.0);
        (focus, distance_m)
    }

    pub(crate) fn set_targets(&mut self, targets: BTreeMap<String, JointPosition>) -> Result<()> {
        let mut candidate = self.project.clone();
        candidate.targets = targets;
        candidate.validate()?;
        self.project = candidate;
        Ok(())
    }

    pub(crate) fn set_drive(&mut self, command: DriveCommand) -> Result<()> {
        ensure!(
            self.project.planar_base.is_some(),
            "this project has a fixed base"
        );
        command.validate()?;
        self.drive = command;
        Ok(())
    }

    pub(crate) fn base_reading(&self) -> ([f64; 3], DriveCommand, [f64; 3]) {
        (self.base_pose, self.drive, self.drive.wheel_velocities())
    }

    pub(crate) fn recording(&self) -> &crate::replay::Recording {
        &self.recording
    }

    pub(crate) fn step(&mut self, steps: u32) -> Result<()> {
        ensure!(
            (1..=240).contains(&steps),
            "step count must be between 1 and 240"
        );
        ensure!(
            self.recording.frames.len() < crate::replay::MAX_FRAMES,
            "recording full; save and reset before continuing"
        );
        let mut frame = crate::replay::ReplayFrame {
            steps,
            targets: self.project.targets.clone(),
            drive: self.drive,
            sim_time_ticks: "0".repeat(20),
            state_hash: "0".repeat(16),
        };
        let frame_bytes = serde_json::to_vec(&frame)?.len() + 1;
        ensure!(
            self.recording_bytes + frame_bytes <= crate::replay::MAX_RECORDING_BYTES,
            "recording exceeds 4 MiB; save and reset before continuing"
        );
        let dt = self.clock.fixed_delta();
        for _ in 0..steps {
            if let Some(base) = &self.project.planar_base {
                advance_pose(&mut self.base_pose, self.drive, dt.as_seconds().value());
                self.world.entity_mut(self.base_link).insert(Transform3 {
                    translation: Vec3::new(self.base_pose[0], base.height_m, self.base_pose[1]),
                    rotation: rne_math::Quat::from_rotation_y(self.base_pose[2])
                        * rne_urdf_import::rpy_to_quat(Vec3::from_array(
                            self.project.robot_rotation_rpy_rad,
                        )),
                    ..Transform3::IDENTITY
                });
                let speeds = self.drive.wheel_velocities();
                for (index, entity) in &self.wheels {
                    self.world
                        .get_mut::<JointMotor>(*entity)
                        .context("missing wheel motor")?
                        .velocity_rad_s = speeds[*index];
                }
            }
            for (info, entity) in &self.joints {
                let goal = self.project.targets[&info.name].value();
                let mut motor = self
                    .world
                    .get_mut::<JointMotor>(*entity)
                    .context("missing servo")?;
                let delta = info.max_velocity * dt.as_seconds().value();
                motor.target_position += (goal - motor.target_position).clamp(-delta, delta);
            }
            step_physics(&mut self.backend, &mut self.world, self.physics_world, dt)?;
            self.clock.advance(dt);
        }
        frame.sim_time_ticks = self.sim_time_ticks().to_string();
        frame.state_hash = format!("{:016x}", self.state_hash());
        self.recording.frames.push(frame);
        self.recording_bytes += frame_bytes;
        Ok(())
    }

    pub(crate) fn sim_time_ticks(&self) -> u64 {
        self.clock.sim_time().ticks()
    }

    pub(crate) fn state_hash(&self) -> u64 {
        hash_physics_state_v2(&self.world)
    }

    pub(crate) fn readings(&self) -> Result<Vec<JointReading>> {
        self.joints
            .iter()
            .map(|(info, entity)| {
                let (measured, velocity) = self
                    .backend
                    .multibody_joint_state(self.physics_world, *entity)
                    .context("joint state unavailable")?;
                ensure!(
                    measured.is_finite() && velocity.is_finite(),
                    "non-finite joint measurement"
                );
                Ok(JointReading {
                    info: info.clone(),
                    measured,
                    velocity,
                    target: self.project.targets[&info.name],
                })
            })
            .collect()
    }

    #[cfg_attr(not(feature = "gpu"), allow(dead_code))]
    pub(crate) fn object_readings(&self) -> Result<Vec<ObjectReading>> {
        self.project
            .objects
            .iter()
            .map(|object| {
                Ok(ObjectReading {
                    name: object.name.clone(),
                    position_m: self.object_position(&object.name)?.to_array(),
                    dynamic: object.dynamics.is_some(),
                    height_gain_m: self.object_position(&object.name)?.y - object.position_m[1],
                    robot_contacts: self.object_contacts(&object.name)?,
                })
            })
            .collect()
    }

    pub(crate) fn object_position(&self, name: &str) -> Result<Vec3> {
        let entity = self.objects.get(name).context("unknown scene object")?;
        Ok(world_transform_of(&self.world, *entity).translation)
    }

    /// World-space bounds account for the box's simulated rotation.
    pub(crate) fn object_bounds(&self, name: &str) -> Result<(Vec3, Vec3)> {
        let entity = *self.objects.get(name).context("unknown scene object")?;
        let object = self
            .project
            .objects
            .iter()
            .find(|o| o.name == name)
            .context("missing object description")?;
        let pose = world_transform_of(&self.world, entity);
        let half = Vec3::from_array(object.size_m) * 0.5;
        let extent = (pose.rotation * Vec3::X).abs() * half.x
            + (pose.rotation * Vec3::Y).abs() * half.y
            + (pose.rotation * Vec3::Z).abs() * half.z;
        Ok((pose.translation - extent, pose.translation + extent))
    }

    pub(crate) fn object_speeds(&self, name: &str) -> Result<(f64, f64)> {
        let entity = *self.objects.get(name).context("unknown scene object")?;
        let body = self
            .world
            .get::<RigidBody>(entity)
            .context("missing object body")?;
        Ok((
            body.linear_velocity_m_s.length(),
            body.angular_velocity_rad_s.length(),
        ))
    }

    pub(crate) fn objects_in_contact(&self, first: &str, second: &str) -> Result<bool> {
        let first = *self.objects.get(first).context("unknown scene object")?;
        let second = *self.objects.get(second).context("unknown scene object")?;
        Ok(self
            .backend
            .contacts(self.physics_world)?
            .iter()
            .any(|contact| {
                contact.impulse > 0.0
                    && ((contact.entity_a == first && contact.entity_b == second)
                        || (contact.entity_b == first && contact.entity_a == second))
            }))
    }

    pub(crate) fn object_contacts(&self, name: &str) -> Result<Vec<String>> {
        let object = *self.objects.get(name).context("unknown scene object")?;
        let contacts = self.backend.contacts(self.physics_world)?;
        Ok(self
            .links
            .iter()
            .filter(|(_, link)| {
                contacts.iter().any(|contact| {
                    contact.impulse > 0.0
                        && ((contact.entity_a == object && contact.entity_b == **link)
                            || (contact.entity_b == object && contact.entity_a == **link))
                })
            })
            .map(|(name, _)| name.clone())
            .collect())
    }

    pub(crate) fn lidar(&self) -> Result<LidarReading> {
        let origin_m = [0.0, 0.5, 0.0];
        let max_range_m = 10.0;
        let mut ranges_m = Vec::with_capacity(180);
        let mut points_m = Vec::new();
        for column in 0..180 {
            let angle = std::f64::consts::TAU * f64::from(column) / 180.0;
            let hits = self.backend.raycast(
                self.physics_world,
                RaycastQuery {
                    origin_m: Vec3::from_array(origin_m),
                    direction: Vec3::new(angle.cos(), 0.0, angle.sin()),
                    max_distance_m: max_range_m,
                },
            )?;
            ranges_m.push(hits.first().map(|hit| hit.distance_m));
            if let Some(hit) = hits.first() {
                points_m.push(hit.point_m.to_array());
            }
        }
        Ok(LidarReading {
            sim_time_ticks: self.sim_time_ticks(),
            origin_m,
            max_range_m,
            ranges_m,
            points_m,
        })
    }

    pub(crate) fn render_scene(&self) -> RenderScene {
        let mut scene = RenderScene::new();
        let mut entities = self
            .world
            .iter_entities()
            .map(|e| e.id())
            .collect::<Vec<_>>();
        entities.sort_by_key(|e| e.index());
        for entity in entities {
            let transform = world_transform_of(&self.world, entity);
            if let Some(visuals) = self.world.get::<LinkVisuals>(entity) {
                for visual in &visuals.visuals {
                    scene.items.push(RenderScene::item_from_visual(
                        transform,
                        visual.shape.clone(),
                        visual.color_rgba,
                        visual.local_offset,
                    ));
                }
            } else if let Some(visual) = self.world.get::<Visual>(entity) {
                scene.items.push(RenderScene::item_from_visual(
                    transform,
                    visual.shape.clone(),
                    visual.color_rgba,
                    visual.local_offset,
                ));
            }
        }
        scene
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::project::{RobotFormat, RobotSource, SceneObject};

    fn slider() -> Project {
        Project::new(RobotSource {
            format: RobotFormat::Urdf,
            xml: include_str!("../../crates/rne_urdf_import/tests/fixtures/prismatic_slider.urdf")
                .into(),
        })
        .unwrap()
    }

    #[test]
    fn native_servo_moves_and_replay_is_deterministic() {
        let mut project = slider();
        project.targets.insert(
            "slider_joint".into(),
            JointPosition::Prismatic { position_m: 0.1 },
        );
        let mut first = Simulation::new(project.clone(), None).unwrap();
        let mut second = Simulation::new(project, None).unwrap();
        first.step(240).unwrap();
        second.step(240).unwrap();
        assert!(first.readings().unwrap()[0].measured > 0.06);
        assert_eq!(first.sim_time_ticks(), STEP_TICKS * 240);
        assert_eq!(first.state_hash(), second.state_hash());
    }

    #[test]
    fn scene_obstacle_changes_real_raycast_scan() {
        let project = slider();
        let empty = Simulation::new(project.clone(), None)
            .unwrap()
            .lidar()
            .unwrap();
        assert_eq!(empty.ranges_m[0], None);
        let mut with_box = project;
        with_box.objects.push(SceneObject {
            dynamics: None,
            name: "wall".into(),
            position_m: [2.0, 0.5, 0.0],
            size_m: [0.2, 1.0, 1.0],
            color_rgba: [0.8, 0.2, 0.1, 1.0],
        });
        let scan = Simulation::new(with_box, None).unwrap().lidar().unwrap();
        assert!((scan.ranges_m[0].unwrap() - 1.9).abs() < 1e-5);
        assert_eq!(scan.sim_time_ticks, 0);
    }
    #[test]
    fn planar_base_moves_arm_and_replays_with_stop_and_validation() {
        let mut project = slider();
        project.schema_version = 2;
        project.planar_base = Some(crate::project::PlanarBase { height_m: 0.5 });
        project.targets.insert(
            "slider_joint".into(),
            JointPosition::Prismatic { position_m: 0.08 },
        );
        let mut simulation = Simulation::new(project, None).unwrap();
        let command = DriveCommand {
            vx_m_s: 0.04,
            vz_m_s: 0.03,
            yaw_rad_s: 0.2,
        };
        simulation.set_drive(command).unwrap();
        assert!(simulation
            .set_drive(DriveCommand {
                vx_m_s: 0.2,
                ..command
            })
            .is_err());
        assert_eq!(simulation.base_reading().1, command);
        simulation.step(240).unwrap();
        let root = world_transform_of(&simulation.world, simulation.base_link);
        assert!((root.translation.x - simulation.base_pose[0]).abs() < 1e-5);
        assert!((root.translation.z - simulation.base_pose[1]).abs() < 1e-5);
        assert!((root.translation.y - 0.5).abs() < 1e-5);
        assert!(simulation.readings().unwrap()[0].measured > 0.05);
        let pose = simulation.base_pose;
        simulation.set_drive(DriveCommand::default()).unwrap();
        simulation.step(240).unwrap();
        assert_eq!(simulation.base_pose, pose);
        let replay = simulation.recording().clone();
        let verified = replay.verify(None).unwrap();
        assert_eq!(verified.state_hash(), simulation.state_hash());
        let mut tampered = replay.clone();
        tampered.frames[0].drive.vx_m_s = 0.01;
        assert!(tampered.verify(None).is_err());
        let mut tampered = replay;
        tampered.frames[0].sim_time_ticks = "0".into();
        assert!(tampered.verify(None).is_err());
        let reset = Simulation::new(simulation.project.clone(), None).unwrap();
        assert_eq!(
            reset.base_reading(),
            ([0.0; 3], DriveCommand::default(), [0.0; 3])
        );
    }

    #[test]
    fn fixed_base_rejects_drive_and_recording_limit_preserves_state() {
        let mut simulation = Simulation::new(slider(), None).unwrap();
        assert!(simulation.set_drive(DriveCommand::default()).is_err());
        simulation.recording_bytes = crate::replay::MAX_RECORDING_BYTES;
        let hash = simulation.state_hash();
        assert!(simulation.step(1).is_err());
        assert_eq!(simulation.sim_time_ticks(), 0);
        assert_eq!(simulation.state_hash(), hash);
    }

    #[test]
    fn lekiwi_arm_is_upright_and_preview_inertias_remain_usable() {
        let mut project = crate::presets::preset("lekiwi_so101").unwrap();
        project.use_declared_inertias = false;
        let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/robots");
        let mut simulation = Simulation::new(project, Some(&root)).unwrap();
        let shoulder = simulation
            .joints
            .iter()
            .find(|(info, _)| info.name == "shoulder_pan")
            .unwrap()
            .1;
        let axis = world_transform_of(&simulation.world, shoulder).rotation * Vec3::Z;
        assert!(
            axis.y.abs() > 0.999,
            "shoulder pan axis must be vertical: {axis:?}"
        );
        simulation.step(240).unwrap();
        for reading in simulation.readings().unwrap() {
            assert!(
                reading.measured >= reading.info.lower - 0.05
                    && reading.measured <= reading.info.upper + 0.05
            );
        }
    }

    #[test]
    fn vendored_lekiwi_preset_has_six_arm_joints_and_replays() {
        crate::smoke_lekiwi().unwrap();
    }
}
