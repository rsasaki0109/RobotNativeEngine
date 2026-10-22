//! Browser-local kinematic control of the vendored Unitree G1.
use crate::playground::{Playground, GOALS, GOAL_RADIUS_M};
use rne_ecs::{Entity, World};
use rne_math::{Quat, Vec3};
use rne_render::{load_stl_bytes, RenderScene, TriangleMesh, VisualShape};
use rne_robot::Joint;
use rne_urdf_import::{
    parse_urdf, rpy_to_quat, spawn_urdf_robot_with_config, UrdfGeometry, UrdfJoint, UrdfJointType,
    UrdfSpawnConfig,
};
use rne_world::{world_transform_of, Transform3};
use std::sync::Arc;

const URDF: &str = include_str!("../../../assets/robots/g1_description/g1_23dof.urdf");

#[derive(Debug)]
struct Part {
    link: Entity,
    offset: Transform3,
    shape: VisualShape,
    color: [f32; 4],
    mesh: Arc<TriangleMesh>,
}

/// G1 pose viewer. Motion is kinematic; no balance/contact solver runs.
#[derive(Debug)]
pub struct G1Scene {
    world: World,
    base: Entity,
    joints: Vec<(Entity, Entity, UrdfJoint)>,
    parts: Vec<Part>,
    heading_rad: f64,
    playground: Playground,
}

/// Complete command for one fixed kinematic preview tick.
#[derive(Clone, Debug)]
pub struct G1Input {
    /// Forward command in [-1, 1].
    pub forward: f64,
    /// Yaw command in [-1, 1].
    pub turn: f64,
    /// Joint coordinates in URDF order, in radians and within source limits.
    pub targets_rad: Vec<f64>,
    /// Reset the base before applying the command.
    pub reset: bool,
}

impl G1Scene {
    /// Imports the official, vendored 23-DoF model and embedded STL meshes.
    pub fn load() -> Result<Self, String> {
        let urdf = parse_urdf(URDF).map_err(|error| error.to_string())?;
        let mut world = World::new();
        let spawned = spawn_urdf_robot_with_config(
            &mut world,
            &urdf,
            UrdfSpawnConfig {
                attach_physics: false,
                attach_colliders: false,
                attach_mesh_colliders: false,
                ..UrdfSpawnConfig::default()
            },
        )
        .map_err(|error| error.to_string())?;
        world.entity_mut(spawned.base_link).insert(home());
        let joints = urdf
            .joints
            .iter()
            .filter(|joint| joint.joint_type == UrdfJointType::Revolute)
            .map(|joint| {
                (
                    spawned.joints[&joint.name],
                    spawned.links[&joint.child],
                    joint.clone(),
                )
            })
            .collect();
        let mut parts = Vec::new();
        for link in &urdf.links {
            for visual in &link.visuals {
                let UrdfGeometry::Mesh { path, scale } = &visual.geometry else {
                    continue;
                };
                let mesh =
                    load_stl_bytes(path, mesh_bytes(path)?).map_err(|error| error.to_string())?;
                parts.push(Part {
                    link: spawned.links[&link.name],
                    offset: Transform3::from_translation_rotation(
                        visual.origin_xyz,
                        rpy_to_quat(visual.origin_rpy),
                    ),
                    shape: VisualShape::Mesh {
                        path: path.clone(),
                        scale: *scale,
                    },
                    color: visual.material_rgba.unwrap_or([0.7, 0.7, 0.75, 1.0]),
                    mesh: Arc::new(mesh),
                });
            }
        }
        Ok(Self {
            world,
            base: spawned.base_link,
            joints,
            parts,
            heading_rad: 0.0,
            playground: Playground::default(),
        })
    }

    /// Number of revolute coordinates expected in a command.
    pub fn joint_count(&self) -> usize {
        self.joints.len()
    }

    /// Camera focus follows the pelvis.
    pub fn focus(&self) -> Vec3 {
        world_transform_of(&self.world, self.base).translation + Vec3::Y * 0.15
    }

    /// Validates the whole command, then advances one kinematic tick.
    ///
    /// Translation is 0.015 m/tick and yaw 0.025 rad/tick at full input.
    /// The browser schedules ticks; this is not real-time simulated walking.
    pub fn control_frame(&mut self, input: G1Input) -> Result<RenderScene, String> {
        self.play_frame(input, false, false, false)
    }

    /// Advances the kinematic playground, with optional gait, wave and course start.
    ///
    /// Wave and start are one-tick commands. Starting a course restores the base
    /// pose; reset cancels both animation and course. Targets remain in radians.
    pub fn play_frame(
        &mut self,
        input: G1Input,
        animate: bool,
        wave: bool,
        start: bool,
    ) -> Result<RenderScene, String> {
        if !input.forward.is_finite()
            || input.forward.abs() > 1.0
            || !input.turn.is_finite()
            || input.turn.abs() > 1.0
            || input.targets_rad.len() != self.joints.len()
        {
            return Err("invalid G1 drive command or joint count".into());
        }
        for ((_, _, joint), angle) in self.joints.iter().zip(&input.targets_rad) {
            let limit = joint.limit.ok_or("missing source joint limit")?;
            if !angle.is_finite() || !(limit.lower..=limit.upper).contains(angle) {
                return Err(format!("target outside source limits: {}", joint.name));
            }
        }
        let start = start && !input.reset;
        let wave = wave && !input.reset;
        if input.reset || start {
            self.heading_rad = 0.0;
            self.world.entity_mut(self.base).insert(home());
            self.playground = Playground::default();
            if start {
                self.playground.course.start();
            }
        }
        let previous = self.focus();
        self.heading_rad =
            (self.heading_rad + input.turn * 0.025).rem_euclid(std::f64::consts::TAU);
        let yaw = Quat::from_rotation_y(self.heading_rad);
        {
            let mut base = self
                .world
                .get_mut::<Transform3>(self.base)
                .ok_or("missing pelvis")?;
            base.rotation = yaw * home().rotation;
            base.translation += yaw * Vec3::X * (input.forward * 0.015);
            base.translation.x = base.translation.x.clamp(-4.0, 4.0);
            base.translation.z = base.translation.z.clamp(-4.0, 4.0);
        }
        let position = self.focus();
        self.playground.tick(
            (position - previous).length(),
            input.turn,
            animate,
            wave,
            position,
        );
        for ((entity, child, joint), manual_rad) in self.joints.iter().zip(input.targets_rad) {
            let limit = joint.limit.ok_or("missing source joint limit")?;
            let angle = self
                .playground
                .angle(&joint.name, manual_rad)
                .clamp(limit.lower, limit.upper);
            let rotation = rpy_to_quat(joint.origin_rpy)
                * Quat::from_axis_angle(joint.axis.normalize(), angle);
            self.world
                .entity_mut(*child)
                .insert(Transform3::from_translation_rotation(
                    joint.origin_xyz,
                    rotation,
                ));
            if let Some(mut coordinate) = self.world.get_mut::<Joint>(*entity) {
                coordinate.position = angle;
                coordinate.velocity = 0.0;
            }
        }
        Ok(self.render())
    }

    /// Current course progress, distance and elapsed fixed preview ticks.
    pub fn mission_status(&self) -> String {
        self.playground.course.status(self.focus())
    }

    /// Current pose animation state for the controls HUD.
    pub fn motion_status(&self) -> &'static str {
        self.playground.motion_status()
    }

    /// Base heading around the world's up axis, in radians.
    pub fn heading_rad(&self) -> f64 {
        self.heading_rad
    }

    fn render(&self) -> RenderScene {
        let mut scene = RenderScene::new();
        scene.items.push(RenderScene::item_from_visual(
            Transform3::from_translation_rotation(Vec3::new(0.0, -0.025, 0.0), Quat::IDENTITY),
            VisualShape::Box {
                size_m: Vec3::new(10.0, 0.05, 10.0),
            },
            [0.18, 0.25, 0.29, 1.0],
            Transform3::IDENTITY,
        ));
        // Floor markers make translation/rotation visible as the camera follows.
        for x in -4..=4 {
            for z in -4..=4 {
                scene.items.push(RenderScene::item_from_visual(
                    Transform3::from_translation_rotation(
                        Vec3::new(x as f64, 0.002, z as f64),
                        Quat::IDENTITY,
                    ),
                    VisualShape::Box {
                        size_m: Vec3::new(0.05, 0.004, 0.05),
                    },
                    [0.4, 0.65, 0.7, 1.0],
                    Transform3::IDENTITY,
                ));
            }
        }
        for (index, (x, z)) in GOALS.iter().copied().enumerate() {
            let color = if index < self.playground.course.reached {
                [0.1, 0.9, 0.45, 1.0]
            } else if index == self.playground.course.reached {
                [1.0, 0.65, 0.08, 1.0]
            } else {
                [0.35, 0.45, 0.55, 1.0]
            };
            for segment in 0..16 {
                let angle = f64::from(segment) * std::f64::consts::TAU / 16.0;
                scene.items.push(RenderScene::item_from_visual(
                    Transform3::from_translation_rotation(
                        Vec3::new(
                            x + GOAL_RADIUS_M * angle.cos(),
                            0.016,
                            z + GOAL_RADIUS_M * angle.sin(),
                        ),
                        Quat::from_rotation_y(-angle),
                    ),
                    VisualShape::Box {
                        size_m: Vec3::new(0.065, 0.03, 0.14),
                    },
                    color,
                    Transform3::IDENTITY,
                ));
            }
            if index == self.playground.course.reached {
                scene.items.push(RenderScene::item_from_visual(
                    Transform3::from_translation_rotation(Vec3::new(x, 0.2, z), Quat::IDENTITY),
                    VisualShape::Sphere { radius_m: 0.08 },
                    color,
                    Transform3::IDENTITY,
                ));
            }
        }
        for part in &self.parts {
            let mut item = RenderScene::item_from_visual(
                world_transform_of(&self.world, part.link),
                part.shape.clone(),
                part.color,
                part.offset,
            );
            item.mesh = Some(Arc::clone(&part.mesh));
            scene.items.push(item);
        }
        scene
    }
}

fn home() -> Transform3 {
    Transform3::from_translation_rotation(
        Vec3::new(0.0, 0.80, 0.0),
        Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
    )
}

fn mesh_bytes(path: &str) -> Result<&'static [u8], String> {
    let bytes: &'static [u8] = match path {
        "meshes/pelvis.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/pelvis.STL")
        }
        "meshes/pelvis_contour_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/pelvis_contour_link.STL")
        }
        "meshes/left_hip_pitch_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/left_hip_pitch_link.STL")
        }
        "meshes/left_hip_roll_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/left_hip_roll_link.STL")
        }
        "meshes/left_hip_yaw_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/left_hip_yaw_link.STL")
        }
        "meshes/left_knee_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/left_knee_link.STL")
        }
        "meshes/left_ankle_pitch_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/left_ankle_pitch_link.STL")
        }
        "meshes/left_ankle_roll_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/left_ankle_roll_link.STL")
        }
        "meshes/right_hip_pitch_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/right_hip_pitch_link.STL")
        }
        "meshes/right_hip_roll_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/right_hip_roll_link.STL")
        }
        "meshes/right_hip_yaw_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/right_hip_yaw_link.STL")
        }
        "meshes/right_knee_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/right_knee_link.STL")
        }
        "meshes/right_ankle_pitch_link.STL" => include_bytes!(
            "../../../assets/robots/g1_description/meshes/right_ankle_pitch_link.STL"
        ),
        "meshes/right_ankle_roll_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/right_ankle_roll_link.STL")
        }
        "meshes/waist_yaw_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/waist_yaw_link.STL")
        }
        "meshes/torso_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/torso_link.STL")
        }
        "meshes/logo_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/logo_link.STL")
        }
        "meshes/head_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/head_link.STL")
        }
        "meshes/waist_support_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/waist_support_link.STL")
        }
        "meshes/left_shoulder_pitch_link.STL" => include_bytes!(
            "../../../assets/robots/g1_description/meshes/left_shoulder_pitch_link.STL"
        ),
        "meshes/left_shoulder_roll_link.STL" => include_bytes!(
            "../../../assets/robots/g1_description/meshes/left_shoulder_roll_link.STL"
        ),
        "meshes/left_shoulder_yaw_link.STL" => include_bytes!(
            "../../../assets/robots/g1_description/meshes/left_shoulder_yaw_link.STL"
        ),
        "meshes/left_elbow_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/left_elbow_link.STL")
        }
        "meshes/left_wrist_roll_rubber_hand.STL" => include_bytes!(
            "../../../assets/robots/g1_description/meshes/left_wrist_roll_rubber_hand.STL"
        ),
        "meshes/right_shoulder_pitch_link.STL" => include_bytes!(
            "../../../assets/robots/g1_description/meshes/right_shoulder_pitch_link.STL"
        ),
        "meshes/right_shoulder_roll_link.STL" => include_bytes!(
            "../../../assets/robots/g1_description/meshes/right_shoulder_roll_link.STL"
        ),
        "meshes/right_shoulder_yaw_link.STL" => include_bytes!(
            "../../../assets/robots/g1_description/meshes/right_shoulder_yaw_link.STL"
        ),
        "meshes/right_elbow_link.STL" => {
            include_bytes!("../../../assets/robots/g1_description/meshes/right_elbow_link.STL")
        }
        "meshes/right_wrist_roll_rubber_hand.STL" => include_bytes!(
            "../../../assets/robots/g1_description/meshes/right_wrist_roll_rubber_hand.STL"
        ),
        _ => return Err(format!("mesh is not embedded: {path}")),
    };
    Ok(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn animated_pose_respects_limits_and_reset_cancels_commands() {
        let mut scene = G1Scene::load().unwrap();
        let mut command = neutral(&scene);
        command.forward = 1.0;
        command.targets_rad = scene
            .joints
            .iter()
            .map(|(_, _, joint)| joint.limit.unwrap().upper)
            .collect();
        for tick in 0..40 {
            scene
                .play_frame(command.clone(), true, tick == 0, false)
                .unwrap();
            for (entity, _, joint) in &scene.joints {
                let angle = scene.world.get::<Joint>(*entity).unwrap().position;
                let limit = joint.limit.unwrap();
                assert!((limit.lower..=limit.upper).contains(&angle));
            }
        }
        let mut reset = neutral(&scene);
        reset.reset = true;
        scene.play_frame(reset, true, true, true).unwrap();
        assert_eq!(scene.motion_status(), "立ち姿");
        assert!(scene.mission_status().starts_with("未開始"));
        assert_eq!(scene.focus().x, 0.0);
    }

    #[test]
    fn drive_commands_complete_the_entire_course() {
        let mut scene = G1Scene::load().unwrap();
        scene
            .play_frame(neutral(&scene), true, false, true)
            .unwrap();
        for _ in 0..2000 {
            let reached = scene.playground.course.reached;
            if reached == GOALS.len() {
                break;
            }
            let (x, z) = GOALS[reached];
            let delta = Vec3::new(x, scene.focus().y, z) - scene.focus();
            let desired = (-delta.z).atan2(delta.x);
            let error = (desired - scene.heading_rad + std::f64::consts::PI)
                .rem_euclid(std::f64::consts::TAU)
                - std::f64::consts::PI;
            let mut command = neutral(&scene);
            if error.abs() > 0.03 {
                command.turn = error.signum();
            } else {
                command.forward = 1.0;
            }
            scene.play_frame(command, true, false, false).unwrap();
        }
        assert_eq!(scene.playground.course.reached, 4);
        assert!(scene.mission_status().starts_with("クリア"));
        let completed = scene.mission_status();
        scene
            .play_frame(neutral(&scene), true, false, false)
            .unwrap();
        assert_eq!(scene.mission_status(), completed);
    }
    fn neutral(scene: &G1Scene) -> G1Input {
        G1Input {
            forward: 0.0,
            turn: 0.0,
            targets_rad: vec![0.0; scene.joint_count()],
            reset: false,
        }
    }
    #[test]
    fn g1_meshes_and_manual_control_work_without_gpu() {
        let mut scene = G1Scene::load().unwrap();
        assert_eq!(scene.joint_count(), 23);
        assert_eq!(scene.parts.len(), 29);
        let start = scene.focus();
        let mut command = neutral(&scene);
        command.forward = 1.0;
        command.targets_rad[3] = 0.5;
        for _ in 0..60 {
            scene.control_frame(command.clone()).unwrap();
        }
        assert!((scene.focus().x - start.x - 0.9).abs() < 1e-9);
        assert_eq!(
            scene
                .world
                .get::<Joint>(scene.joints[3].0)
                .unwrap()
                .position,
            0.5
        );
        let mut reset = neutral(&scene);
        reset.reset = true;
        let frame = scene.control_frame(reset).unwrap();
        assert_eq!(scene.focus(), start);
        assert_eq!(
            frame
                .items
                .iter()
                .filter(|item| item.mesh.is_some())
                .count(),
            29
        );
    }
    #[test]
    fn invalid_targets_preserve_the_live_pose() {
        let mut scene = G1Scene::load().unwrap();
        let start = scene.focus();
        let mut command = neutral(&scene);
        command.forward = 1.0;
        command.targets_rad[0] = f64::NAN;
        assert!(scene.control_frame(command.clone()).is_err());
        command.targets_rad[0] = 100.0;
        assert!(scene.control_frame(command.clone()).is_err());
        command.targets_rad.clear();
        assert!(scene.control_frame(command).is_err());
        assert_eq!(scene.focus(), start);
    }
    #[test]
    fn repeated_commands_are_deterministic_and_bounded() {
        let mut scene = G1Scene::load().unwrap();
        let mut command = neutral(&scene);
        command.forward = 1.0;
        for _ in 0..400 {
            scene.control_frame(command.clone()).unwrap();
        }
        assert_eq!(scene.focus().x, 4.0);
        command.reset = true;
        scene.control_frame(command.clone()).unwrap();
        let a = scene.focus();
        scene.control_frame(command).unwrap();
        assert_eq!(scene.focus(), a);
    }
}
