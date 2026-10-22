//! Browser-local kinematic control of the vendored Unitree G1.
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
        if input.reset {
            self.heading_rad = 0.0;
            self.world.entity_mut(self.base).insert(home());
        }
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
        for ((entity, child, joint), angle) in self.joints.iter().zip(input.targets_rad) {
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
