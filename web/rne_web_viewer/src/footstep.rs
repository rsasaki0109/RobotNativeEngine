//! Kinematic foot planting: fixed support poses and raised, alternating swing feet.
use rne_ecs::{Entity, World};
use rne_math::{Pose3, Quat};
use rne_robot::{IkOptions, KinematicModel};
use rne_world::Transform3;

#[derive(Debug)]
pub(crate) struct Footstep {
    model: KinematicModel,
    feet: [Entity; 2],
    nominal: [Transform3; 2],
    active: [Vec<bool>; 2],
    order: Vec<usize>,
    q: Vec<f64>,
    anchors: Option<[Transform3; 2]>,
    swing: usize,
    progress: f64,
    start: Transform3,
    goal: Transform3,
}
impl Footstep {
    pub(crate) fn new(
        world: &World,
        robot: Entity,
        root: Transform3,
        names: &[String],
    ) -> Result<Self, String> {
        let model = KinematicModel::from_robot(world, robot).map_err(|e| e.to_string())?;
        let dofs = model.movable_dof_names();
        let order = names
            .iter()
            .map(|n| dofs.iter().position(|d| d == n).unwrap())
            .collect();
        let feet = [
            model.link_entity_by_name("left_ankle_roll_link").unwrap(),
            model.link_entity_by_name("right_ankle_roll_link").unwrap(),
        ];
        let fk = model
            .forward_kinematics(&vec![0.0; model.dof()])
            .map_err(|e| e.to_string())?;
        let nominal = feet.map(|foot| {
            let p = fk.link_transform(foot).unwrap();
            Transform3::from_translation_rotation(
                root.rotation.inverse() * (p.translation - root.translation),
                root.rotation.inverse() * p.rotation,
            )
        });
        let active = ["left_", "right_"].map(|prefix| {
            dofs.iter()
                .map(|n| {
                    n.starts_with(prefix)
                        && ["hip_", "knee_", "ankle_"]
                            .iter()
                            .any(|kind| n.contains(kind))
                })
                .collect()
        });
        Ok(Self {
            q: vec![0.0; model.dof()],
            model,
            feet,
            nominal,
            active,
            order,
            anchors: None,
            swing: 0,
            progress: 0.0,
            start: Transform3::IDENTITY,
            goal: Transform3::IDENTITY,
        })
    }
    #[cfg(test)]
    pub(crate) fn targets(&self) -> Option<[Transform3; 2]> {
        self.anchors
    }
    pub(crate) fn reset(&mut self) {
        self.anchors = None;
        self.progress = 0.0;
        self.swing = 0;
        self.q.fill(0.0);
    }
    pub(crate) fn engaged(&self) -> bool {
        self.anchors.is_some()
    }
    pub(crate) fn angles(
        &mut self,
        mut root: Transform3,
        previous: Transform3,
        yaw_delta: f64,
        manual: &[f64],
        enabled: bool,
    ) -> Option<Vec<f64>> {
        let delta = root.translation - previous.translation;
        let distance = delta.x.hypot(delta.z);
        let moving = distance > 1e-8 || yaw_delta.abs() > 1e-8;
        let names = self.model.movable_dof_names();
        let custom_legs = self
            .order
            .iter()
            .zip(manual)
            .any(|(i, q)| (self.active[0][*i] || self.active[1][*i]) && q.abs() > 1e-8);
        if !enabled || custom_legs {
            self.reset();
            return None;
        }
        if distance > 0.10 || yaw_delta.abs() > 0.30 {
            self.reset();
            return None;
        }
        if self.anchors.is_none() {
            if !moving {
                return None;
            }
            self.anchors = Some(self.nominal.map(|p| previous.mul_transform(&p)));
            // Avoid the straight-knee singularity on the initial solve.
            for (i, name) in names.iter().enumerate() {
                self.q[i] = if name.ends_with("knee_joint") {
                    0.30
                } else if name.ends_with("hip_pitch_joint") || name.ends_with("ankle_pitch_joint") {
                    -0.15
                } else {
                    0.0
                };
            }
        }
        let mut anchors = self.anchors.unwrap();
        if self.progress == 0.0 && moving {
            self.start = anchors[self.swing];
            let future = Transform3::from_translation_rotation(
                root.translation + delta * 18.0,
                Quat::from_rotation_y(yaw_delta * 9.0) * root.rotation,
            );
            self.goal = future.mul_transform(&self.nominal[self.swing]);
        }
        if moving || self.progress > 0.0 {
            if !moving {
                self.goal = root.mul_transform(&self.nominal[self.swing]);
            }
            self.progress = (self.progress
                + if moving {
                    distance / 0.18 + yaw_delta.abs() / 0.45
                } else {
                    1.0 / 12.0
                })
            .min(1.0);
            let s = self.progress * self.progress * (3.0 - 2.0 * self.progress);
            let mut target = Transform3::from_translation_rotation(
                self.start.translation.lerp(self.goal.translation, s),
                self.start.rotation.slerp(self.goal.rotation, s),
            );
            target.translation.y += 0.045 * (std::f64::consts::PI * self.progress).sin().powi(2);
            anchors[self.swing] = target;
            if self.progress >= 1.0 {
                self.swing = 1 - self.swing;
                self.progress = 0.0;
            }
        }
        self.anchors = Some(anchors);
        root.translation.y -= 0.04;
        self.model.set_root_transform(root);
        for (index, value) in self.order.iter().zip(manual) {
            if !self.active[0][*index] && !self.active[1][*index] {
                self.q[*index] = *value;
            }
        }
        for (side, target) in anchors.iter().enumerate() {
            if let Ok(solution) = self.model.inverse_kinematics_active(
                &Pose3 {
                    translation: target.translation,
                    rotation: target.rotation,
                },
                self.feet[side],
                &self.q,
                &IkOptions {
                    max_iterations: 100,
                    position_tolerance_m: 0.0005,
                    orientation_tolerance_rad: 0.002,
                    damping: 0.01,
                    step_size: 0.8,
                    ..IkOptions::default()
                },
                &self.active[side],
            ) {
                self.q = solution.joint_positions;
            }
        }
        Some(self.order.iter().map(|i| self.q[*i]).collect())
    }
}
