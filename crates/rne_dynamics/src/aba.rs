//! Articulated-body algorithm (ABA) forward dynamics.
//!
//! [`aba`] computes `qdd = M(q)⁻¹ (tau - C(q, qd) qd - g(q))` in `O(n)` by
//! Featherstone's three recursive passes, without forming or factoring the
//! mass matrix. It produces the same result as
//! [`crate::forward_dynamics`] (the dense `O(n³)` solve) to round-off, and it
//! is the forward pass a simulation loop runs every step.
//!
//! The conventions match [`crate::rnea`]: motion vectors are `[linear;
//! angular]`, a floating base's generalized velocity and acceleration are its
//! body-frame twist and twist derivative, and gravity enters as a fictitious
//! base acceleration.

use crate::algorithms::{inertia_matrices, mat6_transpose, validate, xup_transforms, DenseMatrix};
use crate::model::{ArticulatedModel, DynamicsError};
use crate::spatial::{
    add6, cross_force, cross_motion, dot6, mat3_mul_vec, mat3_transpose, mat6_add, mat6_mul,
    mat6_mul_vec, mat6_transpose_mul_vec, rotation_matrix, scale6, Mat6, SpatialVec,
};

/// Smallest articulated inertia `Sᵀ Iᴬ S` accepted for a joint.
const MIN_JOINT_INERTIA: f64 = 1.0e-12;

/// Forward dynamics by the articulated-body algorithm.
///
/// Returns the generalized acceleration for configuration `q`, velocity `qd`,
/// and generalized force `tau` (including floating-base wrench rows, in the
/// base body frame). Fails with [`DynamicsError::SingularMassMatrix`] when a
/// joint drives a subtree with no inertia, where the dense mass matrix would
/// be singular as well.
#[allow(clippy::needless_range_loop)]
pub fn aba(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
) -> Result<Vec<f64>, DynamicsError> {
    validate(model, q, "q")?;
    validate(model, qd, "qd")?;
    validate(model, tau, "tau")?;

    let kinematics = model.kinematic.forward_kinematics(q)?;
    let transforms = kinematics.transforms();
    let xup = xup_transforms(model, transforms);
    let inertia = inertia_matrices(model);
    let link_count = model.link_count();
    let floating = model.base_dof() == 6;

    let base_rotation = rotation_matrix(transforms[0].rotation);
    let gravity_body = mat3_mul_vec(&mat3_transpose(&base_rotation), model.gravity_m_s2);

    // Pass 1: link velocities, velocity-product accelerations, bias forces.
    let mut velocity: Vec<SpatialVec> = vec![[0.0; 6]; link_count];
    let mut bias_acceleration: Vec<SpatialVec> = vec![[0.0; 6]; link_count];
    let mut articulated_inertia: Vec<Mat6> = inertia.clone();
    let mut articulated_bias: Vec<SpatialVec> = vec![[0.0; 6]; link_count];
    if floating {
        velocity[0] = [qd[0], qd[1], qd[2], qd[3], qd[4], qd[5]];
    }
    for index in 0..link_count {
        if let Some(parent) = model.links[index].parent {
            let mut link_velocity = mat6_mul_vec(&xup[index], &velocity[parent]);
            if let Some(dof) = joint_dof(model, index) {
                let joint_velocity = scale6(&model.dofs[dof].s, qd[dof]);
                link_velocity = add6(&link_velocity, &joint_velocity);
                bias_acceleration[index] = cross_motion(&link_velocity, &joint_velocity);
            }
            velocity[index] = link_velocity;
        }
        let momentum = mat6_mul_vec(&inertia[index], &velocity[index]);
        articulated_bias[index] = cross_force(&velocity[index], &momentum);
    }

    // Pass 2: articulated inertias and bias forces, leaves to root.
    let mut joint_u: Vec<SpatialVec> = vec![[0.0; 6]; link_count];
    let mut joint_d = vec![0.0; link_count];
    let mut joint_force = vec![0.0; link_count];
    for index in (1..link_count).rev() {
        let parent = model.links[index]
            .parent
            .expect("non-root link has a parent");
        let mut passed_inertia = articulated_inertia[index];
        let mut passed_bias = add6(
            &articulated_bias[index],
            &mat6_mul_vec(&articulated_inertia[index], &bias_acceleration[index]),
        );
        if let Some(dof) = joint_dof(model, index) {
            let subspace = model.dofs[dof].s;
            let u_vec = mat6_mul_vec(&articulated_inertia[index], &subspace);
            let d = dot6(&subspace, &u_vec);
            if d.is_nan() || d <= MIN_JOINT_INERTIA {
                return Err(DynamicsError::SingularMassMatrix);
            }
            let force = tau[dof] - dot6(&subspace, &articulated_bias[index]);
            for row in 0..6 {
                for col in 0..6 {
                    passed_inertia[row][col] -= u_vec[row] * u_vec[col] / d;
                }
            }
            passed_bias = add6(
                &articulated_bias[index],
                &mat6_mul_vec(&passed_inertia, &bias_acceleration[index]),
            );
            passed_bias = add6(&passed_bias, &scale6(&u_vec, force / d));
            joint_u[index] = u_vec;
            joint_d[index] = d;
            joint_force[index] = force;
        }
        let transpose = mat6_transpose(&xup[index]);
        articulated_inertia[parent] = mat6_add(
            &articulated_inertia[parent],
            &mat6_mul(&transpose, &mat6_mul(&passed_inertia, &xup[index])),
        );
        articulated_bias[parent] = add6(
            &articulated_bias[parent],
            &mat6_transpose_mul_vec(&xup[index], &passed_bias),
        );
    }

    // Pass 3: accelerations, root to leaves. Gravity is a base acceleration.
    let mut qdd = vec![0.0; model.nv()];
    let mut acceleration: Vec<SpatialVec> = vec![[0.0; 6]; link_count];
    if floating {
        let mut root = DenseMatrix::zeros(6, 6);
        for row in 0..6 {
            for col in 0..6 {
                root.set(row, col, articulated_inertia[0][row][col]);
            }
        }
        let rhs: Vec<f64> = (0..6).map(|k| tau[k] - articulated_bias[0][k]).collect();
        let base = root.solve(&rhs).ok_or(DynamicsError::SingularMassMatrix)?;
        acceleration[0].copy_from_slice(&base);
        qdd[..6].copy_from_slice(&base);
        for k in 0..3 {
            qdd[k] += gravity_body[k];
        }
    } else {
        for k in 0..3 {
            acceleration[0][k] = -gravity_body[k];
        }
    }
    for index in 1..link_count {
        let parent = model.links[index]
            .parent
            .expect("non-root link has a parent");
        let mut link_acceleration = add6(
            &mat6_mul_vec(&xup[index], &acceleration[parent]),
            &bias_acceleration[index],
        );
        if let Some(dof) = joint_dof(model, index) {
            let value =
                (joint_force[index] - dot6(&joint_u[index], &link_acceleration)) / joint_d[index];
            qdd[dof] = value;
            link_acceleration = add6(&link_acceleration, &scale6(&model.dofs[dof].s, value));
        }
        acceleration[index] = link_acceleration;
    }
    Ok(qdd)
}

/// Degree of freedom driven by the joint into link `index`, if any.
fn joint_dof(model: &ArticulatedModel, index: usize) -> Option<usize> {
    if index == 0 {
        return None;
    }
    model.link_dofs[index].first().copied()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::forward_dynamics;
    use rne_ecs::{spawn_named, Entity, World};
    use rne_math::{Quat, Vec3};
    use rne_physics::{RigidBody, RigidBodyInertia};
    use rne_robot::{FloatingBase, Joint, JointKind, JointLimits, Link, Robot, RobotId};
    use rne_world::Transform3;

    struct Lcg(u64);

    impl Lcg {
        fn next(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((self.0 >> 11) as f64 / (1_u64 << 53) as f64) * 2.0 - 1.0
        }

        fn vec3(&mut self) -> Vec3 {
            Vec3::new(self.next(), self.next(), self.next())
        }
    }

    fn body(
        world: &mut World,
        robot: Entity,
        name: &str,
        rng: &mut Lcg,
        local: Transform3,
    ) -> Entity {
        let entity = spawn_named(world, name);
        let principal = [
            0.02 + 0.05 * rng.next().abs(),
            0.02 + 0.05 * rng.next().abs(),
            0.02 + 0.05 * rng.next().abs(),
        ];
        world.entity_mut(entity).insert((
            Link {
                robot,
                name: name.into(),
            },
            local,
            RigidBody {
                mass_kg: 0.5 + rng.next().abs(),
                ..RigidBody::default()
            },
            RigidBodyInertia {
                center_of_mass_local_m: rng.vec3() * 0.1,
                ixx_kg_m2: principal[0],
                ixy_kg_m2: 0.002 * rng.next(),
                ixz_kg_m2: 0.002 * rng.next(),
                iyy_kg_m2: principal[1],
                iyz_kg_m2: 0.002 * rng.next(),
                izz_kg_m2: principal[2],
            },
        ));
        entity
    }

    /// A branching tree mixing revolute, prismatic, and fixed joints with
    /// skewed axes, offset origins, and full inertia tensors.
    fn branching_tree(floating: bool, seed: u64) -> ArticulatedModel {
        let mut rng = Lcg(seed);
        let mut world = World::new();
        let robot = spawn_named(&mut world, "robot");
        let base = body(&mut world, robot, "base", &mut rng, Transform3::IDENTITY);
        world.entity_mut(robot).insert(Robot {
            robot_id: RobotId::new_v4(),
            model_name: "tree".into(),
            base_link: base,
        });
        if floating {
            world.entity_mut(base).insert(FloatingBase);
        }
        // (parent index, kind) for each child link; index 0 is the base.
        let layout = [
            (0, JointKind::Revolute),
            (1, JointKind::Revolute),
            (2, JointKind::Prismatic),
            (0, JointKind::Revolute),
            (4, JointKind::Fixed),
            (5, JointKind::Revolute),
            (0, JointKind::Prismatic),
            (7, JointKind::Continuous),
        ];
        let mut links = vec![base];
        for (child, (parent, kind)) in layout.into_iter().enumerate() {
            let rotation = Quat::from_scaled_axis(rng.vec3() * 0.8);
            let local = Transform3::from_translation_rotation(rng.vec3() * 0.3, rotation);
            let link = body(&mut world, robot, &format!("link{child}"), &mut rng, local);
            let joint = spawn_named(&mut world, format!("joint{child}"));
            world.entity_mut(joint).insert(Joint {
                robot,
                parent_link: links[parent],
                child_link: link,
                kind,
                limits: JointLimits::default(),
                axis: rng.vec3().normalize(),
                position: 0.0,
                velocity: 0.0,
            });
            links.push(link);
        }
        ArticulatedModel::from_robot(&world, robot).expect("model")
    }

    fn assert_matches_dense(model: &ArticulatedModel, seed: u64) {
        let mut rng = Lcg(seed);
        let nv = model.nv();
        for _ in 0..20 {
            let mut q: Vec<f64> = (0..nv).map(|_| rng.next()).collect();
            if model.base_dof() == 6 {
                // Keep pitch away from the roll-pitch-yaw singularity.
                q[4] *= 0.5;
            }
            let qd: Vec<f64> = (0..nv).map(|_| 2.0 * rng.next()).collect();
            let tau: Vec<f64> = (0..nv).map(|_| 5.0 * rng.next()).collect();
            let fast = aba(model, &q, &qd, &tau).expect("aba");
            let dense = forward_dynamics(model, &q, &qd, &tau).expect("dense");
            let scale = dense.iter().fold(1.0_f64, |m, v| m.max(v.abs()));
            for (a, b) in fast.iter().zip(&dense) {
                assert!(
                    (a - b).abs() <= 1.0e-9 * scale,
                    "aba {fast:?}\ndense {dense:?}"
                );
            }
        }
    }

    #[test]
    fn aba_matches_dense_forward_dynamics_on_a_fixed_tree() {
        let model = branching_tree(false, 3);
        assert_eq!(model.base_dof(), 0);
        assert_matches_dense(&model, 17);
    }

    #[test]
    fn aba_matches_dense_forward_dynamics_on_a_floating_tree() {
        let model = branching_tree(true, 5);
        assert_eq!(model.base_dof(), 6);
        assert_matches_dense(&model, 29);
    }

    #[test]
    fn aba_rejects_wrong_lengths() {
        let model = branching_tree(false, 3);
        assert!(matches!(
            aba(&model, &[0.0], &[0.0], &[0.0]),
            Err(DynamicsError::DimensionMismatch { .. })
        ));
    }
}
