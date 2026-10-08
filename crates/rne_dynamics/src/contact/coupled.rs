//! One contact solve shared by several articulated models.
//!
//! RaiSim resolves every contact of a world in one per-contact iteration: a
//! robot standing on a box and the box standing on the ground are coupled
//! through the same Delassus matrix. [`contact_step_coupled`] does the same
//! for any number of [`ArticulatedModel`]s. A contact joins a point on one
//! model's link either to the static world or to a point on another link — of
//! another model, or of the same model for self-collision — and its Jacobian
//! is the relative velocity of the two points. The effective mass stays block
//! diagonal, one factored block per model, so `G = J M̃⁻¹ Jᵀ` couples the
//! models only through shared contacts.

use super::{
    active_joint_limits, actuator_torques, contact_frame, effort_limit, fold_implicit_pd,
    integrate_from_base_frame, solve_contact_impulses, validate_pd, Cholesky, ContactOutcome,
    ContactStep, ContactStepConfig, JointLimitOutcome, JointLimitSide, JointPdControl, LimitRow,
    EFFORT_TOLERANCE, MAX_EFFORT_PASSES,
};
use crate::algorithms::{
    mass_matrix_from_xup, point_linear_jacobian, rnea_from_xup, xup_transforms, DenseMatrix,
};
use crate::model::{ArticulatedModel, DynamicsError};
use rne_ecs::Entity;
use rne_math::Vec3;
use rne_robot::ForwardKinematics;

/// One model and its state entering [`contact_step_coupled`].
#[derive(Clone, Copy, Debug)]
pub struct CoupledBody<'a> {
    /// Articulated model.
    pub model: &'a ArticulatedModel,
    /// Configuration.
    pub q: &'a [f64],
    /// Generalized velocity.
    pub qd: &'a [f64],
    /// Generalized force, one entry per velocity coordinate.
    pub tau: &'a [f64],
    /// Implicit joint PD, if any.
    pub pd: Option<&'a JointPdControl>,
}

/// A point fixed to a link of one model of a coupled step.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactAnchor {
    /// Index of the model in the step's body list.
    pub body: usize,
    /// Link carrying the point.
    pub link: Entity,
    /// Point in the link frame, in meters.
    pub point_local_m: Vec3,
}

/// A contact of a coupled step between a point on `a` and either the static
/// world (`b` is `None`) or a point on `b`.
///
/// The normal points from `b` (or the world) into `a`: a compressive impulse
/// pushes `a` along it and `b` against it. The gap is the signed distance
/// along the normal, as for [`super::ContactPoint`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CoupledContact {
    /// Point that the impulse pushes along the normal.
    pub a: ContactAnchor,
    /// Point that the impulse pushes against the normal, or the static world.
    pub b: Option<ContactAnchor>,
    /// Unit contact normal in the world frame, from `b` into `a`.
    pub normal_world: Vec3,
    /// Signed distance along the normal, in meters.
    pub gap_m: f64,
    /// Coulomb friction coefficient (dimensionless, non-negative).
    pub friction_coefficient: f64,
}

/// Result of [`contact_step_coupled`].
#[derive(Clone, Debug, PartialEq)]
pub struct CoupledStep {
    /// Per-model results in input order. Their `contacts` are empty: contact
    /// outcomes are shared and listed in [`Self::contacts`].
    pub bodies: Vec<ContactStep>,
    /// Per-contact outcomes in input order; the impulse acts on `a`, and its
    /// opposite on `b`.
    pub contacts: Vec<ContactOutcome>,
    /// Sweeps used by the contact solver.
    pub solver_iterations: usize,
    /// Whether the contact solver converged within its sweep budget.
    pub solver_converged: bool,
}

/// Quantities of one model that do not depend on actuator saturation.
struct BodyFrame {
    kinematics: ForwardKinematics,
    mass: DenseMatrix,
    bias: Vec<f64>,
    offset: usize,
}

/// Advances several articulated models one step with shared hard contacts.
///
/// Each model is stepped exactly as by [`super::contact_step`] — implicit PD,
/// joint limits, and effort limits included — but every contact and joint
/// limit of every model is solved in one per-contact iteration, so contacts
/// between models and between two links of one model push both sides. With a
/// single model and only world contacts, the result matches
/// [`super::contact_step`] to round-off.
pub fn contact_step_coupled(
    bodies: &[CoupledBody<'_>],
    contacts: &[CoupledContact],
    config: &ContactStepConfig,
    warm_start_n_s: Option<&[[f64; 3]]>,
) -> Result<CoupledStep, DynamicsError> {
    validate(bodies, contacts, config, warm_start_n_s)?;
    let mut offset = 0;
    let frames = bodies
        .iter()
        .map(|body| {
            let kinematics = body.model.kinematic.forward_kinematics(body.q)?;
            let xup = xup_transforms(body.model, kinematics.transforms());
            let mass = mass_matrix_from_xup(body.model, &xup);
            let bias = rnea_from_xup(
                body.model,
                kinematics.transforms(),
                &xup,
                body.qd,
                &vec![0.0; body.model.nv()],
            );
            let frame = BodyFrame {
                kinematics,
                mass,
                bias,
                offset,
            };
            offset += body.model.nv();
            Ok(frame)
        })
        .collect::<Result<Vec<_>, DynamicsError>>()?;

    let mut held: Vec<Vec<Option<f64>>> = bodies
        .iter()
        .map(|body| vec![None; body.model.nv() - body.model.base_dof()])
        .collect();
    let dt = config.step_time_s;
    let mut pass = 0;
    loop {
        let mut step = solve(bodies, &frames, contacts, &held, config, warm_start_n_s)?;
        let mut changed = false;
        for ((body, body_step), held) in bodies.iter().zip(&mut step.bodies).zip(&mut held) {
            let torques = actuator_torques(
                body.model,
                body.q,
                body.tau,
                body.pd,
                held,
                dt,
                &body_step.qd,
            );
            if config.enforce_effort_limits && pass < MAX_EFFORT_PASSES {
                for (joint, torque) in torques.iter().enumerate() {
                    if held[joint].is_some() {
                        continue;
                    }
                    if let Some(limit) = effort_limit(body.model, body.pd, joint) {
                        if torque.abs() > limit * (1.0 + EFFORT_TOLERANCE) {
                            held[joint] = Some(torque.clamp(-limit, limit));
                            changed = true;
                        }
                    }
                }
            }
            body_step.actuator_torques = torques;
            body_step.saturated_joints = (0..held.len()).filter(|&j| held[j].is_some()).collect();
        }
        if !changed {
            return Ok(step);
        }
        pass += 1;
    }
}

fn validate(
    bodies: &[CoupledBody<'_>],
    contacts: &[CoupledContact],
    config: &ContactStepConfig,
    warm_start_n_s: Option<&[[f64; 3]]>,
) -> Result<(), DynamicsError> {
    for body in bodies {
        let nv = body.model.nv();
        for values in [body.q, body.qd, body.tau] {
            if values.len() != nv {
                return Err(DynamicsError::DimensionMismatch {
                    provided: values.len(),
                    expected: nv,
                });
            }
            if values.iter().any(|value| !value.is_finite()) {
                return Err(DynamicsError::NonFiniteInput);
            }
        }
        if let Some(pd) = body.pd {
            validate_pd(body.model, pd)?;
        }
    }
    let dt = config.step_time_s;
    if !(dt.is_finite() && dt > 0.0) {
        return Err(DynamicsError::InvalidContact("step time must be positive"));
    }
    if !(0.0..=1.0).contains(&config.penetration_recovery) {
        return Err(DynamicsError::InvalidContact(
            "penetration recovery must lie in [0, 1]",
        ));
    }
    for contact in contacts {
        let anchors_valid = std::iter::once(contact.a)
            .chain(contact.b)
            .all(|anchor| anchor.body < bodies.len() && anchor.point_local_m.is_finite());
        if !anchors_valid {
            return Err(DynamicsError::InvalidContact(
                "contact anchors must name a body of the step and a finite point",
            ));
        }
        if !contact.normal_world.is_finite()
            || contact.normal_world.length_squared() <= f64::EPSILON
            || !contact.gap_m.is_finite()
        {
            return Err(DynamicsError::InvalidContact(
                "contact normal and gap must be finite and the normal non-zero",
            ));
        }
    }
    if let Some(seed) = warm_start_n_s {
        if seed.len() != contacts.len() {
            return Err(DynamicsError::DimensionMismatch {
                provided: seed.len(),
                expected: contacts.len(),
            });
        }
    }
    Ok(())
}

/// One coupled solve with some joints held at a fixed actuator torque.
fn solve(
    bodies: &[CoupledBody<'_>],
    frames: &[BodyFrame],
    contacts: &[CoupledContact],
    held: &[Vec<Option<f64>>],
    config: &ContactStepConfig,
    warm_start_n_s: Option<&[[f64; 3]]>,
) -> Result<CoupledStep, DynamicsError> {
    let dt = config.step_time_s;
    let total_nv: usize = bodies.iter().map(|body| body.model.nv()).sum();
    let mut factors = Vec::with_capacity(bodies.len());
    let mut free_velocity = Vec::with_capacity(total_nv);
    let mut limits: Vec<(usize, LimitRow)> = Vec::new();
    for (index, ((body, frame), held)) in bodies.iter().zip(frames).zip(held).enumerate() {
        let (factor, free) = free_motion(body, frame, held, dt)?;
        if config.enforce_joint_limits {
            limits.extend(
                active_joint_limits(body.model, body.q, &free, dt)
                    .into_iter()
                    .map(|limit| (index, limit)),
            );
        }
        free_velocity.extend(free);
        factors.push(factor);
    }

    let (jacobian, contact_frames, friction) =
        coupled_jacobian(bodies, frames, contacts, &limits, total_nv)?;
    let (response, delassus) = block_delassus(&factors, frames, bodies, &jacobian);
    let mut constraint_velocity = jacobian.mul_vec(&free_velocity);
    let gaps = contacts
        .iter()
        .map(|contact| contact.gap_m)
        .chain(limits.iter().map(|(_, limit)| limit.gap));
    for (index, gap) in gaps.enumerate() {
        let recovery = if gap >= 0.0 {
            1.0
        } else {
            config.penetration_recovery
        };
        constraint_velocity[3 * index + 2] += recovery * gap / dt;
    }
    let blocks = contacts.len() + limits.len();
    let warm_start = warm_start_n_s.map(|seed| {
        let mut full = seed.to_vec();
        full.resize(blocks, [0.0; 3]);
        full
    });
    let solution = solve_contact_impulses(
        &delassus,
        &constraint_velocity,
        &friction,
        &config.solver,
        warm_start.as_deref(),
    )?;

    let mut next_velocity = free_velocity;
    for (index, impulse) in solution.impulses_n_s.iter().enumerate() {
        for (axis, magnitude) in impulse.iter().enumerate() {
            if *magnitude == 0.0 {
                continue;
            }
            for (velocity, change) in next_velocity.iter_mut().zip(&response[3 * index + axis]) {
                *velocity += change * magnitude;
            }
        }
    }

    let count = contacts.len();
    let mut body_steps = Vec::with_capacity(bodies.len());
    for (index, (body, frame)) in bodies.iter().zip(frames).enumerate() {
        let nv = body.model.nv();
        let qd = next_velocity[frame.offset..frame.offset + nv].to_vec();
        let q = integrate_from_base_frame(
            body.model,
            body.q,
            &qd,
            dt,
            &frame.kinematics.transforms()[0],
        );
        let joint_limits = limits
            .iter()
            .zip(&solution.impulses_n_s[count..])
            .filter(|((owner, _), impulse)| *owner == index && impulse[2] > 0.0)
            .map(|((_, limit), impulse)| JointLimitOutcome {
                dof: limit.dof,
                side: limit.side,
                impulse: impulse[2],
            })
            .collect();
        body_steps.push(ContactStep {
            q,
            qd,
            contacts: Vec::new(),
            joint_limits,
            actuator_torques: Vec::new(),
            saturated_joints: Vec::new(),
            solver_iterations: solution.iterations,
            solver_converged: solution.converged,
        });
    }
    let outcomes = solution.impulses_n_s[..count]
        .iter()
        .zip(&solution.states)
        .zip(&contact_frames)
        .map(|((impulse, state), frame)| ContactOutcome {
            impulse_world_n_s: frame[0] * impulse[0]
                + frame[1] * impulse[1]
                + frame[2] * impulse[2],
            state: *state,
        })
        .collect();
    Ok(CoupledStep {
        bodies: body_steps,
        contacts: outcomes,
        solver_iterations: solution.iterations,
        solver_converged: solution.converged,
    })
}

/// Factors one model's effective mass and returns its unconstrained velocity.
fn free_motion(
    body: &CoupledBody<'_>,
    frame: &BodyFrame,
    held: &[Option<f64>],
    dt: f64,
) -> Result<(Cholesky, Vec<f64>), DynamicsError> {
    let base = body.model.base_dof();
    let mut effective_mass = frame.mass.clone();
    let mut generalized_force: Vec<f64> = body
        .tau
        .iter()
        .zip(&frame.bias)
        .map(|(effort, bias)| effort - bias)
        .collect();
    for (joint, torque) in held.iter().enumerate() {
        if let Some(torque) = torque {
            generalized_force[base + joint] = torque - frame.bias[base + joint];
        }
    }
    if let Some(pd) = body.pd {
        fold_implicit_pd(
            body.model,
            body.q,
            body.qd,
            dt,
            pd,
            held,
            &mut effective_mass,
            &mut generalized_force,
        );
    }
    let factor = Cholesky::new(&effective_mass, body.model.dof_parents.as_deref())
        .ok_or(DynamicsError::SingularMassMatrix)?;
    let impulse_rhs: Vec<f64> = generalized_force.iter().map(|force| force * dt).collect();
    let delta = factor.solve(&impulse_rhs);
    let free = body.qd.iter().zip(&delta).map(|(v, dv)| v + dv).collect();
    Ok((factor, free))
}

/// Stacks `[tangent_1; tangent_2; normal]` rows for every contact (relative
/// velocity of `a` with respect to `b`) and the joint-limit blocks below them.
#[allow(clippy::type_complexity)]
fn coupled_jacobian(
    bodies: &[CoupledBody<'_>],
    frames: &[BodyFrame],
    contacts: &[CoupledContact],
    limits: &[(usize, LimitRow)],
    total_nv: usize,
) -> Result<(DenseMatrix, Vec<[Vec3; 3]>, Vec<f64>), DynamicsError> {
    let rows = 3 * (contacts.len() + limits.len());
    let mut jacobian = DenseMatrix::zeros(rows, total_nv);
    let mut contact_frames = Vec::with_capacity(contacts.len());
    let mut friction = Vec::with_capacity(contacts.len() + limits.len());
    for (index, contact) in contacts.iter().enumerate() {
        let frame = contact_frame(contact.normal_world);
        let sides = std::iter::once((contact.a, 1.0)).chain(contact.b.map(|b| (b, -1.0)));
        for (anchor, sign) in sides {
            let model = bodies[anchor.body].model;
            let body_frame = &frames[anchor.body];
            let nv = model.nv();
            let link_index =
                model
                    .kinematic
                    .link_index(anchor.link)
                    .ok_or(DynamicsError::Kinematics(
                        rne_robot::KinematicsError::UnknownLink(anchor.link),
                    ))?;
            let linear = point_linear_jacobian(
                model,
                body_frame.kinematics.transforms(),
                link_index,
                anchor.point_local_m,
            );
            for (axis, direction) in frame.iter().enumerate() {
                let row = 3 * index + axis;
                for column in 0..nv {
                    let value = direction.x * linear[column]
                        + direction.y * linear[nv + column]
                        + direction.z * linear[2 * nv + column];
                    if value != 0.0 {
                        let target = body_frame.offset + column;
                        jacobian.set(row, target, jacobian.get(row, target) + sign * value);
                    }
                }
            }
        }
        friction.push(contact.friction_coefficient);
        contact_frames.push(frame);
    }
    for (index, (owner, limit)) in limits.iter().enumerate() {
        let sign = match limit.side {
            JointLimitSide::Lower => 1.0,
            JointLimitSide::Upper => -1.0,
        };
        jacobian.set(
            3 * (contacts.len() + index) + 2,
            frames[*owner].offset + limit.dof,
            sign,
        );
        friction.push(0.0);
    }
    Ok((jacobian, contact_frames, friction))
}

/// `M̃⁻¹ Jᵀ` columns (solved block by block) and the Delassus matrix.
fn block_delassus(
    factors: &[Cholesky],
    frames: &[BodyFrame],
    bodies: &[CoupledBody<'_>],
    jacobian: &DenseMatrix,
) -> (Vec<Vec<f64>>, DenseMatrix) {
    let rows = jacobian.rows();
    let total_nv = jacobian.cols();
    let data = jacobian.data();
    let row_of = |row: usize| &data[row * total_nv..(row + 1) * total_nv];
    let nonzero: Vec<bool> = (0..rows)
        .map(|row| row_of(row).iter().any(|value| *value != 0.0))
        .collect();
    let response: Vec<Vec<f64>> = (0..rows)
        .map(|row| {
            let mut column = vec![0.0; total_nv];
            if !nonzero[row] {
                return column;
            }
            let full = row_of(row);
            for ((factor, frame), body) in factors.iter().zip(frames).zip(bodies) {
                let range = frame.offset..frame.offset + body.model.nv();
                let block = &full[range.clone()];
                if block.iter().any(|value| *value != 0.0) {
                    column[range].copy_from_slice(&factor.solve(block));
                }
            }
            column
        })
        .collect();
    let mut delassus = DenseMatrix::zeros(rows, rows);
    for row in 0..rows {
        if !nonzero[row] {
            continue;
        }
        let jacobian_row = row_of(row);
        for col in row..rows {
            if !nonzero[col] {
                continue;
            }
            let value: f64 = jacobian_row
                .iter()
                .zip(&response[col])
                .map(|(a, b)| a * b)
                .sum();
            delassus.set(row, col, value);
            delassus.set(col, row, value);
        }
    }
    (response, delassus)
}
