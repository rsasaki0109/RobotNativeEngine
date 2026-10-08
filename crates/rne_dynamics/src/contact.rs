//! Hard-contact time stepping with an exact Coulomb friction cone.
//!
//! This module follows the per-contact iteration scheme of Hwangbo, Lee and
//! Hutter, "Per-Contact Iteration Method for Solving Contact Dynamics"
//! (IEEE RA-L 2018), the contact solver behind RaiSim. Contacts are resolved in
//! impulse space through the Delassus matrix `G = J M⁻¹ Jᵀ`. Each sweep visits
//! the contacts in a fixed order and solves one contact exactly while the
//! others are held, which yields one of three cases:
//!
//! * **open** — the point separates without an impulse;
//! * **sticking** — the impulse that stops the point lies inside the cone;
//! * **sliding** — the impulse lies on the cone boundary, the normal velocity
//!   vanishes, and the tangential velocity opposes the friction impulse
//!   (maximum dissipation). The direction on the boundary is found by
//!   bisection on its angle.
//!
//! The cone is never linearized into a pyramid, and the normal constraint is a
//! hard (Signorini) condition rather than a penalty spring, so stacks of
//! contacts neither creep nor need stiffness tuning.
//!
//! [`contact_step`] wraps the solver into a semi-implicit Euler step of an
//! [`ArticulatedModel`]. Joint PD control can be integrated implicitly in the
//! same step, the way RaiSim treats its generalized-coordinate PD controller:
//! the gains are folded into the effective mass `M + dt Kd + dt² Kp`, so very
//! stiff position control stays stable at ordinary step sizes.
//!
//! Every routine here is a pure, deterministic function: contacts are visited
//! in input order, no hash order participates, and no wall-clock time is read.

use crate::algorithms::{
    mass_matrix_from_xup, point_linear_jacobian, rnea_from_xup, xup_transforms, DenseMatrix,
};
use crate::model::{ArticulatedModel, DynamicsError};
use crate::spatial::{inverse_transform, rotation_matrix};
use rne_ecs::Entity;
use rne_math::{Quat, Vec3};
use rne_world::Transform3;

/// Number of evenly spaced angles sampled to bracket the sliding direction.
const SLIDING_BRACKET_SAMPLES: usize = 32;

/// Re-solves allowed per step while joints newly reach their effort limit.
const MAX_EFFORT_PASSES: usize = 4;

/// Relative slack before an actuator torque counts as over its effort limit.
const EFFORT_TOLERANCE: f64 = 1.0e-9;

/// Distance to a joint limit, in the joint's units, inside which the limit
/// always takes part in a step.
const LIMIT_ACTIVATION_MARGIN: f64 = 1.0e-3;

/// Relative slack accepted when testing a sticking impulse against the cone.
const CONE_TOLERANCE: f64 = 1.0e-12;

/// A candidate point contact between a robot link and its environment.
///
/// The normal points from the environment into the robot, which is the
/// direction a compressive contact impulse pushes the link. The gap is the
/// signed distance along that normal: positive while the point is still
/// approaching the surface and negative once it has penetrated. A positive gap
/// lets the point close the distance within one step before the contact acts,
/// so contacts can be handed to [`contact_step`] slightly before they touch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactPoint {
    /// Link that owns the contact point.
    pub link: Entity,
    /// Contact position in the link frame, in meters.
    pub point_local_m: Vec3,
    /// Unit contact normal in the world frame, pointing into the robot.
    pub normal_world: Vec3,
    /// Signed distance to the surface along the normal, in meters.
    pub gap_m: f64,
    /// Coulomb friction coefficient (dimensionless, non-negative).
    pub friction_coefficient: f64,
}

/// Iteration controls for [`solve_contact_impulses`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactSolverConfig {
    /// Maximum number of Gauss-Seidel sweeps over all contacts.
    pub max_iterations: usize,
    /// Convergence threshold on the largest per-contact impulse change in one
    /// sweep, in newton-seconds.
    pub tolerance_n_s: f64,
    /// Relaxation factor in `(0, 1]` blending each new contact impulse with
    /// the previous one. `1.0` is plain Gauss-Seidel.
    pub relaxation: f64,
    /// Bisection steps used to locate the sliding direction on the cone.
    pub bisection_iterations: usize,
}

impl Default for ContactSolverConfig {
    fn default() -> Self {
        Self {
            max_iterations: 200,
            tolerance_n_s: 1.0e-10,
            relaxation: 1.0,
            bisection_iterations: 60,
        }
    }
}

/// Resolved state of one contact.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum ContactState {
    /// The point separates and carries no impulse.
    Open,
    /// The point is held at rest by an impulse strictly inside the cone.
    Sticking,
    /// The point slides with its impulse on the friction cone boundary. A
    /// frictionless contact that pushes is always reported as sliding.
    Sliding,
}

/// Contact impulses produced by [`solve_contact_impulses`].
#[derive(Clone, Debug, PartialEq)]
pub struct ContactSolution {
    /// Impulse of each contact in its contact frame `[tangent_1, tangent_2,
    /// normal]`, in newton-seconds.
    pub impulses_n_s: Vec<[f64; 3]>,
    /// Resolved state of each contact.
    pub states: Vec<ContactState>,
    /// Number of sweeps performed.
    pub iterations: usize,
    /// Whether the impulse change dropped below the tolerance.
    pub converged: bool,
    /// Largest per-contact impulse change in the last sweep, in newton-seconds.
    pub last_change_n_s: f64,
}

/// Solves hard contact with an exact Coulomb cone by per-contact iteration.
///
/// `delassus` is the `3n x 3n` matrix `J M⁻¹ Jᵀ` with each contact's rows
/// ordered `[tangent_1, tangent_2, normal]`, `free_velocity_m_s` is the `3n`
/// contact-frame velocity the points would reach without contact impulses,
/// and `friction_coefficients` holds one coefficient per contact. The returned
/// impulses `λ` satisfy, per contact,
///
/// * `λₙ ≥ 0`, `vₙ ≥ 0`, `λₙ vₙ = 0` with `v = free_velocity + G λ`;
/// * `‖λₜ‖ ≤ μ λₙ`, and a sliding contact's tangential velocity opposes `λₜ`.
///
/// Contacts are swept in index order, so the result is deterministic.
/// `warm_start_n_s` optionally seeds the impulses, which typically reduces the
/// sweep count when consecutive steps share their contacts.
pub fn solve_contact_impulses(
    delassus: &DenseMatrix,
    free_velocity_m_s: &[f64],
    friction_coefficients: &[f64],
    config: &ContactSolverConfig,
    warm_start_n_s: Option<&[[f64; 3]]>,
) -> Result<ContactSolution, DynamicsError> {
    let count = friction_coefficients.len();
    let size = 3 * count;
    if delassus.rows() != size || delassus.cols() != size {
        return Err(DynamicsError::DimensionMismatch {
            provided: delassus.rows(),
            expected: size,
        });
    }
    if free_velocity_m_s.len() != size {
        return Err(DynamicsError::DimensionMismatch {
            provided: free_velocity_m_s.len(),
            expected: size,
        });
    }
    if delassus.data().iter().any(|value| !value.is_finite())
        || free_velocity_m_s.iter().any(|value| !value.is_finite())
    {
        return Err(DynamicsError::NonFiniteInput);
    }
    if friction_coefficients
        .iter()
        .any(|mu| !mu.is_finite() || *mu < 0.0)
    {
        return Err(DynamicsError::InvalidContact(
            "friction coefficients must be finite and non-negative",
        ));
    }
    if !(config.relaxation > 0.0 && config.relaxation <= 1.0) {
        return Err(DynamicsError::InvalidContact(
            "solver relaxation must lie in (0, 1]",
        ));
    }

    let mut impulses = match warm_start_n_s {
        Some(seed) => {
            if seed.len() != count {
                return Err(DynamicsError::DimensionMismatch {
                    provided: seed.len(),
                    expected: count,
                });
            }
            seed.iter()
                .zip(friction_coefficients)
                .map(|(impulse, mu)| project_onto_cone(*impulse, *mu))
                .collect()
        }
        None => vec![[0.0; 3]; count],
    };
    let mut states = vec![ContactState::Open; count];
    let blocks: Vec<[[f64; 3]; 3]> = (0..count)
        .map(|contact| diagonal_block(delassus, contact))
        .collect();

    let mut iterations = 0;
    let mut last_change = 0.0;
    let mut converged = count == 0;
    while !converged && iterations < config.max_iterations {
        iterations += 1;
        last_change = 0.0_f64;
        for contact in 0..count {
            // Velocity of this contact with its own impulse removed.
            let mut bias = [0.0; 3];
            for (axis, value) in bias.iter_mut().enumerate() {
                let row = 3 * contact + axis;
                let mut sum = free_velocity_m_s[row];
                for (other, impulse) in impulses.iter().enumerate() {
                    if other == contact {
                        continue;
                    }
                    for (component, magnitude) in impulse.iter().enumerate() {
                        sum += delassus.get(row, 3 * other + component) * magnitude;
                    }
                }
                *value = sum;
            }
            let (local, state) = solve_single_contact(
                &blocks[contact],
                bias,
                friction_coefficients[contact],
                config.bisection_iterations,
            );
            let previous = impulses[contact];
            let blended = [
                previous[0] + config.relaxation * (local[0] - previous[0]),
                previous[1] + config.relaxation * (local[1] - previous[1]),
                previous[2] + config.relaxation * (local[2] - previous[2]),
            ];
            let change = ((blended[0] - previous[0]).powi(2)
                + (blended[1] - previous[1]).powi(2)
                + (blended[2] - previous[2]).powi(2))
            .sqrt();
            last_change = last_change.max(change);
            impulses[contact] = blended;
            states[contact] = state;
        }
        converged = last_change <= config.tolerance_n_s;
    }

    Ok(ContactSolution {
        impulses_n_s: impulses,
        states,
        iterations,
        converged,
        last_change_n_s: last_change,
    })
}

fn diagonal_block(delassus: &DenseMatrix, contact: usize) -> [[f64; 3]; 3] {
    let mut block = [[0.0; 3]; 3];
    for (row, values) in block.iter_mut().enumerate() {
        for (col, value) in values.iter_mut().enumerate() {
            *value = delassus.get(3 * contact + row, 3 * contact + col);
        }
    }
    block
}

fn project_onto_cone(impulse: [f64; 3], mu: f64) -> [f64; 3] {
    let normal = impulse[2].max(0.0);
    let tangential = impulse[0].hypot(impulse[1]);
    let limit = mu * normal;
    if tangential <= limit || tangential == 0.0 {
        [impulse[0], impulse[1], normal]
    } else {
        let scale = limit / tangential;
        [impulse[0] * scale, impulse[1] * scale, normal]
    }
}

/// Solves one contact `v = G λ + b` exactly against the Coulomb cone.
fn solve_single_contact(
    g: &[[f64; 3]; 3],
    b: [f64; 3],
    mu: f64,
    bisection_iterations: usize,
) -> ([f64; 3], ContactState) {
    // A separating contact needs no impulse.
    if b[2] >= 0.0 {
        return ([0.0; 3], ContactState::Open);
    }
    if mu == 0.0 || g[2][2] <= 0.0 {
        let normal = if g[2][2] > 0.0 { -b[2] / g[2][2] } else { 0.0 };
        if normal > 0.0 {
            return ([0.0, 0.0, normal], ContactState::Sliding);
        }
        return ([0.0; 3], ContactState::Open);
    }

    // Sticking: the impulse that brings the contact to rest.
    let sticking = solve3(g, [-b[0], -b[1], -b[2]]);
    if let Some(stick) = sticking {
        let tangential = stick[0].hypot(stick[1]);
        if stick[2] > 0.0 && tangential <= mu * stick[2] * (1.0 + CONE_TOLERANCE) {
            return (stick, ContactState::Sticking);
        }
    }

    // Sliding: λ = λₙ (μ cos θ, μ sin θ, 1) with vₙ = 0, and θ chosen so the
    // tangential velocity is anti-parallel to the friction impulse.
    let seed_angle = match sticking {
        Some(stick) if stick[0].hypot(stick[1]) > 0.0 => stick[1].atan2(stick[0]),
        _ => (-b[1]).atan2(-b[0]),
    };
    let evaluate = |angle: f64| sliding_residual(g, b, mu, angle);

    let step = std::f64::consts::TAU / SLIDING_BRACKET_SAMPLES as f64;
    let samples: Vec<Option<SlidingSample>> = (0..=SLIDING_BRACKET_SAMPLES)
        .map(|index| evaluate(seed_angle + step * index as f64))
        .collect();
    // Prefer the bracket nearest the sticking direction, searching outward in
    // both directions in a fixed order.
    let mut order = Vec::with_capacity(SLIDING_BRACKET_SAMPLES);
    for offset in 0..SLIDING_BRACKET_SAMPLES.div_ceil(2) {
        order.push(offset);
        let mirrored = SLIDING_BRACKET_SAMPLES - 1 - offset;
        if mirrored != offset {
            order.push(mirrored);
        }
    }
    for index in order {
        let (Some(low), Some(high)) = (samples[index], samples[index + 1]) else {
            continue;
        };
        if low.cross.signum() == high.cross.signum() && low.cross != 0.0 && high.cross != 0.0 {
            continue;
        }
        let mut low_angle = seed_angle + step * index as f64;
        let mut high_angle = low_angle + step;
        let mut low_cross = low.cross;
        let mut best = if low.cross.abs() <= high.cross.abs() {
            low
        } else {
            high
        };
        for _ in 0..bisection_iterations {
            let mid_angle = 0.5 * (low_angle + high_angle);
            let Some(mid) = evaluate(mid_angle) else {
                break;
            };
            best = mid;
            if mid.cross == 0.0 {
                break;
            }
            if mid.cross.signum() == low_cross.signum() {
                low_angle = mid_angle;
                low_cross = mid.cross;
            } else {
                high_angle = mid_angle;
            }
        }
        // A sign change can also mark the anti-dissipative direction, where the
        // tangential velocity is parallel to the impulse; reject it.
        if best.alignment > 0.0 {
            return (best.impulse, ContactState::Sliding);
        }
    }

    // No valid bracket (degenerate geometry): fall back to the projected
    // sticking impulse, which is still admissible.
    match sticking {
        Some(stick) => {
            let projected = project_onto_cone(stick, mu);
            if projected[2] > 0.0 {
                (projected, ContactState::Sliding)
            } else {
                ([0.0; 3], ContactState::Open)
            }
        }
        None => ([0.0; 3], ContactState::Open),
    }
}

#[derive(Clone, Copy, Debug)]
struct SlidingSample {
    impulse: [f64; 3],
    /// Signed sine between the impulse direction and the opposed tangential
    /// velocity; zero at the sliding solution.
    cross: f64,
    /// Cosine-like alignment between the impulse direction and the opposed
    /// tangential velocity; positive at the dissipative solution.
    alignment: f64,
}

fn sliding_residual(g: &[[f64; 3]; 3], b: [f64; 3], mu: f64, angle: f64) -> Option<SlidingSample> {
    let (sin, cos) = angle.sin_cos();
    let direction = [mu * cos, mu * sin, 1.0];
    let denominator = g[2][0] * direction[0] + g[2][1] * direction[1] + g[2][2];
    if denominator <= 0.0 {
        return None;
    }
    let normal = -b[2] / denominator;
    if normal <= 0.0 {
        return None;
    }
    let impulse = [normal * direction[0], normal * direction[1], normal];
    let opposed = [
        -(g[0][0] * impulse[0] + g[0][1] * impulse[1] + g[0][2] * impulse[2] + b[0]),
        -(g[1][0] * impulse[0] + g[1][1] * impulse[1] + g[1][2] * impulse[2] + b[1]),
    ];
    Some(SlidingSample {
        impulse,
        cross: cos * opposed[1] - sin * opposed[0],
        alignment: cos * opposed[0] + sin * opposed[1],
    })
}

fn solve3(g: &[[f64; 3]; 3], rhs: [f64; 3]) -> Option<[f64; 3]> {
    let det = g[0][0] * (g[1][1] * g[2][2] - g[1][2] * g[2][1])
        - g[0][1] * (g[1][0] * g[2][2] - g[1][2] * g[2][0])
        + g[0][2] * (g[1][0] * g[2][1] - g[1][1] * g[2][0]);
    if det.abs() <= f64::EPSILON * (g[0][0].abs() + g[1][1].abs() + g[2][2].abs()).powi(3) {
        return None;
    }
    let mut out = [0.0; 3];
    for (column, value) in out.iter_mut().enumerate() {
        let mut replaced = *g;
        for row in 0..3 {
            replaced[row][column] = rhs[row];
        }
        let det_column = replaced[0][0]
            * (replaced[1][1] * replaced[2][2] - replaced[1][2] * replaced[2][1])
            - replaced[0][1] * (replaced[1][0] * replaced[2][2] - replaced[1][2] * replaced[2][0])
            + replaced[0][2] * (replaced[1][0] * replaced[2][1] - replaced[1][1] * replaced[2][0]);
        *value = det_column / det;
    }
    Some(out)
}

/// Joint PD control integrated implicitly by [`contact_step`].
///
/// Each vector holds one entry per actuated joint coordinate, that is the
/// model's generalized coordinates without the floating-base rows. Gains are in
/// N·m/rad and N·m·s/rad for revolute joints and N/m and N·s/m for prismatic
/// joints. The PD force `Kp (q* - q⁺) + Kd (qd* - qd⁺)` is evaluated at the end
/// of the step, which is unconditionally stable for non-negative gains.
#[derive(Clone, Debug, PartialEq)]
pub struct JointPdControl {
    /// Proportional gain per actuated joint.
    pub position_gains: Vec<f64>,
    /// Derivative gain per actuated joint.
    pub velocity_gains: Vec<f64>,
    /// Position target per actuated joint (rad or m).
    pub target_positions: Vec<f64>,
    /// Velocity target per actuated joint (rad/s or m/s).
    pub target_velocities: Vec<f64>,
    /// Actuator effort limit per actuated joint (N·m or N), applied with the
    /// model's own limit ([`ArticulatedModel::joint_effort_limit`]) when
    /// [`ContactStepConfig::enforce_effort_limits`] is set: the tighter of the
    /// two holds. Empty, or an infinite entry, leaves the model's limit alone.
    pub effort_limits: Vec<f64>,
}

/// Configuration of one [`contact_step`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactStepConfig {
    /// Integration step in seconds.
    pub step_time_s: f64,
    /// Fraction in `[0, 1]` of a penetration removed per step.
    pub penetration_recovery: f64,
    /// Contact solver iteration controls.
    pub solver: ContactSolverConfig,
    /// Whether bounded revolute and prismatic joints stop at their position
    /// limits ([`ArticulatedModel::joint_position_limits`]).
    pub enforce_joint_limits: bool,
    /// Whether actuator torque (feed-forward plus PD) is clamped to each
    /// joint's effort limit ([`ArticulatedModel::joint_effort_limit`]).
    pub enforce_effort_limits: bool,
}

impl Default for ContactStepConfig {
    fn default() -> Self {
        Self {
            step_time_s: 0.002,
            penetration_recovery: 0.2,
            solver: ContactSolverConfig::default(),
            enforce_joint_limits: true,
            enforce_effort_limits: true,
        }
    }
}

/// Which end of a joint's range a [`JointLimitOutcome`] refers to.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum JointLimitSide {
    /// The lower position limit; its impulse pushes the coordinate upward.
    Lower,
    /// The upper position limit; its impulse pushes the coordinate downward.
    Upper,
}

/// A joint limit that carried an impulse during a [`contact_step`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointLimitOutcome {
    /// Velocity coordinate of the limited joint.
    pub dof: usize,
    /// Which limit was engaged.
    pub side: JointLimitSide,
    /// Magnitude of the generalized limit impulse, in N·m·s (revolute) or N·s
    /// (prismatic). Divide by the step time for the mean limit torque or force.
    pub impulse: f64,
}

/// Result of one contact of a [`contact_step`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactOutcome {
    /// Contact impulse applied to the robot in the world frame, in
    /// newton-seconds. Divide by the step time for the mean contact force.
    pub impulse_world_n_s: Vec3,
    /// Resolved contact state.
    pub state: ContactState,
}

/// Result of [`contact_step`].
#[derive(Clone, Debug, PartialEq)]
pub struct ContactStep {
    /// Configuration at the end of the step.
    pub q: Vec<f64>,
    /// Generalized velocity at the end of the step.
    pub qd: Vec<f64>,
    /// Per-contact impulses and states, in input order.
    pub contacts: Vec<ContactOutcome>,
    /// Joint limits that carried an impulse, in ascending coordinate order.
    pub joint_limits: Vec<JointLimitOutcome>,
    /// Actuator torque applied to each actuated joint over the step
    /// (feed-forward plus implicit PD, after effort clamping), in N·m or N.
    /// Entries follow the velocity coordinates after the floating base.
    pub actuator_torques: Vec<f64>,
    /// Actuated joint indices (into `actuator_torques`) held at their effort
    /// limit this step.
    pub saturated_joints: Vec<usize>,
    /// Sweeps used by the contact solver.
    pub solver_iterations: usize,
    /// Whether the contact solver converged within its sweep budget.
    pub solver_converged: bool,
}

/// Advances an articulated model one step with hard contacts and implicit PD.
///
/// The step is semi-implicit Euler in the RaiSim form:
///
/// ```text
/// M̃ = M + dt Kd + dt² Kp
/// v_free = qd + M̃⁻¹ dt (tau + Kp (q* - q - dt qd) + Kd (qd* - qd) - h)
/// qd⁺ = v_free + M̃⁻¹ Jᵀ λ,   q⁺ = q ⊕ dt qd⁺
/// ```
///
/// where `λ` comes from [`solve_contact_impulses`] on `G = J M̃⁻¹ Jᵀ`. A
/// contact with a positive gap may close it within the step before it pushes;
/// a penetrating contact is driven out at `penetration_recovery` of its depth
/// per step. With `enforce_joint_limits`, every finite joint position limit is
/// one more unilateral, frictionless constraint in the same solve, acting on
/// the joint coordinate itself with the distance to the limit as its gap, so
/// a joint stops at its limit however stiffly it is driven. `tau` holds one
/// generalized force per velocity coordinate (including floating-base rows),
/// and `warm_start_n_s` optionally seeds the contact impulses from a previous
/// step; joint-limit impulses always start from zero.
#[allow(clippy::too_many_arguments)]
pub fn contact_step(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    contacts: &[ContactPoint],
    pd: Option<&JointPdControl>,
    config: &ContactStepConfig,
    warm_start_n_s: Option<&[[f64; 3]]>,
) -> Result<ContactStep, DynamicsError> {
    let nv = model.nv();
    for values in [q, qd, tau] {
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
    let dt = config.step_time_s;
    if !(dt.is_finite() && dt > 0.0) {
        return Err(DynamicsError::InvalidContact("step time must be positive"));
    }
    if !(0.0..=1.0).contains(&config.penetration_recovery) {
        return Err(DynamicsError::InvalidContact(
            "penetration recovery must lie in [0, 1]",
        ));
    }

    if let Some(pd) = pd {
        validate_pd(model, pd)?;
    }

    // One forward-kinematics pass serves the mass matrix, the bias forces, the
    // contact Jacobians, and the base integration of this step.
    let kinematics = model.kinematic.forward_kinematics(q)?;
    let transforms = kinematics.transforms();
    let xup = xup_transforms(model, transforms);
    let mass = mass_matrix_from_xup(model, &xup);
    let bias = rnea_from_xup(model, transforms, &xup, qd, &vec![0.0; nv]);
    let frame = StepFrame {
        transforms,
        mass: &mass,
        bias: &bias,
    };

    // Joints whose actuator torque would exceed the effort limit are held at
    // the limit (with their PD taken out of the implicit solve) and the step
    // is solved again; a few passes settle which joints saturate.
    let actuated = nv - model.base_dof();
    let mut held: Vec<Option<f64>> = vec![None; actuated];
    let mut pass = 0;
    loop {
        let mut step = step_with_actuation(
            model,
            q,
            qd,
            tau,
            contacts,
            pd,
            &held,
            config,
            warm_start_n_s,
            &frame,
        )?;
        let torques = actuator_torques(model, q, tau, pd, &held, dt, &step.qd);
        let mut changed = false;
        if config.enforce_effort_limits && pass < MAX_EFFORT_PASSES {
            for (joint, torque) in torques.iter().enumerate() {
                if held[joint].is_some() {
                    continue;
                }
                if let Some(limit) = effort_limit(model, pd, joint) {
                    if torque.abs() > limit * (1.0 + EFFORT_TOLERANCE) {
                        held[joint] = Some(torque.clamp(-limit, limit));
                        changed = true;
                    }
                }
            }
        }
        if !changed {
            step.actuator_torques = torques;
            step.saturated_joints = (0..actuated).filter(|&j| held[j].is_some()).collect();
            return Ok(step);
        }
        pass += 1;
    }
}

/// Quantities of a step that do not depend on the actuator torques.
struct StepFrame<'a> {
    transforms: &'a [Transform3],
    mass: &'a DenseMatrix,
    bias: &'a [f64],
}

/// Actuator torque of each actuated joint at the end of a step.
#[allow(clippy::too_many_arguments)]
fn actuator_torques(
    model: &ArticulatedModel,
    q: &[f64],
    tau: &[f64],
    pd: Option<&JointPdControl>,
    held: &[Option<f64>],
    dt: f64,
    next_velocity: &[f64],
) -> Vec<f64> {
    let base = model.base_dof();
    held.iter()
        .enumerate()
        .map(|(joint, held)| {
            if let Some(torque) = held {
                return *torque;
            }
            let dof = base + joint;
            let feedback = pd.map_or(0.0, |pd| {
                let next_position = q[dof] + dt * next_velocity[dof];
                pd.position_gains[joint] * (pd.target_positions[joint] - next_position)
                    + pd.velocity_gains[joint] * (pd.target_velocities[joint] - next_velocity[dof])
            });
            tau[dof] + feedback
        })
        .collect()
}

/// One semi-implicit solve with some joints held at a fixed actuator torque.
#[allow(clippy::too_many_arguments)]
fn step_with_actuation(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
    tau: &[f64],
    contacts: &[ContactPoint],
    pd: Option<&JointPdControl>,
    held: &[Option<f64>],
    config: &ContactStepConfig,
    warm_start_n_s: Option<&[[f64; 3]]>,
    frame: &StepFrame<'_>,
) -> Result<ContactStep, DynamicsError> {
    let base = model.base_dof();
    let dt = config.step_time_s;
    let transforms = frame.transforms;
    let mut effective_mass = frame.mass.clone();
    let mut generalized_force: Vec<f64> = tau
        .iter()
        .zip(frame.bias)
        .map(|(effort, bias)| effort - bias)
        .collect();
    for (joint, torque) in held.iter().enumerate() {
        if let Some(torque) = torque {
            generalized_force[base + joint] = torque - frame.bias[base + joint];
        }
    }
    if let Some(pd) = pd {
        fold_implicit_pd(
            model,
            q,
            qd,
            dt,
            pd,
            held,
            &mut effective_mass,
            &mut generalized_force,
        );
    }
    let factor = Cholesky::new(&effective_mass, model.dof_parents.as_deref())
        .ok_or(DynamicsError::SingularMassMatrix)?;

    let impulse_rhs: Vec<f64> = generalized_force.iter().map(|force| force * dt).collect();
    let delta = factor.solve(&impulse_rhs);
    let free_velocity: Vec<f64> = qd.iter().zip(&delta).map(|(v, dv)| v + dv).collect();

    let count = contacts.len();
    let (contact_rows, frames, mut friction) = contact_jacobian(model, transforms, contacts)?;
    let limits = if config.enforce_joint_limits {
        active_joint_limits(model, q, &free_velocity, dt)
    } else {
        Vec::new()
    };
    let blocks = count + limits.len();
    let jacobian = stack_joint_limits(&contact_rows, &limits, &mut friction);

    let (response, delassus) = delassus_operator(&factor, &jacobian);
    let mut contact_velocity = jacobian.mul_vec(&free_velocity);
    for (index, contact) in contacts.iter().enumerate() {
        let allowed_approach = if contact.gap_m >= 0.0 {
            contact.gap_m / dt
        } else {
            config.penetration_recovery * contact.gap_m / dt
        };
        contact_velocity[3 * index + 2] += allowed_approach;
    }
    for (index, limit) in limits.iter().enumerate() {
        let allowed_approach = if limit.gap >= 0.0 {
            limit.gap / dt
        } else {
            config.penetration_recovery * limit.gap / dt
        };
        contact_velocity[3 * (count + index) + 2] += allowed_approach;
    }
    let warm_start: Option<Vec<[f64; 3]>> = match warm_start_n_s {
        Some(seed) if seed.len() != count => {
            return Err(DynamicsError::DimensionMismatch {
                provided: seed.len(),
                expected: count,
            });
        }
        Some(seed) => {
            let mut full = seed.to_vec();
            full.resize(blocks, [0.0; 3]);
            Some(full)
        }
        None => None,
    };

    let solution = solve_contact_impulses(
        &delassus,
        &contact_velocity,
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
    let next_configuration =
        integrate_from_base_frame(model, q, &next_velocity, dt, &transforms[0]);

    let joint_limits = limits
        .iter()
        .zip(&solution.impulses_n_s[count..])
        .filter(|(_, impulse)| impulse[2] > 0.0)
        .map(|(limit, impulse)| JointLimitOutcome {
            dof: limit.dof,
            side: limit.side,
            impulse: impulse[2],
        })
        .collect();
    let outcomes = solution.impulses_n_s[..count]
        .iter()
        .zip(&solution.states)
        .zip(&frames)
        .map(|((impulse, state), frame)| ContactOutcome {
            impulse_world_n_s: frame[0] * impulse[0]
                + frame[1] * impulse[1]
                + frame[2] * impulse[2],
            state: *state,
        })
        .collect();

    Ok(ContactStep {
        q: next_configuration,
        qd: next_velocity,
        contacts: outcomes,
        joint_limits,
        actuator_torques: Vec::new(),
        saturated_joints: Vec::new(),
        solver_iterations: solution.iterations,
        solver_converged: solution.converged,
    })
}

/// Appends one frictionless block per joint limit below the contact rows: its
/// normal row is `±1` on the joint coordinate and its tangent rows are empty.
fn stack_joint_limits(
    contact_rows: &DenseMatrix,
    limits: &[LimitRow],
    friction: &mut Vec<f64>,
) -> DenseMatrix {
    let nv = contact_rows.cols();
    let contact_count = contact_rows.rows();
    let mut jacobian = DenseMatrix::zeros(contact_count + 3 * limits.len(), nv);
    for row in 0..contact_count {
        for dof in 0..nv {
            jacobian.set(row, dof, contact_rows.get(row, dof));
        }
    }
    for (index, limit) in limits.iter().enumerate() {
        let sign = match limit.side {
            JointLimitSide::Lower => 1.0,
            JointLimitSide::Upper => -1.0,
        };
        jacobian.set(contact_count + 3 * index + 2, limit.dof, sign);
        friction.push(0.0);
    }
    jacobian
}

/// Returns the columns of `M̃⁻¹ Jᵀ` (one per constraint row, zero for empty
/// rows) and the symmetric Delassus matrix `J M̃⁻¹ Jᵀ`.
fn delassus_operator(factor: &Cholesky, jacobian: &DenseMatrix) -> (Vec<Vec<f64>>, DenseMatrix) {
    let rows = jacobian.rows();
    let nv = jacobian.cols();
    let data = jacobian.data();
    let row_of = |row: usize| &data[row * nv..(row + 1) * nv];
    let nonzero: Vec<bool> = (0..rows)
        .map(|row| row_of(row).iter().any(|value| *value != 0.0))
        .collect();
    let response: Vec<Vec<f64>> = (0..rows)
        .map(|row| {
            if nonzero[row] {
                factor.solve(row_of(row))
            } else {
                vec![0.0; nv]
            }
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

/// A joint position limit considered by one step.
struct LimitRow {
    dof: usize,
    side: JointLimitSide,
    /// Distance to the limit in the joint's units; negative past the limit.
    gap: f64,
}

/// Joint limits a step can reach, lower before upper, in coordinate order.
///
/// A limit is included when it lies within [`LIMIT_ACTIVATION_MARGIN`] or
/// within twice the distance the unconstrained motion covers toward it in one
/// step. Limits further away cannot carry an impulse this step, and leaving
/// them out keeps the Delassus matrix small; a joint pushed past an excluded
/// limit by other impulses is caught and recovered on the next step.
fn active_joint_limits(
    model: &ArticulatedModel,
    q: &[f64],
    free_velocity: &[f64],
    dt: f64,
) -> Vec<LimitRow> {
    let mut rows = Vec::new();
    for (dof, position) in q.iter().enumerate().skip(model.base_dof()) {
        let Some((lower, upper)) = model.joint_position_limits(dof) else {
            continue;
        };
        let velocity = free_velocity[dof];
        let lower_gap = position - lower;
        if lower_gap + 2.0 * dt * velocity.min(0.0) <= LIMIT_ACTIVATION_MARGIN {
            rows.push(LimitRow {
                dof,
                side: JointLimitSide::Lower,
                gap: lower_gap,
            });
        }
        let upper_gap = upper - position;
        if upper_gap - 2.0 * dt * velocity.max(0.0) <= LIMIT_ACTIVATION_MARGIN {
            rows.push(LimitRow {
                dof,
                side: JointLimitSide::Upper,
                gap: upper_gap,
            });
        }
    }
    rows
}

/// Checks PD vector lengths, finiteness, and gain signs.
fn validate_pd(model: &ArticulatedModel, pd: &JointPdControl) -> Result<(), DynamicsError> {
    let actuated = model.nv() - model.base_dof();
    for values in [
        &pd.position_gains,
        &pd.velocity_gains,
        &pd.target_positions,
        &pd.target_velocities,
    ] {
        if values.len() != actuated {
            return Err(DynamicsError::DimensionMismatch {
                provided: values.len(),
                expected: actuated,
            });
        }
        if values.iter().any(|value| !value.is_finite()) {
            return Err(DynamicsError::NonFiniteInput);
        }
    }
    if pd
        .position_gains
        .iter()
        .chain(&pd.velocity_gains)
        .any(|gain| *gain < 0.0)
    {
        return Err(DynamicsError::InvalidContact(
            "PD gains must be non-negative",
        ));
    }
    if !(pd.effort_limits.is_empty() || pd.effort_limits.len() == actuated) {
        return Err(DynamicsError::DimensionMismatch {
            provided: pd.effort_limits.len(),
            expected: actuated,
        });
    }
    if pd
        .effort_limits
        .iter()
        .any(|limit| limit.is_nan() || *limit < 0.0)
    {
        return Err(DynamicsError::InvalidContact(
            "effort limits must be non-negative",
        ));
    }
    Ok(())
}

/// Effective effort limit of actuated joint `joint`: the tighter of the
/// model's limit and the PD command's.
fn effort_limit(
    model: &ArticulatedModel,
    pd: Option<&JointPdControl>,
    joint: usize,
) -> Option<f64> {
    let model_limit = model.joint_effort_limit(model.base_dof() + joint);
    let command_limit = pd
        .and_then(|pd| pd.effort_limits.get(joint).copied())
        .filter(|limit| limit.is_finite());
    match (model_limit, command_limit) {
        (Some(model_limit), Some(command_limit)) => Some(model_limit.min(command_limit)),
        (model_limit, None) => model_limit,
        (None, command_limit) => command_limit,
    }
}

/// Folds implicit joint PD into the effective mass and generalized force,
/// skipping joints held at a fixed actuator torque.
#[allow(clippy::too_many_arguments)]
fn fold_implicit_pd(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
    dt: f64,
    pd: &JointPdControl,
    held: &[Option<f64>],
    effective_mass: &mut DenseMatrix,
    generalized_force: &mut [f64],
) {
    let base = model.base_dof();
    for (joint, held) in held.iter().enumerate() {
        if held.is_some() {
            continue;
        }
        let kp = pd.position_gains[joint];
        let kd = pd.velocity_gains[joint];
        let dof = base + joint;
        effective_mass.set(
            dof,
            dof,
            effective_mass.get(dof, dof) + dt * kd + dt * dt * kp,
        );
        generalized_force[dof] += kp * (pd.target_positions[joint] - q[dof] - dt * qd[dof])
            + kd * (pd.target_velocities[joint] - qd[dof]);
    }
}

/// Stacks the contact-frame Jacobians `[tangent_1; tangent_2; normal]` of all
/// contacts, returning them with each contact's frame and friction coefficient.
#[allow(clippy::type_complexity)]
fn contact_jacobian(
    model: &ArticulatedModel,
    transforms: &[Transform3],
    contacts: &[ContactPoint],
) -> Result<(DenseMatrix, Vec<[Vec3; 3]>, Vec<f64>), DynamicsError> {
    let nv = model.nv();
    let mut jacobian = DenseMatrix::zeros(3 * contacts.len(), nv);
    let mut frames = Vec::with_capacity(contacts.len());
    let mut friction = Vec::with_capacity(contacts.len());
    for (index, contact) in contacts.iter().enumerate() {
        if !contact.normal_world.is_finite()
            || contact.normal_world.length_squared() <= f64::EPSILON
            || !contact.gap_m.is_finite()
            || !contact.point_local_m.is_finite()
        {
            return Err(DynamicsError::InvalidContact(
                "contact normal, gap, and point must be finite and the normal non-zero",
            ));
        }
        let link_index =
            model
                .kinematic
                .link_index(contact.link)
                .ok_or(DynamicsError::Kinematics(
                    rne_robot::KinematicsError::UnknownLink(contact.link),
                ))?;
        let frame = contact_frame(contact.normal_world);
        let linear = point_linear_jacobian(model, transforms, link_index, contact.point_local_m);
        for (axis, direction) in frame.iter().enumerate() {
            for column in 0..nv {
                let value = direction.x * linear[column]
                    + direction.y * linear[nv + column]
                    + direction.z * linear[2 * nv + column];
                jacobian.set(3 * index + axis, column, value);
            }
        }
        friction.push(contact.friction_coefficient);
        frames.push(frame);
    }
    Ok((jacobian, frames, friction))
}

/// Integrates a configuration by one step of generalized velocity.
///
/// Joint coordinates advance as `q + dt qd`. A floating base advances on the
/// rigid-body manifold: the body-frame twist `qd[0..6]` moves the base frame,
/// and the resulting pose is mapped back to the `(x, y, z, roll, pitch, yaw)`
/// configuration, with roll and yaw unwrapped to stay continuous with `q`.
pub fn integrate_configuration(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
    dt: f64,
) -> Result<Vec<f64>, DynamicsError> {
    let nv = model.nv();
    for values in [q, qd] {
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
    if !dt.is_finite() {
        return Err(DynamicsError::NonFiniteInput);
    }
    let kinematics = model.kinematic.forward_kinematics(q)?;
    Ok(integrate_from_base_frame(
        model,
        q,
        qd,
        dt,
        &kinematics.transforms()[0],
    ))
}

/// [`integrate_configuration`] with the base link frame at `q` already known.
fn integrate_from_base_frame(
    model: &ArticulatedModel,
    q: &[f64],
    qd: &[f64],
    dt: f64,
    base_frame: &Transform3,
) -> Vec<f64> {
    let base = model.base_dof();
    let mut next: Vec<f64> = q.iter().zip(qd).map(|(q, qd)| q + dt * qd).collect();
    if base == 6 {
        let base_frame = *base_frame;
        let configuration_frame = floating_base_transform(q);
        // Fixed offset between the configured base pose and the base link frame.
        let offset = inverse_transform(&configuration_frame).mul_transform(&base_frame);

        let linear = Vec3::new(qd[0], qd[1], qd[2]);
        let angular = Vec3::new(qd[3], qd[4], qd[5]);
        let rotation = (base_frame.rotation * Quat::from_scaled_axis(angular * dt)).normalize();
        let translation = base_frame.translation + base_frame.rotation * linear * dt;
        let moved = Transform3::from_translation_rotation(translation, rotation);
        let configured = moved.mul_transform(&inverse_transform(&offset));

        // Recover fixed-axis roll-pitch-yaw from R = Rz(yaw) Ry(pitch) Rx(roll).
        let matrix = rotation_matrix(configured.rotation);
        let pitch = (-matrix[2][0]).clamp(-1.0, 1.0).asin();
        let roll = matrix[2][1].atan2(matrix[2][2]);
        let yaw = matrix[1][0].atan2(matrix[0][0]);
        next[0] = configured.translation.x;
        next[1] = configured.translation.y;
        next[2] = configured.translation.z;
        next[3] = unwrap_angle(roll, q[3]);
        next[4] = pitch;
        next[5] = unwrap_angle(yaw, q[5]);
    }
    next
}

fn floating_base_transform(q: &[f64]) -> Transform3 {
    let rotation =
        Quat::from_rotation_z(q[5]) * Quat::from_rotation_y(q[4]) * Quat::from_rotation_x(q[3]);
    Transform3::from_translation_rotation(Vec3::new(q[0], q[1], q[2]), rotation)
}

fn unwrap_angle(angle: f64, reference: f64) -> f64 {
    let turns = ((reference - angle) / std::f64::consts::TAU).round();
    angle + turns * std::f64::consts::TAU
}

/// Orthonormal contact frame `[tangent_1, tangent_2, normal]` for a normal.
///
/// The first tangent is built against the world axis least aligned with the
/// normal, so the frame is a deterministic function of the normal alone.
pub fn contact_frame(normal_world: Vec3) -> [Vec3; 3] {
    let normal = normal_world.normalize();
    let abs = normal.abs();
    let reference = if abs.x <= abs.y && abs.x <= abs.z {
        Vec3::X
    } else if abs.y <= abs.z {
        Vec3::Y
    } else {
        Vec3::Z
    };
    let tangent_1 = reference.cross(normal).normalize();
    let tangent_2 = normal.cross(tangent_1);
    [tangent_1, tangent_2, normal]
}

/// Cholesky-type factor of the (effective) mass matrix.
///
/// With the coordinate tree `λ` known, the factor is Featherstone's `LᵀL`
/// factorization, which produces no fill-in outside the branch-induced
/// sparsity of a mass matrix, so both factoring and solving only walk each
/// coordinate's ancestors. Without it, a dense `LLᵀ` factor is used.
struct Cholesky {
    size: usize,
    lower: Vec<f64>,
    parents: Option<Vec<Option<usize>>>,
}

impl Cholesky {
    fn new(matrix: &DenseMatrix, parents: Option<&[Option<usize>]>) -> Option<Self> {
        match parents {
            Some(parents) if parents.len() == matrix.rows() => Self::tree(matrix, parents),
            _ => Self::dense(matrix),
        }
    }

    /// `H = Lᵀ L` with `L` lower triangular, walking only ancestor chains
    /// (Featherstone, Rigid Body Dynamics Algorithms, Table 6.3).
    fn tree(matrix: &DenseMatrix, parents: &[Option<usize>]) -> Option<Self> {
        let n = matrix.rows();
        let mut h = matrix.data().to_vec();
        for k in (0..n).rev() {
            let diagonal = h[k * n + k];
            if diagonal <= 0.0 || !diagonal.is_finite() {
                return None;
            }
            let root = diagonal.sqrt();
            h[k * n + k] = root;
            let mut i = parents[k];
            while let Some(row) = i {
                h[k * n + row] /= root;
                i = parents[row];
            }
            let mut i = parents[k];
            while let Some(row) = i {
                let mut j = Some(row);
                while let Some(col) = j {
                    h[row * n + col] -= h[k * n + row] * h[k * n + col];
                    j = parents[col];
                }
                i = parents[row];
            }
        }
        Some(Self {
            size: n,
            lower: h,
            parents: Some(parents.to_vec()),
        })
    }

    fn dense(matrix: &DenseMatrix) -> Option<Self> {
        let size = matrix.rows();
        let mut lower = vec![0.0; size * size];
        for row in 0..size {
            for col in 0..=row {
                let mut sum = 0.5 * (matrix.get(row, col) + matrix.get(col, row));
                for k in 0..col {
                    sum -= lower[row * size + k] * lower[col * size + k];
                }
                if row == col {
                    if sum <= 0.0 || !sum.is_finite() {
                        return None;
                    }
                    lower[row * size + col] = sum.sqrt();
                } else {
                    lower[row * size + col] = sum / lower[col * size + col];
                }
            }
        }
        Some(Self {
            size,
            lower,
            parents: None,
        })
    }

    fn solve(&self, rhs: &[f64]) -> Vec<f64> {
        let n = self.size;
        let l = &self.lower;
        let mut x = rhs.to_vec();
        if let Some(parents) = &self.parents {
            // Lᵀ y = b, leaves to root.
            for i in (0..n).rev() {
                x[i] /= l[i * n + i];
                let mut j = parents[i];
                while let Some(col) = j {
                    x[col] -= l[i * n + col] * x[i];
                    j = parents[col];
                }
            }
            // L x = y, root to leaves.
            for i in 0..n {
                let mut j = parents[i];
                while let Some(col) = j {
                    x[i] -= l[i * n + col] * x[col];
                    j = parents[col];
                }
                x[i] /= l[i * n + i];
            }
            return x;
        }
        for row in 0..n {
            for k in 0..row {
                x[row] -= l[row * n + k] * x[k];
            }
            x[row] /= l[row * n + row];
        }
        for row in (0..n).rev() {
            for k in (row + 1)..n {
                x[row] -= l[k * n + row] * x[k];
            }
            x[row] /= l[row * n + row];
        }
        x
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_ecs::{spawn_named, World};
    use rne_physics::{RigidBody, RigidBodyInertia};
    use rne_robot::{FloatingBase, Joint, JointKind, JointLimits, Link, Robot, RobotId};

    const HALF_EXTENTS_M: [f64; 3] = [0.1, 0.05, 0.1];
    const BOX_MASS_KG: f64 = 2.0;

    /// Small deterministic generator for test matrices.
    struct Lcg(u64);

    impl Lcg {
        fn next(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((self.0 >> 11) as f64 / (1_u64 << 53) as f64) * 2.0 - 1.0
        }
    }

    fn random_spd(size: usize, rng: &mut Lcg) -> DenseMatrix {
        let factor: Vec<f64> = (0..size * size).map(|_| rng.next()).collect();
        let mut matrix = DenseMatrix::zeros(size, size);
        for row in 0..size {
            for col in 0..size {
                let value: f64 = (0..size)
                    .map(|k| factor[row * size + k] * factor[col * size + k])
                    .sum();
                let diagonal = if row == col { 0.5 } else { 0.0 };
                matrix.set(row, col, value + diagonal);
            }
        }
        matrix
    }

    /// Checks Signorini and Coulomb conditions of a converged solution.
    fn assert_contact_conditions(
        delassus: &DenseMatrix,
        free: &[f64],
        mu: &[f64],
        solution: &ContactSolution,
        tolerance: f64,
    ) {
        let flat: Vec<f64> = solution.impulses_n_s.iter().flatten().copied().collect();
        let velocity: Vec<f64> = delassus
            .mul_vec(&flat)
            .iter()
            .zip(free)
            .map(|(a, b)| a + b)
            .collect();
        for (contact, impulse) in solution.impulses_n_s.iter().enumerate() {
            let v = &velocity[3 * contact..3 * contact + 3];
            let tangential = impulse[0].hypot(impulse[1]);
            assert!(impulse[2] >= -tolerance, "negative normal impulse");
            assert!(v[2] >= -tolerance, "penetrating normal velocity {}", v[2]);
            assert!(
                (impulse[2] * v[2]).abs() <= tolerance,
                "normal complementarity {} * {}",
                impulse[2],
                v[2]
            );
            assert!(
                tangential <= mu[contact] * impulse[2] + tolerance,
                "outside cone: {tangential} > {}",
                mu[contact] * impulse[2]
            );
            match solution.states[contact] {
                ContactState::Open => assert!(impulse.iter().all(|c| c.abs() <= tolerance)),
                ContactState::Sticking => {
                    assert!(
                        v.iter().all(|c| c.abs() <= tolerance),
                        "sticking moves {v:?}"
                    )
                }
                ContactState::Sliding => {
                    let slip = v[0].hypot(v[1]);
                    assert!((tangential - mu[contact] * impulse[2]).abs() <= tolerance);
                    if slip > tolerance && tangential > tolerance {
                        // Maximum dissipation: slip opposes the friction impulse.
                        let cosine = (impulse[0] * v[0] + impulse[1] * v[1]) / (tangential * slip);
                        assert!(cosine <= -1.0 + 1.0e-6, "slip not opposed, cos {cosine}");
                    }
                }
            }
        }
    }

    #[test]
    fn single_contact_cases_match_closed_form() {
        let mut identity = DenseMatrix::zeros(3, 3);
        for axis in 0..3 {
            identity.set(axis, axis, 1.0);
        }
        let config = ContactSolverConfig::default();

        let open = solve_contact_impulses(&identity, &[0.3, 0.0, 0.5], &[0.8], &config, None)
            .expect("open");
        assert_eq!(open.states, vec![ContactState::Open]);
        assert_eq!(open.impulses_n_s, vec![[0.0; 3]]);

        let stick = solve_contact_impulses(&identity, &[0.2, 0.0, -1.0], &[0.8], &config, None)
            .expect("stick");
        assert_eq!(stick.states, vec![ContactState::Sticking]);
        assert_eq!(stick.impulses_n_s, vec![[-0.2, 0.0, 1.0]]);

        // Sticking would need |λt| = 2 > μ λn = 0.5, so the contact slides with
        // λn = 1 and the full friction impulse opposing the slip.
        let slide = solve_contact_impulses(&identity, &[2.0, 0.0, -1.0], &[0.5], &config, None)
            .expect("slide");
        assert_eq!(slide.states, vec![ContactState::Sliding]);
        let impulse = slide.impulses_n_s[0];
        assert!((impulse[0] + 0.5).abs() < 1.0e-12, "{impulse:?}");
        assert!(impulse[1].abs() < 1.0e-12, "{impulse:?}");
        assert!((impulse[2] - 1.0).abs() < 1.0e-12, "{impulse:?}");
    }

    #[test]
    fn coupled_single_contact_slides_on_the_exact_cone() {
        let mut rng = Lcg(7);
        for _ in 0..200 {
            let delassus = random_spd(3, &mut rng);
            let free = [3.0 * rng.next(), 3.0 * rng.next(), -rng.next().abs() - 0.1];
            let mu = [0.2 + 0.8 * rng.next().abs()];
            let solution = solve_contact_impulses(
                &delassus,
                &free,
                &mu,
                &ContactSolverConfig::default(),
                None,
            )
            .expect("solve");
            assert!(solution.converged);
            assert_contact_conditions(&delassus, &free, &mu, &solution, 1.0e-9);
        }
    }

    #[test]
    fn many_contacts_converge_to_complementary_impulses() {
        let mut rng = Lcg(11);
        let count = 6;
        let config = ContactSolverConfig {
            max_iterations: 5000,
            tolerance_n_s: 1.0e-13,
            ..ContactSolverConfig::default()
        };
        for _ in 0..20 {
            let delassus = random_spd(3 * count, &mut rng);
            let free: Vec<f64> = (0..3 * count).map(|_| 2.0 * rng.next()).collect();
            let mu: Vec<f64> = (0..count).map(|_| 0.1 + rng.next().abs()).collect();
            let solution =
                solve_contact_impulses(&delassus, &free, &mu, &config, None).expect("solve");
            assert!(solution.converged, "change {}", solution.last_change_n_s);
            assert_contact_conditions(&delassus, &free, &mu, &solution, 1.0e-7);

            // Warm starting from the solution reproduces it in one sweep.
            let warm = solve_contact_impulses(
                &delassus,
                &free,
                &mu,
                &config,
                Some(&solution.impulses_n_s),
            )
            .expect("warm");
            assert!(warm.iterations <= 2, "warm start took {}", warm.iterations);
        }
    }

    #[test]
    fn solver_rejects_invalid_input() {
        let delassus = DenseMatrix::zeros(3, 3);
        let config = ContactSolverConfig::default();
        assert!(matches!(
            solve_contact_impulses(&delassus, &[0.0; 3], &[-0.1], &config, None),
            Err(DynamicsError::InvalidContact(_))
        ));
        assert!(matches!(
            solve_contact_impulses(&delassus, &[0.0; 2], &[0.5], &config, None),
            Err(DynamicsError::DimensionMismatch { .. })
        ));
        assert!(matches!(
            solve_contact_impulses(&delassus, &[f64::NAN, 0.0, 0.0], &[0.5], &config, None),
            Err(DynamicsError::NonFiniteInput)
        ));
    }

    fn box_model(gravity_m_s2: Vec3) -> (ArticulatedModel, Entity) {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "robot");
        let base = spawn_named(&mut world, "box");
        world.entity_mut(robot).insert(Robot {
            robot_id: RobotId::new_v4(),
            model_name: "box".into(),
            base_link: base,
        });
        let [hx, hy, hz] = HALF_EXTENTS_M;
        let third = BOX_MASS_KG / 3.0;
        world.entity_mut(base).insert((
            Link {
                robot,
                name: "box".into(),
            },
            Transform3::IDENTITY,
            FloatingBase,
            RigidBody {
                mass_kg: BOX_MASS_KG,
                ..RigidBody::default()
            },
            RigidBodyInertia {
                center_of_mass_local_m: Vec3::ZERO,
                ixx_kg_m2: third * (hy * hy + hz * hz),
                ixy_kg_m2: 0.0,
                ixz_kg_m2: 0.0,
                iyy_kg_m2: third * (hx * hx + hz * hz),
                iyz_kg_m2: 0.0,
                izz_kg_m2: third * (hx * hx + hy * hy),
            },
        ));
        let model =
            ArticulatedModel::from_robot_with_gravity(&world, robot, gravity_m_s2).expect("model");
        (model, base)
    }

    /// Bottom corners of the box against the ground plane `y = 0`.
    fn box_contacts(
        model: &ArticulatedModel,
        link: Entity,
        q: &[f64],
        mu: f64,
    ) -> Vec<ContactPoint> {
        let base = model
            .kinematic()
            .forward_kinematics(q)
            .expect("fk")
            .transforms()[0];
        let [hx, hy, hz] = HALF_EXTENTS_M;
        [(-hx, -hz), (hx, -hz), (-hx, hz), (hx, hz)]
            .into_iter()
            .map(|(x, z)| {
                let point_local_m = Vec3::new(x, -hy, z);
                let world = base.translation + base.rotation * point_local_m;
                ContactPoint {
                    link,
                    point_local_m,
                    normal_world: Vec3::Y,
                    gap_m: world.y,
                    friction_coefficient: mu,
                }
            })
            .collect()
    }

    fn run_box_on_incline(incline_rad: f64, mu: f64, steps: usize) -> (Vec<f64>, Vec<f64>) {
        // Tilting gravity is equivalent to tilting the ground by the same angle.
        let gravity = Vec3::new(9.81 * incline_rad.sin(), -9.81 * incline_rad.cos(), 0.0);
        let (model, link) = box_model(gravity);
        let mut q = vec![0.0; 6];
        q[1] = HALF_EXTENTS_M[1];
        let mut qd = vec![0.0; 6];
        let config = ContactStepConfig::default();
        let mut warm: Option<Vec<[f64; 3]>> = None;
        for _ in 0..steps {
            let contacts = box_contacts(&model, link, &q, mu);
            let step = contact_step(
                &model,
                &q,
                &qd,
                &[0.0; 6],
                &contacts,
                None,
                &config,
                warm.as_deref(),
            )
            .expect("step");
            warm = Some(
                step.contacts
                    .iter()
                    .zip(&contacts)
                    .map(|(outcome, contact)| {
                        let frame = contact_frame(contact.normal_world);
                        [
                            outcome.impulse_world_n_s.dot(frame[0]),
                            outcome.impulse_world_n_s.dot(frame[1]),
                            outcome.impulse_world_n_s.dot(frame[2]),
                        ]
                    })
                    .collect(),
            );
            q = step.q;
            qd = step.qd;
        }
        (q, qd)
    }

    #[test]
    fn box_sticks_when_friction_exceeds_the_incline() {
        let incline = 20.0_f64.to_radians();
        // tan(20°) ≈ 0.364 < μ = 0.5: static friction holds the box.
        let (q, qd) = run_box_on_incline(incline, 0.5, 500);
        assert!(q[0].abs() < 1.0e-6, "box crept {} m", q[0]);
        assert!((q[1] - HALF_EXTENTS_M[1]).abs() < 1.0e-6, "height {}", q[1]);
        assert!(qd.iter().all(|v| v.abs() < 1.0e-6), "velocity {qd:?}");
    }

    #[test]
    fn box_slides_with_the_coulomb_acceleration() {
        let incline = 20.0_f64.to_radians();
        let mu = 0.2;
        let steps = 250;
        let (q, qd) = run_box_on_incline(incline, mu, steps);
        let duration = steps as f64 * ContactStepConfig::default().step_time_s;
        let expected = 9.81 * (incline.sin() - mu * incline.cos()) * duration;
        assert!(
            (qd[0] - expected).abs() < 1.0e-6,
            "slide velocity {} vs {expected}",
            qd[0]
        );
        assert!((q[1] - HALF_EXTENTS_M[1]).abs() < 1.0e-6, "height {}", q[1]);
        assert!(qd[3..].iter().all(|w| w.abs() < 1.0e-6), "spin {qd:?}");
    }

    #[test]
    fn dropped_box_lands_without_bounce_or_penetration() {
        let (model, link) = box_model(Vec3::new(0.0, -9.81, 0.0));
        let mut q = vec![0.0; 6];
        q[1] = HALF_EXTENTS_M[1] + 0.2;
        // A small tilt so the corners land one after another.
        q[3] = 0.05;
        let mut qd = vec![0.0; 6];
        let config = ContactStepConfig::default();
        let mut deepest = 0.0_f64;
        for _ in 0..1500 {
            let contacts = box_contacts(&model, link, &q, 0.8);
            for contact in &contacts {
                deepest = deepest.min(contact.gap_m);
            }
            let step = contact_step(&model, &q, &qd, &[0.0; 6], &contacts, None, &config, None)
                .expect("step");
            q = step.q;
            qd = step.qd;
        }
        assert!(deepest > -1.0e-3, "penetrated {deepest} m");
        assert!(
            (q[1] - HALF_EXTENTS_M[1]).abs() < 1.0e-4,
            "rest height {}",
            q[1]
        );
        assert!(
            q[3].abs() < 1.0e-4 && q[5].abs() < 1.0e-4,
            "not flat: {q:?}"
        );
        assert!(qd.iter().all(|v| v.abs() < 1.0e-4), "still moving {qd:?}");
    }

    fn pendulum_model() -> ArticulatedModel {
        limited_pendulum_model(JointLimits::default())
    }

    fn limited_pendulum_model(limits: JointLimits) -> ArticulatedModel {
        let mut world = World::new();
        let robot = spawn_named(&mut world, "robot");
        let base = spawn_named(&mut world, "base");
        let link = spawn_named(&mut world, "link");
        let joint = spawn_named(&mut world, "joint");
        world.entity_mut(robot).insert(Robot {
            robot_id: RobotId::new_v4(),
            model_name: "pendulum".into(),
            base_link: base,
        });
        world.entity_mut(base).insert((
            Link {
                robot,
                name: "base".into(),
            },
            Transform3::IDENTITY,
        ));
        world.entity_mut(link).insert((
            Link {
                robot,
                name: "link".into(),
            },
            Transform3::IDENTITY,
            RigidBody {
                mass_kg: 1.0,
                ..RigidBody::default()
            },
            RigidBodyInertia {
                center_of_mass_local_m: Vec3::new(1.0, 0.0, 0.0),
                ixx_kg_m2: 0.0,
                ixy_kg_m2: 0.0,
                ixz_kg_m2: 0.0,
                iyy_kg_m2: 0.0,
                iyz_kg_m2: 0.0,
                izz_kg_m2: 0.0,
            },
        ));
        world.entity_mut(joint).insert(Joint {
            robot,
            parent_link: base,
            child_link: link,
            kind: JointKind::Revolute,
            limits,
            axis: Vec3::Z,
            position: 0.0,
            velocity: 0.0,
        });
        ArticulatedModel::from_robot(&world, robot).expect("model")
    }

    #[test]
    fn implicit_pd_is_stable_far_beyond_the_explicit_limit() {
        let model = pendulum_model();
        // ω = sqrt(kp / m l²) ≈ 3162 rad/s, so ω dt ≈ 32: an explicit PD step
        // diverges immediately at this gain and step.
        let kp = 1.0e7;
        let target = 0.7;
        let pd = JointPdControl {
            position_gains: vec![kp],
            velocity_gains: vec![2.0 * kp.sqrt()],
            target_positions: vec![target],
            target_velocities: vec![0.0],
            effort_limits: Vec::new(),
        };
        let config = ContactStepConfig {
            step_time_s: 0.01,
            ..ContactStepConfig::default()
        };
        let mut q = vec![-1.2];
        let mut qd = vec![0.0];
        for _ in 0..100 {
            let step =
                contact_step(&model, &q, &qd, &[0.0], &[], Some(&pd), &config, None).expect("step");
            q = step.q;
            qd = step.qd;
            assert!(q[0].is_finite() && q[0].abs() < 2.0, "diverged: {q:?}");
        }
        // The residual is the gravity sag m g l cos(q) / kp.
        let sag = 9.81 * target.cos() / kp;
        assert!(
            (q[0] - (target - sag)).abs() < 1.0e-6,
            "settled at {}",
            q[0]
        );
        assert!(qd[0].abs() < 1.0e-6, "rate {}", qd[0]);
    }

    #[test]
    fn floating_base_integration_follows_the_body_twist() {
        let (model, _) = box_model(Vec3::ZERO);
        let mut q = vec![0.0, 0.0, 0.0, 0.0, 0.0, std::f64::consts::FRAC_PI_2];
        // Body-frame forward velocity along +x moves along world +y after a
        // quarter turn about z.
        let qd = vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        q = integrate_configuration(&model, &q, &qd, 0.5).expect("integrate");
        assert!(
            q[0].abs() < 1.0e-12 && (q[1] - 0.5).abs() < 1.0e-12,
            "{q:?}"
        );

        // Spinning about body z accumulates yaw continuously past ±π.
        let spin = vec![0.0, 0.0, 0.0, 0.0, 0.0, 1.0];
        let mut q = vec![0.0; 6];
        for _ in 0..400 {
            q = integrate_configuration(&model, &q, &spin, 0.01).expect("integrate");
        }
        assert!((q[5] - 4.0).abs() < 1.0e-9, "yaw {}", q[5]);
        assert!(q[3].abs() < 1.0e-12 && q[4].abs() < 1.0e-12, "{q:?}");
    }

    #[test]
    fn contact_frame_is_orthonormal_and_right_handed() {
        for normal in [Vec3::Y, Vec3::X, -Vec3::Z, Vec3::new(0.3, 0.9, -0.2)] {
            let [t1, t2, n] = contact_frame(normal);
            assert!((n - normal.normalize()).length() < 1.0e-12);
            assert!(t1.dot(n).abs() < 1.0e-12 && t2.dot(n).abs() < 1.0e-12);
            assert!(t1.dot(t2).abs() < 1.0e-12);
            assert!((t1.cross(t2) - n).length() < 1.0e-12);
        }
    }

    fn limits(lower: f64, upper: f64) -> JointLimits {
        JointLimits {
            lower,
            upper,
            ..JointLimits::default()
        }
    }

    fn swing(
        model: &ArticulatedModel,
        start: f64,
        pd: Option<&JointPdControl>,
        config: &ContactStepConfig,
        steps: usize,
    ) -> (f64, f64, f64, Vec<JointLimitOutcome>) {
        let mut q = vec![start];
        let mut qd = vec![0.0];
        let mut lowest = start;
        let mut last = Vec::new();
        for _ in 0..steps {
            let step = contact_step(model, &q, &qd, &[0.0], &[], pd, config, None).expect("step");
            q = step.q;
            qd = step.qd;
            lowest = lowest.min(q[0]);
            last = step.joint_limits;
        }
        (q[0], qd[0], lowest, last)
    }

    #[test]
    fn falling_pendulum_comes_to_rest_on_its_lower_limit() {
        // Gravity pulls the pendulum toward -π/2; the limit stops it at -0.5.
        let model = limited_pendulum_model(limits(-0.5, 1.0));
        assert_eq!(model.joint_position_limits(0), Some((-0.5, 1.0)));
        let config = ContactStepConfig::default();
        let (angle, rate, lowest, engaged) = swing(&model, 0.8, None, &config, 1500);
        assert!(lowest >= -0.5 - 1.0e-9, "passed the limit: {lowest}");
        assert!((angle + 0.5).abs() < 1.0e-9, "rests at {angle}");
        assert!(rate.abs() < 1.0e-9, "still moving at {rate}");
        // The limit carries the gravity torque m g l cos(q).
        assert_eq!(engaged.len(), 1);
        assert_eq!(engaged[0].side, JointLimitSide::Lower);
        let torque = engaged[0].impulse / config.step_time_s;
        assert!(
            (torque - 9.81 * 0.5_f64.cos()).abs() < 1.0e-6,
            "limit torque {torque}"
        );
    }

    #[test]
    fn stiff_pd_past_a_limit_cannot_drive_the_joint_through_it() {
        // The target lies 0.5 rad beyond the upper limit and the gain is far
        // past the explicit stability bound; the joint still stops at 0.4.
        let model = limited_pendulum_model(limits(-0.4, 0.4));
        let kp = 1.0e6;
        let pd = JointPdControl {
            position_gains: vec![kp],
            velocity_gains: vec![2.0 * kp.sqrt()],
            target_positions: vec![0.9],
            target_velocities: vec![0.0],
            effort_limits: Vec::new(),
        };
        let config = ContactStepConfig::default();
        let (angle, _, _, engaged) = swing(&model, 0.0, Some(&pd), &config, 500);
        assert!((angle - 0.4).abs() < 1.0e-9, "settled at {angle}");
        assert_eq!(engaged.len(), 1);
        assert_eq!(engaged[0].side, JointLimitSide::Upper);
        // The limit balances the PD pull plus gravity at the limit.
        let torque = engaged[0].impulse / config.step_time_s;
        let expected = kp * (0.9 - 0.4) - 9.81 * 0.4_f64.cos();
        assert!(
            (torque - expected).abs() < 1.0e-6 * expected,
            "limit torque {torque}"
        );

        // Without limit enforcement the same command drives it to the target.
        let free = ContactStepConfig {
            enforce_joint_limits: false,
            ..ContactStepConfig::default()
        };
        let (angle, _, _, engaged) = swing(&model, 0.0, Some(&pd), &free, 500);
        assert!((angle - 0.9).abs() < 1.0e-4, "free joint at {angle}");
        assert!(engaged.is_empty());
    }

    #[test]
    fn a_joint_started_past_its_limit_is_recovered() {
        let model = limited_pendulum_model(limits(0.0, 0.5));
        let config = ContactStepConfig::default();
        // Start 0.1 rad below the lower limit with gravity pulling further down.
        let (angle, _, _, _) = swing(&model, -0.1, None, &config, 200);
        assert!(angle.abs() < 1.0e-6, "not recovered: {angle}");
    }

    #[test]
    fn continuous_and_unbounded_joints_have_no_limits() {
        let model = pendulum_model();
        assert_eq!(model.joint_position_limits(0), None);
        let (box_model, _) = box_model(Vec3::ZERO);
        assert!((0..6).all(|dof| box_model.joint_position_limits(dof).is_none()));
    }

    #[test]
    fn tree_factor_matches_the_dense_factor() {
        use crate::test_models::{branching_tree, Lcg as TreeLcg};
        for floating in [false, true] {
            let model = branching_tree(floating, 17);
            let parents = model
                .dof_parents
                .clone()
                .expect("parents-first coordinates");
            let mut rng = TreeLcg(41);
            let nv = model.nv();
            for _ in 0..10 {
                let mut q: Vec<f64> = (0..nv).map(|_| rng.next()).collect();
                if floating {
                    q[4] *= 0.5;
                }
                let mut matrix = mass_matrix_from_q(&model, &q);
                // Implicit PD only adds to the diagonal, which keeps the sparsity.
                for dof in model.base_dof()..nv {
                    matrix.set(dof, dof, matrix.get(dof, dof) + 0.3 * rng.next().abs());
                }
                let rhs: Vec<f64> = (0..nv).map(|_| rng.next()).collect();
                let tree = Cholesky::new(&matrix, Some(&parents)).expect("tree factor");
                assert!(tree.parents.is_some());
                let dense = Cholesky::new(&matrix, None).expect("dense factor");
                let fast = tree.solve(&rhs);
                let reference = dense.solve(&rhs);
                for (a, b) in fast.iter().zip(&reference) {
                    assert!((a - b).abs() < 1.0e-9 * (1.0 + b.abs()), "{a} vs {b}");
                }
                // And it really solves the system.
                let back = matrix.mul_vec(&fast);
                for (a, b) in back.iter().zip(&rhs) {
                    assert!((a - b).abs() < 1.0e-9, "{a} vs {b}");
                }
            }
        }
    }

    fn mass_matrix_from_q(model: &ArticulatedModel, q: &[f64]) -> DenseMatrix {
        crate::algorithms::mass_matrix(model, q).expect("mass matrix")
    }

    fn effort_limited_pendulum(max_effort: f64) -> ArticulatedModel {
        limited_pendulum_model(JointLimits {
            max_effort,
            ..JointLimits::default()
        })
    }

    #[test]
    fn effort_limit_caps_a_stiff_pd_that_cannot_hold_the_load() {
        // Holding the 1 kg, 1 m pendulum horizontal needs 9.81 N·m; a 5 N·m
        // actuator saturates and the arm falls toward hanging.
        let model = effort_limited_pendulum(5.0);
        assert_eq!(model.joint_effort_limit(0), Some(5.0));
        let pd = JointPdControl {
            position_gains: vec![400.0],
            velocity_gains: vec![40.0],
            target_positions: vec![0.0],
            target_velocities: vec![0.0],
            effort_limits: Vec::new(),
        };
        let config = ContactStepConfig::default();
        let mut q = vec![0.0];
        let mut qd = vec![0.0];
        let mut saturated_steps = 0;
        let mut lowest = 0.0_f64;
        for _ in 0..1000 {
            let step =
                contact_step(&model, &q, &qd, &[0.0], &[], Some(&pd), &config, None).expect("step");
            assert!(step.actuator_torques[0].abs() <= 5.0 + 1.0e-9);
            if !step.saturated_joints.is_empty() {
                saturated_steps += 1;
            }
            q = step.q;
            qd = step.qd;
            lowest = lowest.min(q[0]);
        }
        assert!(saturated_steps > 0);
        // A saturated actuator is a constant torque with no PD damping, so the
        // arm swings undamped about the angle where 5 N·m balances gravity
        // (cos q = 5 / 9.81) and passes well below it.
        let balance = -(5.0_f64 / 9.81).acos();
        assert!(
            lowest < balance - 0.5,
            "lowest {lowest} vs balance {balance}"
        );

        // A 20 N·m actuator holds the arm level with the gravity torque.
        let strong = effort_limited_pendulum(20.0);
        let mut q = vec![0.0];
        let mut qd = vec![0.0];
        let mut last = None;
        for _ in 0..1000 {
            let step = contact_step(&strong, &q, &qd, &[0.0], &[], Some(&pd), &config, None)
                .expect("step");
            q.clone_from(&step.q);
            qd.clone_from(&step.qd);
            last = Some(step);
        }
        let last = last.expect("steps");
        assert!(last.saturated_joints.is_empty());
        assert!((last.actuator_torques[0] - 9.81 * q[0].cos()).abs() < 1.0e-6);
        assert!(q[0].abs() < 0.03, "sag {}", q[0]);
    }

    #[test]
    fn a_command_effort_limit_tightens_the_model_limit() {
        let model = effort_limited_pendulum(5.0);
        let config = ContactStepConfig::default();
        let command = |limit: f64| JointPdControl {
            position_gains: vec![0.0],
            velocity_gains: vec![0.0],
            target_positions: vec![0.0],
            target_velocities: vec![0.0],
            effort_limits: vec![limit],
        };
        for (limit, applied) in [(2.0, 2.0), (0.0, 0.0), (8.0, 5.0), (f64::INFINITY, 5.0)] {
            let step = contact_step(
                &model,
                &[0.0],
                &[0.0],
                &[100.0],
                &[],
                Some(&command(limit)),
                &config,
                None,
            )
            .expect("step");
            assert_eq!(step.actuator_torques, vec![applied], "limit {limit}");
        }
        assert!(matches!(
            contact_step(
                &model,
                &[0.0],
                &[0.0],
                &[0.0],
                &[],
                Some(&command(-1.0)),
                &config,
                None
            ),
            Err(DynamicsError::InvalidContact(_))
        ));
    }

    #[test]
    fn feed_forward_torque_beyond_the_limit_is_clamped() {
        let model = effort_limited_pendulum(5.0);
        let config = ContactStepConfig::default();
        let step =
            contact_step(&model, &[0.0], &[0.0], &[100.0], &[], None, &config, None).expect("step");
        assert_eq!(step.saturated_joints, vec![0]);
        assert_eq!(step.actuator_torques, vec![5.0]);
        // m l² qdd = 5 - m g l at the horizontal.
        let acceleration = step.qd[0] / config.step_time_s;
        assert!(
            (acceleration - (5.0 - 9.81)).abs() < 1.0e-9,
            "{acceleration}"
        );

        let unlimited = ContactStepConfig {
            enforce_effort_limits: false,
            ..ContactStepConfig::default()
        };
        let step = contact_step(
            &model,
            &[0.0],
            &[0.0],
            &[100.0],
            &[],
            None,
            &unlimited,
            None,
        )
        .expect("step");
        assert!(step.saturated_joints.is_empty());
        assert!((step.qd[0] / config.step_time_s - (100.0 - 9.81)).abs() < 1.0e-9);
    }
}
