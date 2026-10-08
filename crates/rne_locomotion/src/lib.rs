//! Native legged locomotion on generated terrain for Robot Native Engine.
//!
//! This crate assembles the RaiSim-style pieces of `rne_dynamics` and
//! `rne_physics` into a legged-locomotion stack that runs without a physics
//! backend:
//!
//! - [`Quadruped`] builds a floating-base quadruped from a URDF with the
//!   Unitree leg naming convention, measures its leg geometry, and solves leg
//!   inverse kinematics.
//! - [`QuadrupedTerrainSim`] steps it over a [`rne_physics::FractalTerrain`]
//!   height field with [`rne_dynamics::contact_step`]: sphere feet, the exact
//!   friction cone, implicit joint PD, and actuator effort limits.
//! - [`TrotController`] is a feedback trot: Raibert foot placement for speed
//!   and push recovery, a heading loop for yaw, and a level-frame foot plan
//!   that keeps the body upright on uneven ground.
//! - [`QuadrupedTerrainEpisode`] wraps the simulation as an
//!   [`rne_ai::Episode`] with a new terrain and command each episode, so
//!   [`rne_ai::VectorizedEpisode`] can run many of them in parallel, the
//!   role raisimGym plays for RaiSim.
//!
//! # Frames
//!
//! RNE's world is Y-up. The floating base of `rne_dynamics` is parameterized
//! by fixed-axis roll-pitch-yaw `Rz Ry Rx`, whose middle angle is confined to
//! ±90°. In a Y-up world that middle angle is the heading, so a robot turning
//! past a quarter turn flips its roll and yaw coordinates by π: the pose stays
//! right, but the coordinates a controller or an observation reads jump. This
//! crate therefore simulates in a Z-up *locomotion frame* — RNE's world
//! rotated by +90° about X — where the heading is the yaw coordinate and roll
//! and pitch stay small and continuous.
//! [`world_from_locomotion`] and [`locomotion_from_world`] convert points and
//! directions between the two frames, and the terrain stays an ordinary Y-up
//! [`rne_physics::ColliderShape::HeightField`].

#![deny(missing_docs)]

mod episode;
mod quadruped;
mod sim;
mod trot;

pub use episode::{
    quadruped_terrain_task_spec, ActionReference, QuadrupedJointAction, QuadrupedObservation,
    QuadrupedTerrainConfig, QuadrupedTerrainEpisode, QUADRUPED_ACTION_DIM,
    QUADRUPED_OBSERVATION_DIM,
};
pub use quadruped::{
    locomotion_from_world, world_from_locomotion, BaseState, LegGeometry, LocomotionError,
    Quadruped, QuadrupedSpec,
};
pub use sim::{JointCommand, QuadrupedTerrainSim, SimStep, TerrainSimConfig};
pub use trot::{TrotController, TrotParams, VelocityCommand};
