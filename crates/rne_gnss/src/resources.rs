//! ECS resources for GNSS simulation.

use crate::geodesy::EnuFrame;
use crate::orbit::Constellation;
use crate::scene::GnssScene;
use bevy_ecs::prelude::Resource;

/// The shared GNSS environment: propagation scene, satellites, and the local
/// ENU frame the scene is expressed in.
#[derive(Resource, Clone, Debug)]
pub struct GnssEnvironment {
    /// Ray-traced scene geometry.
    pub scene: GnssScene,
    /// Satellites.
    pub constellation: Constellation,
    /// Local ENU frame of the scene.
    pub frame: EnuFrame,
}
