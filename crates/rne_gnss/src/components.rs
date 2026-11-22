//! ECS components for GNSS simulation.

use crate::receiver::{GnssFix, GnssReceiver};
use bevy_ecs::prelude::Component;
use rne_math::Vec3;

/// A GNSS antenna and its receiver, attached to a robot entity.
///
/// The owner writes `antenna_enu_m` (the antenna's true position in the
/// environment's ENU frame) before stepping; the system writes `latest_fix`.
#[derive(Component, Clone, Debug)]
pub struct GnssAntenna {
    /// The simulated receiver.
    pub receiver: GnssReceiver,
    /// True antenna position in ENU meters.
    pub antenna_enu_m: Vec3,
    /// The most recently released fix.
    pub latest_fix: Option<GnssFix>,
}

impl GnssAntenna {
    /// Creates an antenna at a position.
    pub fn new(receiver: GnssReceiver, antenna_enu_m: Vec3) -> Self {
        Self {
            receiver,
            antenna_enu_m,
            latest_fix: None,
        }
    }
}
