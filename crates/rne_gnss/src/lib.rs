//! Ray-traced GNSS simulation for Robot Native Engine.
//!
//! `rne_gnss` simulates what a GNSS receiver measures in a built environment:
//!
//! * [`geodesy`] — exact WGS-84 geodetic / ECEF / local ENU conversions.
//! * [`orbit`] — Keplerian orbits and nominal GPS and Galileo constellations.
//! * [`scene`] — buildings and walls as material-tagged boxes, plus ground.
//! * [`raytrace`] — line-of-sight occlusion and single-bounce specular
//!   reflections (image method) for every satellite, giving LOS, LOS with
//!   multipath, NLOS, and blocked receptions.
//! * [`signal`] — antenna gain, DLL thermal noise from C/N0, the
//!   narrow-correlator code-multipath envelope, and residual atmosphere.
//! * [`receiver`] — a seeded receiver that turns the ray trace into
//!   pseudoranges and fixes at a set rate and latency.
//! * [`solver`] — weighted least squares single-point positioning with DOP
//!   and RAIM fault detection and exclusion.
//!
//! Everything is deterministic: time is simulation time, iteration is in
//! satellite-id and box-index order, and noise comes from seeded
//! [`rne_core::DeterministicRng`] streams. The crate does not depend on a
//! renderer, physics backend, ROS 2, or an external GNSS library.

#![deny(missing_docs)]

pub mod components;
pub mod geodesy;
pub mod orbit;
pub mod raytrace;
pub mod receiver;
pub mod resources;
pub mod scene;
pub mod signal;
pub mod solver;
pub mod systems;

pub use components::GnssAntenna;
pub use geodesy::{azimuth_elevation, EnuFrame, Geodetic};
pub use orbit::{
    Constellation, GnssSystem, KeplerOrbit, Satellite, SatelliteId, EARTH_ROTATION_RAD_S,
    SPEED_OF_LIGHT_M_S,
};
pub use raytrace::{trace_satellite, Propagation, ReflectedPath};
pub use receiver::{
    ClockConfig, FixStatus, GnssEpoch, GnssFix, GnssReceiver, ReceiverConfig, ReceiverError,
    SatelliteObservation, SignalPath,
};
pub use resources::GnssEnvironment;
pub use scene::{FacetSource, GnssScene, GroundPlane, RayHit, SceneBox, SurfaceMaterial};
pub use signal::{AtmosphereConfig, SignalConfig};
pub use solver::{
    chi_square_quantile, solve_position, Dop, PositionSolution, PseudorangeInput, SolveError,
    SolverConfig,
};
pub use systems::step_gnss_antennas;
