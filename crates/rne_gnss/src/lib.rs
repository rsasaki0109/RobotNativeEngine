//! Ray-traced GNSS simulation for Robot Native Engine.
//!
//! `rne_gnss` simulates what a GNSS receiver measures in a built environment:
//!
//! * [`geodesy`] — exact WGS-84 geodetic / ECEF / local ENU conversions.
//! * [`orbit`] — Keplerian orbits and nominal GPS and Galileo constellations.
//! * [`scene`] — buildings and walls as material-tagged boxes, plus ground.
//! * [`time`], [`ephemeris`] — GPS time and broadcast ephemerides that
//!   reproduce the simulated orbits exactly, for external solvers.
//! * [`atmosphere`] — Klobuchar ionosphere and Saastamoinen troposphere.
//! * [`raytrace`] — line-of-sight occlusion, multi-bounce specular
//!   reflections (image method), and knife-edge diffraction over box edges,
//!   giving LOS, LOS with multipath, NLOS, diffracted, and blocked receptions.
//! * [`signal`] — GPS L1/L2C and Galileo E1/E5a, antenna gain, DLL and PLL
//!   thermal noise from C/N0, and code and carrier multipath.
//! * [`receiver`] — a seeded receiver that turns the ray trace into
//!   pseudoranges, carrier phases (integer ambiguities, cycle slips), Doppler
//!   and fixes at a set rate and latency.
//! * [`antenna`] — multi-antenna rigs with body-frame lever arms.
//! * [`solver`] — weighted least squares single-point positioning with DOP
//!   and RAIM fault detection and exclusion.
//!
//! Everything is deterministic: time is simulation time, iteration is in
//! satellite-id and box-index order, and noise comes from seeded
//! [`rne_core::DeterministicRng`] streams. The crate does not depend on a
//! renderer, physics backend, ROS 2, or an external GNSS library.

#![deny(missing_docs)]

pub mod antenna;
pub mod atmosphere;
pub mod components;
pub mod ephemeris;
pub mod geodesy;
pub mod mesh;
pub mod orbit;
pub mod raytrace;
pub mod receiver;
pub mod resources;
pub mod scene;
pub mod signal;
pub mod solver;
pub mod systems;
pub mod time;

pub use antenna::{AntennaRig, BodyPose, RigAntenna};
pub use atmosphere::{saastamoinen_delay_m, AtmosphereConfig, KlobucharParams, GPS_L1_HZ};
pub use components::GnssAntenna;
pub use ephemeris::{BroadcastEphemeris, NavigationMessage};
pub use geodesy::{azimuth_elevation, EnuFrame, Geodetic};
pub use mesh::{box_triangles, DiffractionEdge, MeshTriangle, TriangleMesh};
pub use orbit::{
    Constellation, GnssSystem, KeplerOrbit, Satellite, SatelliteId, EARTH_ROTATION_RAD_S,
    SPEED_OF_LIGHT_M_S,
};
pub use raytrace::{
    knife_edge_loss_db, trace_satellite, PathKind, Propagation, PropagationPath, TraceConfig,
};
pub use receiver::{
    ClockConfig, FixStatus, GnssEpoch, GnssFix, GnssReceiver, ReceiverConfig, ReceiverError,
    SatelliteObservation, SignalObservation, SignalPath,
};
pub use resources::GnssEnvironment;
pub use scene::{
    FacetSource, FoliageVolume, GnssScene, GroundPlane, RayHit, SceneBox, SurfaceMaterial,
};
pub use signal::{SignalConfig, SignalKind};
pub use solver::{
    chi_square_quantile, solve_position, Dop, PositionSolution, PseudorangeInput, SolveError,
    SolverConfig,
};
pub use systems::step_gnss_antennas;
pub use time::{GnssTime, SECONDS_PER_WEEK};
