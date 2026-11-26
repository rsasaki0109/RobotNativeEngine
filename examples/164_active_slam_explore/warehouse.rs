//! The warehouse of example 127 on its first day: fixed walls, two racks, a
//! column, the office block and six pallets.
//!
//! World coordinates are the physics world's `(x, z)`; the SLAM plane maps
//! `x -> x` and `z -> y`, so a rectangle here is a rectangle on the map.

use rne_ecs::{spawn_named, World};
use rne_math::{Quat, Vec3};
use rne_physics::{Collider, ColliderShape, RigidBody, RigidBodyType};
use rne_world::Transform3;

/// An axis-aligned footprint: centre `(x, z)` and half extents, in meters.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Footprint {
    pub(crate) x: f64,
    pub(crate) z: f64,
    pub(crate) half_x: f64,
    pub(crate) half_z: f64,
    pub(crate) height: f64,
}

impl Footprint {
    const fn new(x: f64, z: f64, half_x: f64, half_z: f64, height: f64) -> Self {
        Self {
            x,
            z,
            half_x,
            half_z,
            height,
        }
    }

    /// Whether a disc of radius `margin` at `(x, z)` overlaps the box.
    pub(crate) fn contains(&self, x: f64, z: f64, margin: f64) -> bool {
        let dx = ((x - self.x).abs() - self.half_x).max(0.0);
        let dz = ((z - self.z).abs() - self.half_z).max(0.0);
        dx.hypot(dz) <= margin
    }
}

const PALLET_HALF_M: f64 = 0.45;
const PALLET_HEIGHT_M: f64 = 1.2;

/// Everything in the warehouse: walls, racks, the column, the office block,
/// and the first day's pallets of example 127.
pub(crate) const FOOTPRINTS: [Footprint; 14] = [
    Footprint::new(0.0, 4.6, 7.2, 0.1, 2.4),    // north wall
    Footprint::new(0.0, -4.6, 7.2, 0.1, 2.4),   // south wall
    Footprint::new(7.1, 0.0, 0.1, 4.6, 2.4),    // east wall
    Footprint::new(-7.1, 0.0, 0.1, 4.6, 2.4),   // west wall
    Footprint::new(-3.0, -3.7, 2.2, 0.45, 2.0), // long rack
    Footprint::new(3.4, -3.7, 1.4, 0.45, 2.0),  // short rack
    Footprint::new(0.0, 1.05, 0.2, 0.2, 2.4),   // column
    Footprint::new(5.9, 3.5, 1.2, 1.0, 2.4),    // office block
    Footprint::new(-3.0, -0.2, PALLET_HALF_M, PALLET_HALF_M, PALLET_HEIGHT_M),
    Footprint::new(-1.6, -0.2, PALLET_HALF_M, PALLET_HALF_M, PALLET_HEIGHT_M),
    Footprint::new(1.4, -0.2, PALLET_HALF_M, PALLET_HALF_M, PALLET_HEIGHT_M),
    Footprint::new(-5.0, 3.3, PALLET_HALF_M, PALLET_HALF_M, PALLET_HEIGHT_M),
    Footprint::new(-1.8, 3.3, PALLET_HALF_M, PALLET_HALF_M, PALLET_HEIGHT_M),
    Footprint::new(0.2, 3.3, PALLET_HALF_M, PALLET_HALF_M, PALLET_HEIGHT_M),
];

/// Inside of the walls, `(min_x, max_x, min_z, max_z)`.
pub(crate) const INTERIOR: (f64, f64, f64, f64) = (-7.0, 7.0, -4.5, 4.5);

/// Whether a disc of `radius` at `(x, z)` touches anything.
pub(crate) fn collides(x: f64, z: f64, radius: f64) -> bool {
    FOOTPRINTS.iter().any(|f| f.contains(x, z, radius))
}

/// Spawns one fixed box per footprint.
pub(crate) fn spawn(world: &mut World) {
    for footprint in &FOOTPRINTS {
        let entity = spawn_named(world, "warehouse");
        world.entity_mut(entity).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: Vec3::new(
                        footprint.half_x,
                        0.5 * footprint.height,
                        footprint.half_z,
                    ),
                },
                ..Collider::default()
            },
            Transform3::from_translation_rotation(
                Vec3::new(footprint.x, 0.5 * footprint.height, footprint.z),
                Quat::IDENTITY,
            ),
        ));
    }
}
