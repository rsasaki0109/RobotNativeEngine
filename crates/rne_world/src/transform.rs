//! World transform helpers.

use bevy_ecs::prelude::World;
use rne_ecs::{Entity, Parent};

use crate::Transform3;

/// Returns the composed world transform for an entity in a parent/child hierarchy.
pub fn world_transform_of(world: &World, entity: Entity) -> Transform3 {
    // One entity lookup serves both components, as `World::get` would for each.
    let Ok(entity_ref) = world.get_entity(entity) else {
        return Transform3::default();
    };
    let local = entity_ref.get::<Transform3>().copied().unwrap_or_default();
    let Some(parent) = entity_ref.get::<Parent>() else {
        return local;
    };
    world_transform_of(world, parent.0).mul_transform(&local)
}

/// Memoized [`world_transform_of`] for a pass that changes no entity's
/// [`Transform3`] or [`Parent`]: each entity's world transform is composed
/// once, in the same order, so every value equals `world_transform_of`'s.
#[derive(Debug, Default)]
pub struct WorldTransformCache {
    transforms: Vec<Option<(Entity, Transform3)>>,
}

impl WorldTransformCache {
    /// An empty cache.
    pub fn new() -> Self {
        Self::default()
    }

    /// The world transform of `entity`, as [`world_transform_of`] returns it.
    pub fn get(&mut self, world: &World, entity: Entity) -> Transform3 {
        let index = entity.index() as usize;
        if let Some(Some((cached, transform))) = self.transforms.get(index) {
            if *cached == entity {
                return *transform;
            }
        }
        let Ok(entity_ref) = world.get_entity(entity) else {
            return Transform3::default();
        };
        let local = entity_ref.get::<Transform3>().copied().unwrap_or_default();
        let transform = match entity_ref.get::<Parent>() {
            Some(parent) => self.get(world, parent.0).mul_transform(&local),
            None => local,
        };
        if self.transforms.len() <= index {
            self.transforms.resize(index + 1, None);
        }
        self.transforms[index] = Some((entity, transform));
        transform
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy_ecs::world::World as EcsWorld;
    use rne_ecs::{spawn_named, Parent};
    use rne_math::{Quat, Vec3};

    #[test]
    fn world_transform_composes_parent_chain() {
        let mut world = EcsWorld::new();
        let parent = spawn_named(&mut world, "parent");
        let child = spawn_named(&mut world, "child");
        world
            .entity_mut(parent)
            .insert(Transform3::from_translation_rotation(
                Vec3::new(1.0, 0.0, 0.0),
                Quat::IDENTITY,
            ));
        world.entity_mut(child).insert((
            Parent(parent),
            Transform3::from_translation_rotation(Vec3::new(0.0, 2.0, 0.0), Quat::IDENTITY),
        ));

        let global = world_transform_of(&world, child);
        assert!((global.translation.x - 1.0).abs() < 1e-10);
        assert!((global.translation.y - 2.0).abs() < 1e-10);
    }

    #[test]
    fn cache_returns_the_uncached_transforms_bit_for_bit() {
        let mut world = EcsWorld::new();
        let mut chain = vec![spawn_named(&mut world, "root")];
        world.entity_mut(chain[0]).insert(Transform3 {
            translation: Vec3::new(0.3, -1.7, 2.9),
            rotation: Quat::from_rotation_x(0.4) * Quat::from_rotation_z(2.3),
            scale: Vec3::new(1.0, 2.0, 0.5),
        });
        for index in 1..6 {
            let entity = spawn_named(&mut world, "link");
            world.entity_mut(entity).insert((
                Parent(chain[index - 1]),
                Transform3::from_translation_rotation(
                    Vec3::new(0.1 * index as f64, -0.37, 0.05),
                    Quat::from_rotation_y(0.3 + index as f64),
                ),
            ));
            chain.push(entity);
        }
        let orphan = spawn_named(&mut world, "untransformed");
        let mut cache = WorldTransformCache::new();
        for &entity in chain.iter().rev().chain(chain.iter()).chain([&orphan]) {
            let expected = world_transform_of(&world, entity);
            let cached = cache.get(&world, entity);
            assert_eq!(
                cached.translation.to_array().map(f64::to_bits),
                expected.translation.to_array().map(f64::to_bits)
            );
            assert_eq!(
                cached.rotation.to_array().map(f64::to_bits),
                expected.rotation.to_array().map(f64::to_bits)
            );
            assert_eq!(
                cached.scale.to_array().map(f64::to_bits),
                expected.scale.to_array().map(f64::to_bits)
            );
        }
    }
}
