//! ECS systems for GNSS simulation.

use crate::components::GnssAntenna;
use crate::resources::GnssEnvironment;
use bevy_ecs::prelude::{Entity, World};

/// Steps every [`GnssAntenna`] to `time_s` against the [`GnssEnvironment`].
///
/// Antennas are visited in entity order, so the result does not depend on
/// archetype layout. Returns the number of fixes released this step.
pub fn step_gnss_antennas(world: &mut World, time_s: f64) -> usize {
    let Some(environment) = world.get_resource::<GnssEnvironment>().cloned() else {
        return 0;
    };
    let mut query = world.query::<(Entity, &mut GnssAntenna)>();
    let mut antennas: Vec<(Entity, bevy_ecs::prelude::Mut<'_, GnssAntenna>)> =
        query.iter_mut(world).collect();
    antennas.sort_by_key(|(entity, _)| *entity);
    let mut released = 0;
    for (_, antenna) in &mut antennas {
        let position = antenna.antenna_enu_m;
        let fixes = antenna.receiver.step(
            &environment.scene,
            &environment.constellation,
            &environment.frame,
            position,
            time_s,
        );
        released += fixes.len();
        if let Some(fix) = fixes.into_iter().last() {
            antenna.latest_fix = Some(fix);
        }
    }
    released
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geodesy::{EnuFrame, Geodetic};
    use crate::orbit::Constellation;
    use crate::receiver::{GnssReceiver, ReceiverConfig};
    use crate::scene::GnssScene;
    use rne_math::Vec3;

    #[test]
    fn system_releases_fixes_into_components() {
        let mut world = World::new();
        world.insert_resource(GnssEnvironment {
            scene: GnssScene::open_sky(),
            constellation: Constellation::gps_galileo_nominal(),
            frame: EnuFrame::new(Geodetic::from_degrees(36.0, 140.0, 20.0)),
        });
        let receiver = GnssReceiver::new(ReceiverConfig {
            latency_s: 0.0,
            ..ReceiverConfig::default()
        })
        .unwrap();
        let entity = world
            .spawn(GnssAntenna::new(receiver, Vec3::new(1.0, 2.0, 1.0)))
            .id();
        assert_eq!(step_gnss_antennas(&mut world, 0.0), 1);
        let fix = world
            .get::<GnssAntenna>(entity)
            .unwrap()
            .latest_fix
            .clone()
            .unwrap();
        assert!(fix.has_position());
        assert!((fix.position_enu_m.x - 1.0).abs() < 10.0);
    }
}
