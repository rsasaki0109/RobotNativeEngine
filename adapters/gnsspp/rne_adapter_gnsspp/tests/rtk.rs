//! libgnss++ on simulated observations: SPP, RTK against a base station, and
//! moving-baseline RTK between two antennas. Skipped when the adapter was
//! built without libgnss++.

use rne_adapter_gnsspp::{is_available, Navigation, Rtk, RtkMode, SolutionStatus, Spp};
use rne_gnss::{
    Constellation, EnuFrame, Geodetic, GnssReceiver, GnssScene, NavigationMessage, ReceiverConfig,
};
use rne_math::Vec3;

struct Setup {
    frame: EnuFrame,
    scene: GnssScene,
    constellation: Constellation,
    navigation: Navigation,
    config: ReceiverConfig,
}

fn setup() -> Setup {
    let config = ReceiverConfig::default();
    let constellation = Constellation::gps_galileo_nominal();
    let message = NavigationMessage::from_constellation(
        &constellation,
        config.epoch,
        0.0,
        config.atmosphere.klobuchar,
    );
    Setup {
        frame: EnuFrame::new(Geodetic::from_degrees(36.08, 140.08, 25.0)),
        scene: GnssScene::open_sky(),
        navigation: Navigation::new(&message).expect("navigation"),
        constellation,
        config,
    }
}

#[test]
fn spp_on_simulated_pseudoranges_is_metre_level() {
    if !is_available() {
        return;
    }
    let s = setup();
    let mut receiver = GnssReceiver::new(s.config).unwrap();
    let mut spp = Spp::new(10.0).unwrap();
    let antenna = Vec3::new(3.0, -2.0, 1.5);
    let truth = s.frame.enu_to_ecef(antenna);
    let mut valid = 0;
    for step in 0..20 {
        let epoch = receiver.observe(
            &s.scene,
            &s.constellation,
            &s.frame,
            antenna,
            step as f64 * 0.2,
        );
        let solution = spp.process(&epoch, None, &s.navigation).unwrap();
        if solution.status.has_position() {
            valid += 1;
            let error = s.frame.ecef_vector_to_enu(solution.position_ecef_m - truth);
            assert!(error.x.hypot(error.y) < 6.0, "SPP error {error:?}");
        }
    }
    assert!(valid >= 15, "{valid} valid SPP epochs");
}

#[test]
fn kinematic_rtk_fixes_to_centimetres_and_moving_base_gives_heading() {
    if !is_available() {
        return;
    }
    let s = setup();
    let base_enu = Vec3::new(-150.0, 80.0, 12.0);
    let base_ecef = s.frame.enu_to_ecef(base_enu);
    let mut base = GnssReceiver::new(ReceiverConfig {
        seed: 1,
        ..s.config
    })
    .unwrap();
    let mut front = GnssReceiver::new(ReceiverConfig {
        seed: 2,
        ..s.config
    })
    .unwrap();
    let mut rear = GnssReceiver::new(ReceiverConfig {
        seed: 3,
        ..s.config
    })
    .unwrap();
    let mut rtk = Rtk::new(RtkMode::Kinematic, 10.0, 3.0).unwrap();
    rtk.set_base_position(base_ecef).unwrap();
    let mut heading_rtk = Rtk::new(RtkMode::MovingBase, 10.0, 3.0).unwrap();

    let yaw = 0.6_f64;
    let (mut fixed, mut heading_fixed) = (0, 0);
    let mut last_error = f64::INFINITY;
    let mut last_heading_error = f64::INFINITY;
    for step in 0..150 {
        let t = step as f64 * 0.2;
        // Driving at 1 m/s along the heading.
        let center = Vec3::new(t * yaw.cos(), t * yaw.sin(), 0.8);
        let offset = Vec3::new(0.5 * yaw.cos(), 0.5 * yaw.sin(), 0.0);
        let (front_enu, rear_enu) = (center + offset, center - offset);
        let base_epoch = base.observe(&s.scene, &s.constellation, &s.frame, base_enu, t);
        let front_epoch = front.observe(&s.scene, &s.constellation, &s.frame, front_enu, t);
        let rear_epoch = rear.observe(&s.scene, &s.constellation, &s.frame, rear_enu, t);

        let solution = rtk
            .process(
                &rear_epoch,
                None,
                &base_epoch,
                Some(base_ecef),
                &s.navigation,
            )
            .unwrap();
        if solution.status == SolutionStatus::Fixed {
            fixed += 1;
            let error = s
                .frame
                .ecef_vector_to_enu(solution.position_ecef_m - s.frame.enu_to_ecef(rear_enu));
            last_error = error.length();
        }
        // Moving base: the rear antenna is the base, at its RTK position.
        let rear_ecef = if solution.status.has_position() {
            solution.position_ecef_m
        } else {
            s.frame.enu_to_ecef(rear_enu)
        };
        let baseline = heading_rtk
            .process(
                &front_epoch,
                None,
                &rear_epoch,
                Some(rear_ecef),
                &s.navigation,
            )
            .unwrap();
        if baseline.status == SolutionStatus::Fixed {
            heading_fixed += 1;
            let vector = s
                .frame
                .ecef_vector_to_enu(baseline.position_ecef_m - rear_ecef);
            let heading = vector.y.atan2(vector.x);
            last_heading_error = (heading - yaw).abs();
        }
    }
    assert!(fixed > 50, "{fixed} fixed RTK epochs");
    assert!(last_error < 0.05, "RTK error {last_error} m");
    assert!(
        heading_fixed > 50,
        "{heading_fixed} fixed moving-base epochs"
    );
    assert!(
        last_heading_error < 0.02,
        "heading error {last_heading_error} rad"
    );
}

#[test]
fn rtk_replays_bit_identically() {
    if !is_available() {
        return;
    }
    let run = || {
        let s = setup();
        let base_enu = Vec3::new(-150.0, 80.0, 12.0);
        let base_ecef = s.frame.enu_to_ecef(base_enu);
        let mut base = GnssReceiver::new(ReceiverConfig {
            seed: 1,
            ..s.config
        })
        .unwrap();
        let mut rover = GnssReceiver::new(ReceiverConfig {
            seed: 2,
            ..s.config
        })
        .unwrap();
        let mut rtk = Rtk::new(RtkMode::Kinematic, 10.0, 3.0).unwrap();
        rtk.set_base_position(base_ecef).unwrap();
        (0..30)
            .map(|step| {
                let t = step as f64 * 0.2;
                let b = base.observe(&s.scene, &s.constellation, &s.frame, base_enu, t);
                let r = rover.observe(
                    &s.scene,
                    &s.constellation,
                    &s.frame,
                    Vec3::new(t, 0.0, 1.0),
                    t,
                );
                let solution = rtk
                    .process(&r, None, &b, Some(base_ecef), &s.navigation)
                    .unwrap();
                (solution.status, solution.position_ecef_m)
            })
            .collect::<Vec<_>>()
    };
    assert_eq!(run(), run());
}

#[test]
fn qzss_joins_spp_and_rtk() {
    if !is_available() {
        return;
    }
    let config = ReceiverConfig::default();
    let constellation = Constellation::gps_galileo_qzss_nominal();
    let message = NavigationMessage::from_constellation(
        &constellation,
        config.epoch,
        0.0,
        config.atmosphere.klobuchar,
    );
    let navigation = Navigation::new(&message).expect("navigation");
    // Tokyo, where QZSS is overhead.
    let frame = EnuFrame::new(Geodetic::from_degrees(35.68, 139.77, 40.0));
    let scene = GnssScene::open_sky();
    // Over a day of QZSS orbits (whose eccentricity makes the relativistic
    // clock term tens of metres), SPP uses the QZSS satellites and stays
    // metre-level.
    let antenna = Vec3::new(0.0, 0.0, 1.5);
    for hour in [0.0, 5.5, 11.0, 17.0] {
        let t = hour * 3600.0;
        let mut receiver = GnssReceiver::new(config).unwrap();
        let epoch = receiver.observe(&scene, &constellation, &frame, antenna, t);
        let mut spp = Spp::new(10.0).unwrap();
        let message = NavigationMessage::from_constellation(
            &constellation,
            config.epoch,
            t,
            config.atmosphere.klobuchar,
        );
        let solution = spp
            .process(&epoch, None, &Navigation::new(&message).unwrap())
            .unwrap();
        let qzss_up = epoch
            .observations
            .iter()
            .filter(|o| o.id.system == rne_gnss::GnssSystem::Qzss && o.elevation_rad > 0.2)
            .count();
        assert!(qzss_up >= 2, "hour {hour}: {qzss_up} QZSS up");
        let error = frame.ecef_vector_to_enu(solution.position_ecef_m - frame.enu_to_ecef(antenna));
        assert!(error.length() < 8.0, "hour {hour}: SPP error {error:?}");
    }

    let base_enu = Vec3::new(-150.0, 80.0, 12.0);
    let base_ecef = frame.enu_to_ecef(base_enu);
    let mut base = GnssReceiver::new(ReceiverConfig { seed: 1, ..config }).unwrap();
    let mut rover = GnssReceiver::new(ReceiverConfig { seed: 3, ..config }).unwrap();
    let mut rtk = Rtk::new(RtkMode::Kinematic, 10.0, 3.0).unwrap();
    rtk.set_base_position(base_ecef).unwrap();
    let (mut fixed, mut last_error) = (0, f64::INFINITY);
    for step in 0..100 {
        let t = f64::from(step) * 0.2;
        let antenna = Vec3::new(t, 0.0, 0.8);
        let base_epoch = base.observe(&scene, &constellation, &frame, base_enu, t);
        let rover_epoch = rover.observe(&scene, &constellation, &frame, antenna, t);
        let solution = rtk
            .process(
                &rover_epoch,
                None,
                &base_epoch,
                Some(base_ecef),
                &navigation,
            )
            .unwrap();
        if solution.status == SolutionStatus::Fixed {
            fixed += 1;
            last_error = frame
                .ecef_vector_to_enu(solution.position_ecef_m - frame.enu_to_ecef(antenna))
                .length();
        }
    }
    assert!(fixed >= 50, "{fixed} fixed epochs");
    assert!(last_error < 0.05, "RTK error {last_error} m");
}

#[test]
fn beidou_joins_spp_and_rtk_with_its_geostationary_satellites() {
    if !is_available() {
        return;
    }
    let config = ReceiverConfig::default();
    let constellation = Constellation::gps_galileo_qzss_beidou_nominal();
    let frame = EnuFrame::new(Geodetic::from_degrees(35.68, 139.77, 40.0));
    let scene = GnssScene::open_sky();
    // SPP from BeiDou alone: libgnss++ must place every BeiDou satellite,
    // the geostationary ones in their tilted frame, where the simulation
    // put it.
    let beidou = Constellation::beidou_nominal();
    let antenna = Vec3::new(0.0, 0.0, 1.5);
    for hour in [0.0, 8.0, 16.0] {
        let t = hour * 3600.0;
        let mut receiver = GnssReceiver::new(config).unwrap();
        let epoch = receiver.observe(&scene, &beidou, &frame, antenna, t);
        let geo_up = epoch
            .observations
            .iter()
            .filter(|o| o.id.is_beidou_geo() && o.elevation_rad > 0.2)
            .count();
        assert!(geo_up >= 2, "hour {hour}: {geo_up} geostationary up");
        let message = NavigationMessage::from_constellation(
            &beidou,
            config.epoch,
            t,
            config.atmosphere.klobuchar,
        );
        let solution = Spp::new(10.0)
            .unwrap()
            .process(&epoch, None, &Navigation::new(&message).unwrap())
            .unwrap();
        let error = frame.ecef_vector_to_enu(solution.position_ecef_m - frame.enu_to_ecef(antenna));
        assert!(
            error.length() < 8.0,
            "hour {hour}: BeiDou-only SPP error {error:?}"
        );
    }

    // RTK with all four systems fixes and stays at centimetres.
    let message = NavigationMessage::from_constellation(
        &constellation,
        config.epoch,
        0.0,
        config.atmosphere.klobuchar,
    );
    let navigation = Navigation::new(&message).expect("navigation");
    let base_enu = Vec3::new(-150.0, 80.0, 12.0);
    let base_ecef = frame.enu_to_ecef(base_enu);
    let mut base = GnssReceiver::new(ReceiverConfig { seed: 1, ..config }).unwrap();
    let mut rover = GnssReceiver::new(ReceiverConfig { seed: 3, ..config }).unwrap();
    let mut rtk = Rtk::new(RtkMode::Kinematic, 10.0, 3.0).unwrap();
    rtk.set_base_position(base_ecef).unwrap();
    let (mut fixed, mut last_error) = (0, f64::INFINITY);
    for step in 0..100 {
        let t = f64::from(step) * 0.2;
        let antenna = Vec3::new(t, 0.0, 0.8);
        let base_epoch = base.observe(&scene, &constellation, &frame, base_enu, t);
        let rover_epoch = rover.observe(&scene, &constellation, &frame, antenna, t);
        let solution = rtk
            .process(
                &rover_epoch,
                None,
                &base_epoch,
                Some(base_ecef),
                &navigation,
            )
            .unwrap();
        if solution.status == SolutionStatus::Fixed {
            fixed += 1;
            last_error = frame
                .ecef_vector_to_enu(solution.position_ecef_m - frame.enu_to_ecef(antenna))
                .length();
        }
    }
    assert!(fixed >= 50, "{fixed} fixed epochs");
    assert!(last_error < 0.05, "RTK error {last_error} m");
}
