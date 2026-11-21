//! Determinism regression tests for ray-traced GNSS and the GNSS / LiDAR
//! localization arbiter.
//!
//! A robot drives open-loop from open sky through a street canyon into a
//! roofed hall. The run is executed twice and every fix, satellite
//! classification, fused pose, and source switch is hashed with FNV-1a.

use rne_gnss::{
    Constellation, EnuFrame, Geodetic, GnssReceiver, GnssScene, ReceiverConfig, SceneBox,
    SignalPath, SurfaceMaterial,
};
use rne_math::Vec3;
use rne_nav::{FrameId, GridCoord, LaserScan2d, OccupancyGrid, Pose2d};
use rne_slam::{
    AbsolutePositionFix, AmclConfig, HybridLocalizer, HybridLocalizerConfig, LocalizationSource,
};
use std::f64::consts::TAU;

fn fnv1a(hash: &mut u64, bytes: &[u8]) {
    for byte in bytes {
        *hash ^= u64::from(*byte);
        *hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
}

fn scene() -> GnssScene {
    let mut scene = GnssScene::open_sky();
    scene.add_box(SceneBox::building(
        30.0,
        12.0,
        70.0,
        40.0,
        35.0,
        SurfaceMaterial::GLASS,
    ));
    scene.add_box(SceneBox::building(
        30.0,
        -40.0,
        70.0,
        -12.0,
        30.0,
        SurfaceMaterial::CONCRETE,
    ));
    // A roofed 20 m x 12 m hall with a door on its west wall.
    let concrete = SurfaceMaterial::CONCRETE;
    scene.add_box(SceneBox::building(80.0, -6.0, 80.3, -1.5, 3.5, concrete));
    scene.add_box(SceneBox::building(80.0, 1.5, 80.3, 6.0, 3.5, concrete));
    scene.add_box(SceneBox::building(99.7, -6.0, 100.0, 6.0, 3.5, concrete));
    scene.add_box(SceneBox::building(80.0, -6.0, 100.0, -5.7, 3.5, concrete));
    scene.add_box(SceneBox::building(80.0, 5.7, 100.0, 6.0, 3.5, concrete));
    scene.add_box(SceneBox::building(88.0, 2.0, 89.0, 3.0, 3.5, concrete));
    scene.add_box(SceneBox::building(93.0, -3.5, 94.5, -2.5, 3.5, concrete));
    scene.add_box(SceneBox::new(
        Vec3::new(80.0, -6.0, 3.5),
        Vec3::new(100.0, 6.0, 3.8),
        concrete,
    ));
    scene
}

fn prior_map(scene: &GnssScene) -> OccupancyGrid {
    let mut grid = OccupancyGrid::new(300, 200, 0.1, Pose2d::new(75.0, -10.0, 0.0)).unwrap();
    for y in 0..grid.height() {
        for x in 0..grid.width() {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            let center = grid.grid_to_world(coord);
            if scene.inside_solid(Vec3::new(center.x, center.y, 0.3)) {
                grid.mark_occupied(coord);
            } else {
                grid.mark_free(coord);
            }
        }
    }
    grid
}

fn scan(scene: &GnssScene, pose: Pose2d, time_s: f64) -> LaserScan2d {
    let beams = 180;
    let ranges_m = (0..beams)
        .map(|beam| {
            let angle = pose.yaw_rad + TAU * beam as f64 / beams as f64;
            scene
                .raycast(
                    Vec3::new(pose.x_m, pose.y_m, 0.3),
                    Vec3::new(angle.cos(), angle.sin(), 0.0),
                    15.0,
                )
                .map_or(f64::INFINITY, |hit| hit.distance_m)
        })
        .collect();
    LaserScan2d {
        time_s,
        frame: FrameId::new("laser"),
        angle_min_rad: 0.0,
        angle_increment_rad: TAU / beams as f64,
        range_min_m: 0.05,
        range_max_m: 15.0,
        ranges_m,
    }
}

struct Run {
    hash: u64,
    sources: Vec<LocalizationSource>,
    nlos_seen: usize,
    final_error_m: f64,
}

fn run() -> Run {
    let scene = scene();
    let frame = EnuFrame::new(Geodetic::from_degrees(35.0, 139.0, 10.0));
    let constellation = Constellation::gps_galileo_nominal();
    let mut receiver = GnssReceiver::new(ReceiverConfig::default()).unwrap();
    let config = HybridLocalizerConfig {
        amcl: AmclConfig {
            particle_count: 200,
            ..AmclConfig::default()
        },
        ..HybridLocalizerConfig::default()
    };
    let start = Pose2d::new(0.0, 0.0, 0.0);
    let mut localizer = HybridLocalizer::new(prior_map(&scene), start, config).unwrap();

    let dt = 0.1;
    let speed = 1.5;
    let mut hash = 0xcbf2_9ce4_8422_2325_u64;
    let mut sources = Vec::new();
    let mut nlos_seen = 0;
    let mut truth = start;
    for step in 0..620 {
        let t = step as f64 * dt;
        truth.x_m = (truth.x_m + speed * dt).min(92.0);
        let v = if truth.x_m < 92.0 { speed } else { 0.0 };
        localizer.predict(dt, v * 1.01, 0.0).unwrap();
        let antenna = Vec3::new(truth.x_m, truth.y_m, 0.8);
        for fix in receiver.step(&scene, &constellation, &frame, antenna, 5_000.0 + t) {
            fnv1a(&mut hash, &fix.position_enu_m.x.to_le_bytes());
            fnv1a(&mut hash, &fix.position_enu_m.y.to_le_bytes());
            fnv1a(&mut hash, &(fix.satellites_used as u64).to_le_bytes());
            let absolute = fix.has_position().then_some(AbsolutePositionFix {
                time_s: t,
                x_m: fix.position_enu_m.x,
                y_m: fix.position_enu_m.y,
                horizontal_std_m: fix.horizontal_std_m,
                satellites_used: fix.satellites_used,
                hdop: fix.dop.hdop,
                consistent: fix.consistent,
                satellites_excluded: fix.excluded.len(),
            });
            localizer.update_gnss(absolute.as_ref(), t).unwrap();
        }
        if let Some(epoch) = receiver.last_epoch() {
            nlos_seen += epoch.count(SignalPath::NonLineOfSight);
            for observation in &epoch.observations {
                fnv1a(&mut hash, &[observation.path as u8]);
            }
        }
        localizer
            .update_lidar(&scan(&scene, truth, t), truth, Pose2d::IDENTITY, t)
            .unwrap();
        let pose = localizer.pose();
        fnv1a(&mut hash, &pose.x_m.to_le_bytes());
        fnv1a(&mut hash, &pose.y_m.to_le_bytes());
        fnv1a(&mut hash, &pose.yaw_rad.to_le_bytes());
        if sources.last() != Some(&localizer.source()) {
            sources.push(localizer.source());
        }
    }
    let pose = localizer.pose();
    Run {
        hash,
        sources,
        nlos_seen,
        final_error_m: (pose.x_m - truth.x_m).hypot(pose.y_m - truth.y_m),
    }
}

#[test]
fn gnss_lidar_handover_replays_bit_identically() {
    let first = run();
    let second = run();
    assert_eq!(first.hash, second.hash);
    assert_eq!(first.sources, second.sources);
}

#[test]
fn canyon_has_nlos_and_the_hall_hands_over_to_lidar() {
    let run = run();
    assert!(run.nlos_seen > 0, "the canyon produced no NLOS receptions");
    assert_eq!(run.sources.first(), Some(&LocalizationSource::Gnss));
    assert_eq!(
        run.sources.last(),
        Some(&LocalizationSource::Lidar),
        "{:?}",
        run.sources
    );
    assert!(
        run.final_error_m < 0.2,
        "final error {} m",
        run.final_error_m
    );
}
