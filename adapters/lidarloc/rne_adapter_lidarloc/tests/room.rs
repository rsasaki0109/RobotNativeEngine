//! A room with a pillar, a scan of it from a pose the guess is off from, and
//! the alignment that finds the pose. Skipped without lidar_localization_core.

use rne_adapter_lidarloc::{LidarLoc, LidarLocConfig, Reacquisition, Tracking};
use rne_math::{Mat4, Vec3};

/// A 20 x 14 m floor, walls 3 m high and a 1 m pillar, every 0.2 m, each
/// point moved up to 3 cm along every axis (a fixed sequence): NDT's cells
/// average an exact lattice into a biased pull.
fn room() -> Vec<Vec3> {
    let mut points = Vec::new();
    let steps = |a: f64, b: f64| {
        let n = ((b - a) / 0.2).round() as i32;
        (0..=n).map(move |k| a + 0.2 * f64::from(k))
    };
    for x in steps(-10.0, 10.0) {
        for y in steps(-7.0, 7.0) {
            points.push(Vec3::new(x, y, 0.0));
        }
    }
    for z in steps(0.2, 3.0) {
        for x in steps(-10.0, 10.0) {
            points.push(Vec3::new(x, -7.0, z));
            points.push(Vec3::new(x, 7.0, z));
        }
        for y in steps(-7.0, 7.0) {
            points.push(Vec3::new(-10.0, y, z));
            points.push(Vec3::new(10.0, y, z));
        }
        for s in steps(0.0, 1.0) {
            points.extend([
                Vec3::new(3.0 + s, 2.0, z),
                Vec3::new(3.0 + s, 3.0, z),
                Vec3::new(3.0, 2.0 + s, z),
                Vec3::new(4.0, 2.0 + s, z),
            ]);
        }
    }
    let mut state = 0x2545_F491_4F6C_DD1D_u64;
    let mut jitter = || {
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        (state >> 11) as f64 / (1_u64 << 53) as f64 * 0.06 - 0.03
    };
    for p in &mut points {
        *p += Vec3::new(jitter(), jitter(), jitter());
    }
    points
}

/// Whether the core is linked; `RNE_LIDARLOC_REQUIRED=1` (CI's lidarloc job)
/// turns a missing core into a failure instead of a skip.
fn available() -> bool {
    let linked = rne_adapter_lidarloc::available();
    if !linked {
        assert!(
            std::env::var_os("RNE_LIDARLOC_REQUIRED").is_none_or(|v| v == "0"),
            "RNE_LIDARLOC_REQUIRED is set but lidar_localization_core is not linked"
        );
        eprintln!("lidar_localization_core not linked; skipping");
    }
    linked
}

fn pose(x: f64, y: f64, yaw: f64) -> Mat4 {
    Mat4::from_rotation_translation(rne_math::DQuat::from_rotation_z(yaw), Vec3::new(x, y, 0.0))
}

fn check(method: &str, tolerance_m: f64, tolerance_rad: f64) {
    let map = room();
    let truth = pose(1.0, -0.5, 0.1);
    let to_sensor = truth.inverse();
    let scan: Vec<Vec3> = map.iter().map(|&p| to_sensor.transform_point3(p)).collect();
    let mut loc = LidarLoc::new(&LidarLocConfig {
        registration_method: method.into(),
        ndt_num_threads: 2,
        // The node's 0.01 stops NDT early on this small room.
        transform_epsilon: 1e-4,
        ..LidarLocConfig::default()
    })
    .expect("core");
    loc.set_map(&map).expect("map");
    let aligned = loc.align(&scan, &pose(0.6, -0.2, 0.05)).expect("align");
    let t = aligned.pose.w_axis.truncate();
    let yaw = aligned.pose.x_axis.y.atan2(aligned.pose.x_axis.x);
    assert!(aligned.target_ready && aligned.converged, "{aligned:?}");
    assert!(
        (t.x - 1.0).hypot(t.y + 0.5) < tolerance_m,
        "{method}: {t:?}"
    );
    assert!((yaw - 0.1).abs() < tolerance_rad, "{method}: yaw {yaw}");
    assert!(aligned.inlier_ratio > 0.9, "{aligned:?}");
    assert_eq!(aligned.localizability.is_some(), method == "NDT_OMP");
    // NDT_OMP's score Hessian gives a positive-definite covariance shape.
    assert_eq!(aligned.score_hessian.is_some(), method == "NDT_OMP");
    if let Some(([[xx, xy], [_, yy]], yaw)) = aligned.planar_covariance() {
        assert!(
            xx > 0.0 && yy > 0.0 && yaw > 0.0 && xx * yy > xy * xy,
            "{xx} {xy} {yy}"
        );
    }
}

#[test]
fn finds_the_pose_in_a_room() {
    if !available() {
        return;
    }
    // GICP finds the pose exactly; NDT's 1 m cells average the room's regular
    // 0.2 m samples and it stops a few centimetres off.
    check("GICP_OMP", 0.02, 0.005);
    check("NDT_OMP", 0.1, 0.02);
}

#[test]
fn refuses_an_unknown_method() {
    if !available() {
        return;
    }
    let error = LidarLoc::new(&LidarLocConfig {
        registration_method: "NOT_A_METHOD".into(),
        ..LidarLocConfig::default()
    })
    .expect_err("refused");
    assert!(
        error.to_string().contains("Invalid registration method"),
        "{error}"
    );
}

/// `localize` along a drive: scans every 0.1 s at 2 m/s turning 0.05 rad a
/// scan, seeded by the previous delta; every scan is accepted and the track
/// stays on the truth. A seed 3 m off with the seed correction guard on is
/// then rejected (by the guard or the score) and the track holds.
#[test]
fn tracks_a_drive_and_rejects_a_bad_seed() {
    if !available() {
        return;
    }
    let map = room();
    let mut loc = LidarLoc::new(&LidarLocConfig {
        registration_method: "GICP_OMP".into(),
        ndt_num_threads: 2,
        ..LidarLocConfig::default()
    })
    .expect("core");
    loc.set_map(&map).expect("map");
    loc.set_tracking(&Tracking {
        score_threshold: 0.5,
        enable_seed_correction_guard: true,
        seed_correction_guard_translation_m: 0.5,
        seed_correction_guard_warmup_accepts: 3,
        ..Tracking::default()
    })
    .expect("tracking");
    let scan_from = |m: &Mat4| -> Vec<Vec3> {
        let to_sensor = m.inverse();
        map.iter().map(|&p| to_sensor.transform_point3(p)).collect()
    };
    let (mut x, mut y, mut yaw) = (-4.0_f64, -3.0_f64, 0.0_f64);
    assert!(
        loc.localize(&scan_from(&pose(x, y, yaw)), 0.0, None)
            .is_err(),
        "nothing before the initial pose"
    );
    loc.set_initial_pose(&pose(x, y, yaw), 0.0).expect("start");
    for k in 1..=20 {
        x += 0.2 * yaw.cos();
        y += 0.2 * yaw.sin();
        yaw += 0.05;
        let update = loc
            .localize(&scan_from(&pose(x, y, yaw)), 0.1 * f64::from(k), None)
            .expect("localize");
        let t = update.pose.w_axis;
        assert!(
            update.accepted && (t.x - x).hypot(t.y - y) < 0.05,
            "scan {k}: {update:?}"
        );
        assert_eq!(update.status, "ok");
    }
    let off = pose(x + 3.0, y, yaw);
    let update = loc
        .localize(&scan_from(&pose(x, y, yaw)), 2.1, Some(&off))
        .expect("localize");
    assert!(!update.accepted, "{update:?}");
    assert_eq!(update.consecutive_rejected_updates, 1);
    let t = update.pose.w_axis;
    assert!((t.x - x).hypot(t.y - y) < 0.05, "{update:?}");
}

/// Local re-acquisition: tracking with odometry seeds, the odometry then 4 m
/// off; after three rejected scans the search proposes the true pose, and the
/// next scan, seeded from it, is accepted there.
#[test]
fn reacquires_after_odometry_drift() {
    if !available() {
        return;
    }
    let map = room();
    let mut loc = LidarLoc::new(&LidarLocConfig {
        registration_method: "GICP_OMP".into(),
        ndt_num_threads: 2,
        ..LidarLocConfig::default()
    })
    .expect("core");
    loc.set_reacquisition(&Reacquisition {
        min_rejections: 3,
        attempt_interval_scans: 1,
        search_radius_m: 6.0,
        ..Reacquisition::default()
    })
    .expect("reacquisition");
    loc.set_map(&map).expect("map");
    loc.set_tracking(&Tracking {
        score_threshold: 0.5,
        enable_seed_correction_guard: true,
        seed_correction_guard_translation_m: 0.5,
        seed_correction_guard_warmup_accepts: 3,
        ..Tracking::default()
    })
    .expect("tracking");
    let scan_from = |m: &Mat4| -> Vec<Vec3> {
        let to_sensor = m.inverse();
        map.iter().map(|&p| to_sensor.transform_point3(p)).collect()
    };
    let (mut x, y) = (-5.0_f64, -3.0_f64);
    loc.set_initial_pose(&pose(x, y, 0.0), 0.0).expect("start");
    let mut proposed = None;
    for k in 1..=12 {
        x += 0.2;
        let truth = pose(x, y, 0.0);
        let seed = if k <= 6 {
            truth
        } else {
            pose(x + 4.0, y + 1.0, 0.0)
        };
        let update = loc
            .localize(&scan_from(&truth), 0.1 * f64::from(k), Some(&seed))
            .expect("localize");
        if k <= 6 {
            assert!(
                update.accepted && !update.reacquisition_attempted,
                "{update:?}"
            );
            continue;
        }
        if update.seeded_from_reacquisition {
            let t = update.pose.w_axis;
            assert!(
                update.accepted && (t.x - x).hypot(t.y - y) < 0.05,
                "{update:?}"
            );
            assert_eq!((proposed, k), (Some(9), 10));
            return;
        }
        assert!(!update.accepted, "scan {k}: {update:?}");
        if let Some(p) = update.reacquisition_proposal {
            assert_eq!(update.reacquisition_reason, "proposed");
            assert!((p.w_axis.x - x).hypot(p.w_axis.y - y) < 0.1, "{p:?}");
            proposed = Some(k);
        }
    }
    panic!("not re-acquired");
}
