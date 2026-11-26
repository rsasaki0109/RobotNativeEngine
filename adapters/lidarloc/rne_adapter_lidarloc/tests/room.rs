//! A room with a pillar, a scan of it from a pose the guess is off from, and
//! the alignment that finds the pose. Skipped without lidar_localization_core.

use rne_adapter_lidarloc::{available, LidarLoc, LidarLocConfig};
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
}

#[test]
fn finds_the_pose_in_a_room() {
    if !available() {
        eprintln!("lidar_localization_core not linked; skipping");
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
