//! A LiDAR + IMU rig simulated in RNE, recorded as a ROS 2 bag for Calibrex.
//!
//! A hand-held rig carries a 16-channel spinning LiDAR and an IMU mounted with
//! a known rotation and lever arm. It moves through a room built from Rapier
//! colliders; the LiDAR ray-casts the room column by column while the rig moves
//! (so scans are motion-distorted, with a per-point time), and the IMU is
//! sampled from the rig's rigid-body kinematics with RNE's IMU error model.
//!
//! The output is what a real robot would record:
//!
//! * `rig.mcap` — a ROS 2 bag (MCAP storage, CDR payloads) with `/imu/data`
//!   (`sensor_msgs/Imu`, 200 Hz), `/lidar/points` (`sensor_msgs/PointCloud2`,
//!   10 Hz, fields `x y z intensity ring time`) and `/tf_static` holding the
//!   true `lidar -> imu` mounting;
//! * `good.yaml` — the true mounting as a Calibrex `--tf` frames file;
//! * `turned.yaml` — the same mounting with the IMU turned a few degrees about
//!   the LiDAR z axis, a calibration that should fail a check.
//!
//! ```text
//! cargo run --release -p calibrex_rig_bag --example 149_calibrex_rig_bag -- --out target/calibrex_rig
//! calibrex check target/calibrex_rig/rig.mcap --tf target/calibrex_rig/good.yaml
//! calibrex check target/calibrex_rig/rig.mcap --tf target/calibrex_rig/turned.yaml
//! ```
//!
//! Bag frames follow ROS conventions (x forward, z up) although the RNE world
//! is y-up: the rig pose is converted once, at the sensor frame.

use std::f64::consts::{FRAC_PI_2, PI};
use std::fs::{self, File};
use std::io::{self, BufWriter};
use std::path::{Path, PathBuf};

use rne_adapter_ros2::{
    encode_imu, encode_pointcloud2, encode_tf_message, to_ros_imu, to_ros_pointcloud2,
    to_ros_transform_stamped, McapWriter, RosTfMessage, IMU_SCHEMA, POINTCLOUD2_SCHEMA,
    TF_MESSAGE_SCHEMA,
};
use rne_core::SimTime;
use rne_data::PointCloud;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, PhysicsWorldId, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_sensor::{
    sample_imu_stateful_diagnostic_with_kinematics, sample_lidar_swept, ImuAxisErrors,
    ImuKinematicState, ImuMount, ImuSpec, ImuState, LidarSpec, LidarSweep, SensorNoiseKey,
};
use rne_world::Transform3;

const IMU_TOPIC: &str = "/imu/data";
const LIDAR_TOPIC: &str = "/lidar/points";
const LIDAR_FRAME: &str = "lidar";
const IMU_FRAME: &str = "imu";
/// Bag clock at simulation time zero: an ordinary wall-clock epoch, as a robot records.
const BAG_EPOCH_NS: u64 = 1_700_000_000_000_000_000;
const IMU_RATE_HZ: u64 = 200;
const LIDAR_RATE_HZ: u64 = 10;
/// Each scan is cast in this many sub-sweeps between exactly sampled rig poses.
const SEGMENTS_PER_SCAN: u32 = 20;
const COLUMNS_PER_SEGMENT: u32 = 18;
const CHANNELS: u16 = 16;
const ROOT_SEED: u64 = 0x5EED_CA1B;
const LIDAR_SENSOR_ID: u64 = 1;
const IMU_SENSOR_ID: u64 = 2;
/// The `turned.yaml` candidate turns the IMU this much about the LiDAR z axis.
const TURN_DEG: f64 = 3.0;
/// The true IMU rotation in the LiDAR frame, intrinsic x-y-z angles in degrees.
const MOUNT_RPY_DEG: [f64; 3] = [2.0, -3.0, 92.0];

/// The mounting Calibrex is asked to recover, in ROS conventions.
#[derive(Clone, Copy, Debug)]
struct Mounting {
    /// Rotation taking IMU-frame vectors into the LiDAR frame (`R_lidar_imu`).
    lidar_from_imu: Quat,
    /// IMU origin in the LiDAR frame, meters.
    lever_arm_m: Vec3,
}

fn true_mounting() -> Mounting {
    Mounting {
        lidar_from_imu: Quat::from_rotation_x(MOUNT_RPY_DEG[0].to_radians())
            * Quat::from_rotation_y(MOUNT_RPY_DEG[1].to_radians())
            * Quat::from_rotation_z(MOUNT_RPY_DEG[2].to_radians()),
        lever_arm_m: Vec3::new(0.10, -0.06, 0.15),
    }
}

/// Rotation taking ROS-convention (z-up) vectors into RNE's y-up convention.
fn ros_to_rne() -> Quat {
    Quat::from_rotation_x(-FRAC_PI_2)
}

/// The rig (LiDAR frame) pose in a z-up world: a smooth hand-held motion that
/// turns about all three axes, so that every rotation axis of the mounting is
/// observable.
fn rig_pose_ros(time_s: f64) -> (Quat, Vec3) {
    let rotation_vector = Vec3::new(
        0.5 * (1.3 * time_s).sin(),
        0.45 * (1.1 * time_s + 0.4).sin(),
        1.0 * (0.6 * time_s).sin(),
    );
    let position = Vec3::new(
        0.6 * (0.5 * time_s).sin() + 0.04 * time_s,
        0.5 * (0.35 * time_s + 0.2).sin(),
        1.3 + 0.15 * (0.8 * time_s).sin(),
    );
    (Quat::from_scaled_axis(rotation_vector), position)
}

/// World pose of the RNE LiDAR sensor frame (y-up scan frame) at a time.
fn lidar_pose_rne(time_s: f64) -> Transform3 {
    let (rotation, position) = rig_pose_ros(time_s);
    let m = ros_to_rne();
    Transform3::from_translation_rotation(m * position, m * rotation * m.inverse())
}

/// World-frame linear and angular velocity of the rig, by central differences.
fn rig_velocity_rne(time_s: f64) -> (Vec3, Vec3) {
    const STEP_S: f64 = 1.0e-5;
    let before = lidar_pose_rne(time_s - STEP_S);
    let after = lidar_pose_rne(time_s + STEP_S);
    let linear = (after.translation - before.translation) / (2.0 * STEP_S);
    let angular = (after.rotation * before.rotation.inverse()).to_scaled_axis() / (2.0 * STEP_S);
    (linear, angular)
}

/// One static box of the room, in z-up coordinates.
struct RoomBox {
    name: &'static str,
    center_m: Vec3,
    half_extents_m: Vec3,
    rotation: Quat,
}

fn room_boxes() -> Vec<RoomBox> {
    let level = Quat::IDENTITY;
    let euler = |roll: f64, pitch: f64, yaw: f64| {
        Quat::from_rotation_z(yaw.to_radians())
            * Quat::from_rotation_y(pitch.to_radians())
            * Quat::from_rotation_x(roll.to_radians())
    };
    let boxed = |name, center: [f64; 3], half: [f64; 3], rotation| RoomBox {
        name,
        center_m: Vec3::from_array(center),
        half_extents_m: Vec3::from_array(half),
        rotation,
    };
    vec![
        boxed("floor", [0.0, 0.0, -0.05], [7.0, 6.0, 0.05], level),
        boxed("ceiling", [0.0, 0.0, 3.05], [7.0, 6.0, 0.05], level),
        boxed("wall_east", [6.05, 0.0, 1.5], [0.05, 6.0, 1.6], level),
        boxed("wall_west", [-6.05, 0.0, 1.5], [0.05, 6.0, 1.6], level),
        boxed("wall_north", [0.0, 5.05, 1.5], [7.0, 0.05, 1.6], level),
        boxed("wall_south", [0.0, -5.05, 1.5], [7.0, 0.05, 1.6], level),
        // Slanted panels, pillars and a crate break the room's symmetry so
        // scan registration is constrained in every direction.
        boxed(
            "panel_ne",
            [3.6, 3.2, 1.1],
            [1.5, 0.05, 1.0],
            euler(15.0, 0.0, 40.0),
        ),
        boxed(
            "panel_sw",
            [-3.4, -3.0, 1.3],
            [1.2, 0.05, 1.2],
            euler(0.0, 20.0, -30.0),
        ),
        boxed("pillar_se", [2.6, -2.8, 1.5], [0.2, 0.2, 1.5], level),
        boxed(
            "pillar_nw",
            [-2.8, 2.2, 1.5],
            [0.3, 0.3, 1.5],
            euler(0.0, 0.0, 20.0),
        ),
        boxed(
            "crate",
            [0.8, 3.6, 0.4],
            [0.6, 0.4, 0.4],
            euler(0.0, 0.0, 15.0),
        ),
        boxed(
            "ramp",
            [-1.5, -3.6, 0.3],
            [1.0, 0.8, 0.05],
            euler(18.0, 0.0, 0.0),
        ),
    ]
}

fn spawn_room(world: &mut World) {
    let m = ros_to_rne();
    for room_box in room_boxes() {
        let entity = spawn_named(world, room_box.name);
        world.entity_mut(entity).insert((
            RigidBody {
                body_type: RigidBodyType::Fixed,
                ..RigidBody::default()
            },
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: room_box.half_extents_m,
                },
                ..Collider::default()
            },
            Transform3::from_translation_rotation(m * room_box.center_m, m * room_box.rotation),
        ));
    }
}

fn imu_spec() -> ImuSpec {
    ImuSpec {
        seed: 7,
        gyro: ImuAxisErrors {
            random_walk: 2.0e-4,
            turn_on_bias: Vec3::new(0.004, -0.003, 0.005),
            ..ImuAxisErrors::default()
        },
        accel: ImuAxisErrors {
            random_walk: 2.0e-3,
            turn_on_bias: Vec3::new(0.05, -0.04, 0.03),
            ..ImuAxisErrors::default()
        },
        ..ImuSpec::default()
    }
}

fn lidar_segment_spec(segment: u32) -> LidarSpec {
    let columns_per_scan = SEGMENTS_PER_SCAN * COLUMNS_PER_SEGMENT;
    let column_step_rad = 2.0 * PI / f64::from(columns_per_scan);
    let first = -PI + f64::from(segment * COLUMNS_PER_SEGMENT) * column_step_rad;
    LidarSpec {
        ray_count: COLUMNS_PER_SEGMENT,
        min_angle_rad: first,
        max_angle_rad: first + f64::from(COLUMNS_PER_SEGMENT - 1) * column_step_rad,
        channel_count: CHANNELS,
        min_elevation_rad: (-15.0_f64).to_radians(),
        max_elevation_rad: 15.0_f64.to_radians(),
        rotation_period_s: segment_duration_s(),
        min_range_m: 0.3,
        max_range_m: 40.0,
        height_offset_m: 0.0,
        range_noise_stddev_m: 0.01,
        seed: 11,
        ..LidarSpec::default()
    }
}

fn segment_duration_s() -> f64 {
    1.0 / (LIDAR_RATE_HZ as f64 * f64::from(SEGMENTS_PER_SCAN))
}

fn seconds_to_ns(time_s: f64) -> u64 {
    (time_s * 1.0e9).round() as u64
}

/// One recorded message, before it is sorted into the bag by log time.
struct Event {
    log_time_ns: u64,
    topic: usize,
    payload: Vec<u8>,
}

/// What [`simulate`] recorded.
#[derive(Debug)]
struct Recording {
    imu_messages: usize,
    lidar_messages: usize,
    lidar_points: usize,
}

/// Casts one full scan starting at `start_s`, returned in the ROS LiDAR frame
/// with per-point times after the scan start.
fn cast_scan(
    backend: &RapierBackend,
    physics_world: PhysicsWorldId,
    world: &World,
    scan_index: u64,
    start_s: f64,
) -> PointCloud {
    let to_ros = ros_to_rne().inverse();
    let mut scan = PointCloud::new();
    for segment in 0..SEGMENTS_PER_SCAN {
        let segment_start_s = start_s + f64::from(segment) * segment_duration_s();
        let sweep = LidarSweep::new(
            lidar_pose_rne(segment_start_s),
            lidar_pose_rne(segment_start_s + segment_duration_s()),
        );
        let key = SensorNoiseKey::new(
            ROOT_SEED,
            0,
            LIDAR_SENSOR_ID,
            scan_index * u64::from(SEGMENTS_PER_SCAN) + u64::from(segment),
        );
        let cloud = sample_lidar_swept(
            backend,
            physics_world,
            world,
            &sweep,
            &lidar_segment_spec(segment),
            key,
        );
        for index in 0..cloud.points_m.len() {
            let point_time_s = segment_start_s + cloud.timestamps_s[index];
            // The point in the sensor frame at the instant it was measured.
            let local = lidar_pose_rne(point_time_s)
                .to_matrix()
                .inverse()
                .transform_point3(cloud.points_m[index]);
            scan.push_return(
                to_ros * local,
                cloud.intensities[index],
                segment * COLUMNS_PER_SEGMENT + cloud.ray_indices[index],
                cloud.return_indices[index],
                cloud.channel_indices[index],
                point_time_s - start_s,
            );
        }
    }
    scan
}

/// Simulates the rig for `duration_s` and writes the bag to `bag_path`.
fn simulate(bag_path: &Path, duration_s: f64, mounting: Mounting) -> io::Result<Recording> {
    let mut world = World::new();
    spawn_room(&mut world);
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .map_err(io::Error::other)?;
    // The room is static: one sync builds the colliders and the ray-cast index.
    backend
        .sync_from_ecs(&mut world, physics_world)
        .map_err(io::Error::other)?;

    // The rig body is the LiDAR frame; it carries no collider so rays never hit it.
    let rig = spawn_named(&mut world, "rig");
    world.entity_mut(rig).insert((
        lidar_pose_rne(0.0),
        RigidBody {
            body_type: RigidBodyType::Kinematic,
            ..RigidBody::default()
        },
    ));
    let m = ros_to_rne();
    let imu = spawn_named(&mut world, "imu");
    world.entity_mut(imu).insert(ImuMount {
        body_entity: rig,
        body_from_sensor: Transform3::from_translation_rotation(
            m * mounting.lever_arm_m,
            m * mounting.lidar_from_imu,
        ),
    });

    let mut events = Vec::new();
    let mut recording = Recording {
        imu_messages: 0,
        lidar_messages: 0,
        lidar_points: 0,
    };
    let spec = imu_spec();
    let mut imu_state = ImuState::default();
    let mut kinematic_state = ImuKinematicState::default();
    let imu_samples = (duration_s * IMU_RATE_HZ as f64).floor() as u64;
    for index in 0..imu_samples {
        let time_s = index as f64 / IMU_RATE_HZ as f64;
        set_rig_state(&mut world, rig, time_s);
        let sim_time = SimTime::from_ticks(seconds_to_ns(time_s));
        let sample = sample_imu_stateful_diagnostic_with_kinematics(
            &world,
            imu,
            &spec,
            SensorNoiseKey::new(ROOT_SEED, 0, IMU_SENSOR_ID, index),
            sim_time,
            &mut imu_state,
            &mut kinematic_state,
        )
        .map_err(io::Error::other)?;
        let stamp = SimTime::from_ticks(BAG_EPOCH_NS + seconds_to_ns(time_s));
        events.push(Event {
            log_time_ns: stamp.ticks(),
            topic: 0,
            payload: encode_imu(&to_ros_imu(&sample.measurement, stamp, IMU_FRAME)),
        });
        recording.imu_messages += 1;
    }

    let scans = (duration_s * LIDAR_RATE_HZ as f64).floor() as u64;
    for scan_index in 0..scans.saturating_sub(1) {
        let start_s = scan_index as f64 / LIDAR_RATE_HZ as f64;
        let scan = cast_scan(&backend, physics_world, &world, scan_index, start_s);
        let stamp = SimTime::from_ticks(BAG_EPOCH_NS + seconds_to_ns(start_s));
        recording.lidar_points += scan.points_m.len();
        events.push(Event {
            log_time_ns: stamp.ticks(),
            topic: 1,
            payload: encode_pointcloud2(&to_ros_pointcloud2(&scan, stamp, LIDAR_FRAME)),
        });
        recording.lidar_messages += 1;
    }

    let tf_static = RosTfMessage {
        transforms: vec![to_ros_transform_stamped(
            LIDAR_FRAME,
            IMU_FRAME,
            Transform3::from_translation_rotation(mounting.lever_arm_m, mounting.lidar_from_imu),
            SimTime::from_ticks(BAG_EPOCH_NS),
        )],
    };
    events.push(Event {
        log_time_ns: BAG_EPOCH_NS,
        topic: 2,
        payload: encode_tf_message(&tf_static),
    });
    // A recorder writes in arrival order; the sort is stable, so equal stamps
    // keep the order they were produced in.
    events.sort_by_key(|event| event.log_time_ns);

    let mut writer = McapWriter::new(BufWriter::new(File::create(bag_path)?))?;
    let channels = [
        writer.add_channel(IMU_TOPIC, IMU_SCHEMA, false)?,
        writer.add_channel(LIDAR_TOPIC, POINTCLOUD2_SCHEMA, false)?,
        writer.add_channel("/tf_static", TF_MESSAGE_SCHEMA, true)?,
    ];
    for event in &events {
        writer.write_message(
            channels[event.topic],
            event.log_time_ns,
            event.log_time_ns,
            &event.payload,
        )?;
    }
    writer.finish()?;
    Ok(recording)
}

fn set_rig_state(world: &mut World, rig: Entity, time_s: f64) {
    let (linear, angular) = rig_velocity_rne(time_s);
    if let Some(mut transform) = world.get_mut::<Transform3>(rig) {
        *transform = lidar_pose_rne(time_s);
    }
    if let Some(mut body) = world.get_mut::<RigidBody>(rig) {
        body.linear_velocity_m_s = linear;
        body.angular_velocity_rad_s = angular;
    }
}

/// A Calibrex `--tf` frames file placing the IMU in the LiDAR frame.
fn frames_yaml(mounting: Mounting) -> String {
    let t = mounting.lever_arm_m;
    let q = mounting.lidar_from_imu;
    format!(
        "schema_version: slac.check_frames/v0.1\nframes:\n- name: {IMU_FRAME}\n  parent: {LIDAR_FRAME}\n  translation_m: [{:.9}, {:.9}, {:.9}]\n  rotation_quat_xyzw: [{:.12}, {:.12}, {:.12}, {:.12}]\n",
        t.x, t.y, t.z, q.x, q.y, q.z, q.w
    )
}

fn turned(mounting: Mounting, degrees: f64) -> Mounting {
    Mounting {
        lidar_from_imu: Quat::from_rotation_z(degrees.to_radians()) * mounting.lidar_from_imu,
        ..mounting
    }
}

struct Args {
    out: PathBuf,
    duration_s: f64,
}

fn parse_args() -> Args {
    let mut args = Args {
        out: PathBuf::from("target/calibrex_rig"),
        duration_s: 40.0,
    };
    let mut iter = std::env::args().skip(1);
    while let Some(flag) = iter.next() {
        let value = iter.next();
        match (flag.as_str(), value) {
            ("--out", Some(value)) => args.out = PathBuf::from(value),
            ("--duration-s", Some(value)) => {
                args.duration_s = value.parse().expect("--duration-s takes seconds");
            }
            _ => {
                eprintln!("usage: 149_calibrex_rig_bag [--out DIR] [--duration-s SECONDS]");
                std::process::exit(2);
            }
        }
    }
    args
}

fn main() -> io::Result<()> {
    let args = parse_args();
    fs::create_dir_all(&args.out)?;
    let mounting = true_mounting();
    let bag = args.out.join("rig.mcap");
    let recording = simulate(&bag, args.duration_s, mounting)?;
    let good = args.out.join("good.yaml");
    let bad = args.out.join("turned.yaml");
    fs::write(&good, frames_yaml(mounting))?;
    fs::write(&bad, frames_yaml(turned(mounting, TURN_DEG)))?;

    println!(
        "wrote {}: {} IMU messages, {} LiDAR scans ({} points), /tf_static lidar -> imu",
        bag.display(),
        recording.imu_messages,
        recording.lidar_messages,
        recording.lidar_points
    );
    println!(
        "true mounting: intrinsic x-y-z {MOUNT_RPY_DEG:?} deg, lever arm [{:.3}, {:.3}, {:.3}] m",
        mounting.lever_arm_m.x, mounting.lever_arm_m.y, mounting.lever_arm_m.z
    );
    println!(
        "candidates: {} (true), {} (IMU turned {TURN_DEG} deg about the LiDAR z axis)",
        good.display(),
        bad.display()
    );
    println!(
        "next: calibrex check {} --tf {}",
        bag.display(),
        good.display()
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn read_records(bytes: &[u8]) -> Vec<(u8, &[u8])> {
        let body = &bytes[8..bytes.len() - 8];
        let mut offset = 0;
        let mut records = Vec::new();
        while offset < body.len() {
            let length =
                u64::from_le_bytes(body[offset + 1..offset + 9].try_into().unwrap()) as usize;
            records.push((body[offset], &body[offset + 9..offset + 9 + length]));
            offset += 9 + length;
        }
        records
    }

    #[test]
    fn short_bag_holds_every_topic_in_time_order() {
        let dir = std::env::temp_dir().join(format!("rne_calibrex_rig_{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let bag = dir.join("rig.mcap");
        let recording = simulate(&bag, 1.0, true_mounting()).unwrap();
        assert_eq!(recording.imu_messages, 200);
        assert_eq!(recording.lidar_messages, 9);
        // Most rays hit the closed room.
        let rays = 9 * (SEGMENTS_PER_SCAN * COLUMNS_PER_SEGMENT) as usize * usize::from(CHANNELS);
        assert!(recording.lidar_points > rays * 9 / 10, "{recording:?}");

        let bytes = fs::read(&bag).unwrap();
        let records = read_records(&bytes);
        let messages: Vec<&[u8]> = records
            .iter()
            .filter(|(opcode, _)| *opcode == 0x05)
            .map(|(_, content)| *content)
            .collect();
        assert_eq!(messages.len(), 200 + 9 + 1);
        let times: Vec<u64> = messages
            .iter()
            .map(|content| u64::from_le_bytes(content[6..14].try_into().unwrap()))
            .collect();
        assert!(times.windows(2).all(|pair| pair[0] <= pair[1]));
        assert_eq!(times[0], BAG_EPOCH_NS);
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn lidar_points_lie_inside_the_room() {
        let mut world = World::new();
        spawn_room(&mut world);
        let mut backend = RapierBackend::new();
        let physics_world = backend.create_world(PhysicsWorldDesc::default()).unwrap();
        backend.sync_from_ecs(&mut world, physics_world).unwrap();
        let start_s = 2.0;
        let scan = cast_scan(&backend, physics_world, &world, 0, start_s);
        assert!(!scan.points_m.is_empty());
        for (point, offset_s) in scan.points_m.iter().zip(&scan.timestamps_s) {
            assert!((0.0..0.1).contains(offset_s));
            // Back to the z-up world with the pose at the point's own time.
            let (rotation, position) = rig_pose_ros(start_s + offset_s);
            let world_point = rotation * *point + position;
            assert!(world_point.x.abs() < 6.05 && world_point.y.abs() < 5.05);
            assert!(world_point.z > -0.05 && world_point.z < 3.05);
        }
    }

    #[test]
    fn stationary_imu_reports_gravity_in_its_own_frame() {
        let mounting = true_mounting();
        let mut world = World::new();
        let rig = spawn_named(&mut world, "rig");
        let (rotation, position) = rig_pose_ros(0.0);
        let m = ros_to_rne();
        world.entity_mut(rig).insert((
            Transform3::from_translation_rotation(m * position, m * rotation * m.inverse()),
            RigidBody {
                body_type: RigidBodyType::Kinematic,
                ..RigidBody::default()
            },
        ));
        let imu = spawn_named(&mut world, "imu");
        world.entity_mut(imu).insert(ImuMount {
            body_entity: rig,
            body_from_sensor: Transform3::from_translation_rotation(
                m * mounting.lever_arm_m,
                m * mounting.lidar_from_imu,
            ),
        });
        let sample = sample_imu_stateful_diagnostic_with_kinematics(
            &world,
            imu,
            &ImuSpec::default(),
            SensorNoiseKey::new(0, 0, 0, 0),
            SimTime::from_ticks(0),
            &mut ImuState::default(),
            &mut ImuKinematicState::default(),
        )
        .unwrap();
        // At rest the accelerometer reads +g along the world up axis, which in
        // the IMU frame is R_imu_world * z.
        let up_in_imu = (rotation * mounting.lidar_from_imu).inverse() * Vec3::Z;
        let expected = up_in_imu * 9.81;
        let measured = sample.measurement.linear_acceleration_m_s2;
        assert!(
            (measured - expected).length() < 1.0e-9,
            "{measured:?} vs {expected:?}"
        );
    }

    #[test]
    fn turned_candidate_differs_by_the_turn_angle() {
        let good = true_mounting();
        let bad = turned(good, TURN_DEG);
        let angle = good.lidar_from_imu.angle_between(bad.lidar_from_imu);
        assert!((angle.to_degrees() - TURN_DEG).abs() < 1.0e-9);
        assert!(frames_yaml(bad).contains("parent: lidar"));
    }
}
