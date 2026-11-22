//! A multi-sensor rig simulated in RNE, recorded as a ROS 2 bag for Calibrex.
//!
//! A hand-held rig carries a 16-channel spinning LiDAR, a second tilted LiDAR,
//! an IMU and a camera, each mounted with a known rotation and lever arm. It
//! moves through a room built from Rapier colliders:
//!
//! * the LiDARs ray-cast the room column by column while the rig moves, so
//!   scans are motion-distorted and carry a per-point time;
//! * the IMU is sampled from the rig's rigid-body kinematics with RNE's IMU
//!   error model;
//! * the camera is RNE's camera model ([`sample_camera_rgbd_keyed`]) on top of a
//!   CPU [`RenderBackend`] that ray-casts the same room colliders, read from the
//!   ECS world, with a procedural texture so image features can be tracked. It
//!   needs no GPU.
//!
//! The output is what a real robot would record:
//!
//! * `rig.mcap` — a ROS 2 bag (MCAP storage, CDR payloads) with `/imu/data`
//!   (`sensor_msgs/Imu`, 200 Hz), `/lidar/points` and `/lidar2/points`
//!   (`sensor_msgs/PointCloud2`, 10 Hz, fields `x y z intensity ring time`),
//!   `/camera/image_raw` (`mono8`, 10 Hz) with `/camera/camera_info`, and
//!   `/tf_static` holding the true mountings of `imu`, `lidar2` and `camera` in
//!   the `lidar` frame;
//! * `good.yaml` — the true mountings as a Calibrex `--tf` frames file;
//! * `turned_imu.yaml`, `turned_lidar2.yaml`, `turned_camera.yaml` — the same
//!   mountings with one sensor turned 3°, calibrations whose pairs with that
//!   sensor should fail a check (`turned.yaml` is `turned_imu.yaml`).
//!
//! ```text
//! cargo run --release -p calibrex_rig_bag --example 149_calibrex_rig_bag -- --out target/calibrex_rig
//! calibrex check target/calibrex_rig/rig.mcap --tf target/calibrex_rig/good.yaml
//! calibrex check target/calibrex_rig/rig.mcap --tf target/calibrex_rig/turned_camera.yaml
//! ```
//!
//! Bag frames follow ROS conventions (x forward, z up; the camera frame is the
//! optical frame, z forward and y down) although the RNE world is y-up and RNE
//! cameras look down their local -z: poses are converted once, at each sensor.

use std::f64::consts::{FRAC_PI_2, PI};
use std::fs::{self, File};
use std::io::{self, BufWriter};
use std::path::{Path, PathBuf};

use rne_adapter_ros2::{
    camera_info, encode_camera_info, encode_image, encode_imu, encode_pointcloud2,
    encode_tf_message, pinhole_camera_info, to_ros_imu, to_ros_mono8_image, to_ros_pointcloud2,
    to_ros_transform_stamped, McapWriter, RosTfMessage, CAMERA_INFO_SCHEMA, IMAGE_SCHEMA,
    IMU_SCHEMA, POINTCLOUD2_SCHEMA, TF_MESSAGE_SCHEMA,
};
use rne_core::{mix64, SimTime};
use rne_data::PointCloud;
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Quat, Vec3};
use rne_physics::{
    Collider, ColliderShape, PhysicsBackend, PhysicsWorldDesc, PhysicsWorldId, RigidBody,
    RigidBodyType,
};
use rne_physics_rapier::RapierBackend;
use rne_render::{
    Camera, CameraPassOutput, DepthFrame, ImageFrame, RenderBackend, RenderError, RenderScene,
    RenderTarget,
};
use rne_sensor::{
    sample_camera_rgbd_keyed, sample_imu_stateful_diagnostic_with_kinematics, sample_lidar_swept,
    CameraDistortion, CameraSpec, ImuAxisErrors, ImuKinematicState, ImuMount, ImuSpec, ImuState,
    LidarSpec, LidarSweep, SensorNoiseKey,
};
use rne_world::Transform3;

const IMU_TOPIC: &str = "/imu/data";
const LIDAR_TOPIC: &str = "/lidar/points";
const LIDAR2_TOPIC: &str = "/lidar2/points";
const IMAGE_TOPIC: &str = "/camera/image_raw";
const CAMERA_INFO_TOPIC: &str = "/camera/camera_info";
const IMAGE2_TOPIC: &str = "/camera2/image_raw";
const CAMERA2_INFO_TOPIC: &str = "/camera2/camera_info";
const LIDAR_FRAME: &str = "lidar";
const LIDAR2_FRAME: &str = "lidar2";
const IMU_FRAME: &str = "imu";
const CAMERA_FRAME: &str = "camera";
const CAMERA2_FRAME: &str = "camera2";
/// Bag clock at simulation time zero: an ordinary wall-clock epoch, as a robot records.
const BAG_EPOCH_NS: u64 = 1_700_000_000_000_000_000;
const IMU_RATE_HZ: u64 = 200;
const LIDAR_RATE_HZ: u64 = 10;
const CAMERA_RATE_HZ: u64 = 10;
const CAMERA_WIDTH: u32 = 480;
const CAMERA_HEIGHT: u32 = 360;
const CAMERA_FOV_Y_DEG: f64 = 70.0;
/// The camera's optical centre in the LiDAR frame, in metres.
const CAMERA_OFFSET_M: [f64; 3] = [0.12, 0.02, -0.06];
/// Where `--motion turn` holds the camera's optical centre, in the z-up world.
const TURN_CENTRE_M: [f64; 3] = [0.0, 0.0, 1.3];
/// Edge length of the procedural texture cells, in meters.
const TEXTURE_CELL_M: f64 = 0.15;
/// Each scan is cast in this many sub-sweeps between exactly sampled rig poses.
const SEGMENTS_PER_SCAN: u32 = 20;
const COLUMNS_PER_SEGMENT: u32 = 18;
const CHANNELS: u16 = 16;
const ROOT_SEED: u64 = 0x5EED_CA1B;
const LIDAR_SENSOR_ID: u64 = 1;
const IMU_SENSOR_ID: u64 = 2;
const LIDAR2_SENSOR_ID: u64 = 3;
const CAMERA_SENSOR_ID: u64 = 4;
const CAMERA2_SENSOR_ID: u64 = 5;
/// `--board` stands the checkerboard this far ahead of [`TURN_CENTRE_M`] along x.
const BOARD_DISTANCE_M: f64 = 1.6;
/// `--motion board` holds each pose this long, then moves to the next in [`BOARD_MOVE_S`].
const BOARD_HOLD_S: f64 = 1.5;
const BOARD_MOVE_S: f64 = 1.0;
/// Each turned candidate turns one sensor this much.
const TURN_DEG: f64 = 3.0;
/// How far the `shifted_imu` candidate moves the IMU, meters.
const SHIFT_M: f64 = 0.05;
/// The true IMU rotation in the LiDAR frame, intrinsic x-y-z angles in degrees.
const MOUNT_RPY_DEG: [f64; 3] = [2.0, -3.0, 92.0];

/// A rigid mounting in the `lidar` frame, in ROS conventions.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Mount {
    /// Rotation taking sensor-frame vectors into the LiDAR frame.
    rotation: Quat,
    /// Sensor origin in the LiDAR frame, meters.
    translation_m: Vec3,
}

impl Mount {
    /// The mount as a transform in RNE's y-up convention, for a sensor whose
    /// own local frame is also converted (a LiDAR's scan frame).
    fn rne_converted(self) -> Transform3 {
        let m = ros_to_rne();
        Transform3::from_translation_rotation(
            m * self.translation_m,
            m * self.rotation * m.inverse(),
        )
    }
}

fn intrinsic_xyz_deg(roll: f64, pitch: f64, yaw: f64) -> Quat {
    Quat::from_rotation_x(roll.to_radians())
        * Quat::from_rotation_y(pitch.to_radians())
        * Quat::from_rotation_z(yaw.to_radians())
}

/// The mountings Calibrex is asked to check.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Rig {
    imu: Mount,
    lidar2: Mount,
    /// The camera's optical frame (x right, y down, z forward).
    camera: Mount,
    /// The second camera (`--sensors camera2`), 10 cm to the right of the first.
    camera2: Mount,
}

fn true_rig() -> Rig {
    // The optical frame looking along the LiDAR's x axis: optical x is LiDAR -y,
    // optical y is LiDAR -z, optical z is LiDAR x.
    let forward_optical = Quat::from_rotation_z(-FRAC_PI_2) * Quat::from_rotation_x(-FRAC_PI_2);
    Rig {
        imu: Mount {
            rotation: intrinsic_xyz_deg(MOUNT_RPY_DEG[0], MOUNT_RPY_DEG[1], MOUNT_RPY_DEG[2]),
            translation_m: Vec3::new(0.10, -0.06, 0.15),
        },
        lidar2: Mount {
            rotation: intrinsic_xyz_deg(8.0, -15.0, 20.0),
            translation_m: Vec3::new(0.30, -0.25, 0.10),
        },
        camera: Mount {
            rotation: forward_optical * intrinsic_xyz_deg(4.0, -2.5, 1.5),
            translation_m: Vec3::new(CAMERA_OFFSET_M[0], CAMERA_OFFSET_M[1], CAMERA_OFFSET_M[2]),
        },
        camera2: Mount {
            rotation: forward_optical * intrinsic_xyz_deg(4.5, -1.5, 0.8),
            translation_m: Vec3::new(
                CAMERA_OFFSET_M[0],
                CAMERA_OFFSET_M[1] - 0.10,
                CAMERA_OFFSET_M[2],
            ),
        },
    }
}

/// A checkerboard by its inner corners and square size (`--board COLSxROWSxSQUARE_M`).
///
/// It stands [`BOARD_DISTANCE_M`] ahead of the turn centre facing the rig, with one
/// white square of margin around its squares, and is a thin collider, so the LiDARs
/// see it and the camera images its pattern.
#[derive(Clone, Copy, Debug, PartialEq)]
struct BoardSpec {
    columns: u32,
    rows: u32,
    square_m: f64,
}

impl BoardSpec {
    fn parse(text: &str) -> Result<Self, String> {
        let parts: Vec<&str> = text.split('x').collect();
        let error = || format!("--board takes COLSxROWSxSQUARE_M (inner corners), not {text:?}");
        if parts.len() != 3 {
            return Err(error());
        }
        let columns: u32 = parts[0].parse().map_err(|_| error())?;
        let rows: u32 = parts[1].parse().map_err(|_| error())?;
        let square_m: f64 = parts[2].parse().map_err(|_| error())?;
        if columns < 2 || rows < 2 || !(square_m.is_finite() && square_m > 0.0) {
            return Err(error());
        }
        Ok(Self {
            columns,
            rows,
            square_m,
        })
    }

    /// Half extents of the board: the squares and a one-square margin, 1 cm thick.
    fn half_extents_m(self) -> Vec3 {
        Vec3::new(
            f64::from(self.columns + 3) * self.square_m * 0.5,
            f64::from(self.rows + 3) * self.square_m * 0.5,
            0.005,
        )
    }

    /// The board's pose in the z-up world: its x and y run along the pattern's columns
    /// and rows, its z away from the rig (the camera's optical axes at the start).
    fn pose_ros() -> (Quat, Vec3) {
        let rotation = Quat::from_rotation_z(-FRAC_PI_2) * Quat::from_rotation_x(-FRAC_PI_2);
        let centre = Vec3::new(
            TURN_CENTRE_M[0] + BOARD_DISTANCE_M,
            TURN_CENTRE_M[1],
            TURN_CENTRE_M[2],
        );
        (rotation, centre)
    }

    /// The pattern's gray level at board coordinates `(a, b)` (metres from its centre).
    fn gray(self, a: f64, b: f64) -> f64 {
        let half = self.half_extents_m();
        let column = ((a + half.x) / self.square_m).floor() as i64;
        let row = ((b + half.y) / self.square_m).floor() as i64;
        let inside = (1..=i64::from(self.columns) + 1).contains(&column)
            && (1..=i64::from(self.rows) + 1).contains(&row);
        if inside && (column + row) % 2 == 0 {
            0.08
        } else {
            0.9
        }
    }
}

/// The camera poses `--motion board` holds in front of the board: the board in each
/// cell of a 5 x 4 grid over the image, then tilted about each of its axes, then nearer
/// and farther. Returned as optical frames in the z-up world.
fn board_tour_camera_poses() -> Vec<(Quat, Vec3)> {
    let (board_rotation, centre) = BoardSpec::pose_ros();
    let towards_rig = board_rotation * -Vec3::Z;
    let mut poses = Vec::new();
    // Facing the board, moved parallel to it so its centre images at normalized
    // (ox, oy): the pattern's corners reach into every cell of a 5 x 4 grid.
    for oy in [-0.33, -0.11, 0.11, 0.33] {
        for ox in [-0.47, -0.2, 0.0, 0.2, 0.47] {
            let distance = 1.3;
            let position = centre + towards_rig * distance
                - board_rotation * Vec3::new(ox * distance, oy * distance, 0.0);
            poses.push((board_rotation, position));
        }
    }
    // Then looking at the board from 20-32 deg off its normal, nearer and farther.
    let tilted = [
        (1.2, Vec3::Y, 32.0_f64),
        (1.2, Vec3::Y, -32.0),
        (1.2, Vec3::X, 32.0),
        (1.2, Vec3::X, -32.0),
        (0.95, Vec3::Y, 20.0),
        (0.95, Vec3::X, -20.0),
        (1.9, Vec3::X, 25.0),
        (1.9, Vec3::Y, -25.0),
    ];
    for (distance, axis, degrees) in tilted {
        let tilt = Quat::from_axis_angle(board_rotation * axis, degrees.to_radians());
        let position = centre + tilt * towards_rig * distance;
        // Look at the board centre, optical y as close to down as it goes.
        let forward = (centre - position).normalize();
        let down = Vec3::new(0.0, 0.0, -1.0);
        let y = (down - forward * down.dot(forward)).normalize();
        let x = y.cross(forward);
        let look = Quat::from_mat4(&rne_math::Mat4::from_cols_array(&[
            x.x, x.y, x.z, 0.0, y.x, y.y, y.z, 0.0, forward.x, forward.y, forward.z, 0.0, 0.0, 0.0,
            0.0, 1.0,
        ]));
        poses.push((look, position));
    }
    poses
}

/// Rotation taking ROS-convention (z-up) vectors into RNE's y-up convention.
fn ros_to_rne() -> Quat {
    Quat::from_rotation_x(-FRAC_PI_2)
}

/// Rotation from an RNE camera's render frame (-z forward, y up) to the ROS
/// optical frame (z forward, y down): a half turn about x.
fn optical_from_render() -> Quat {
    Quat::from_rotation_x(PI)
}

/// How the hand-held rig moves: a smooth motion that turns about all three
/// axes, so that every rotation axis of the mountings is observable.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Motion {
    /// Multiplies the rotation's frequencies (not its amplitude, nor the
    /// translation): 2 turns twice as fast, which makes the lever arm of the IMU
    /// observable.
    rotation_speed: f64,
    /// Turn the rig about the camera's optical centre, without translating it
    /// (`--motion turn`): the recording a targetless intrinsics calibration needs.
    in_place: bool,
    /// Hold the camera still at each of [`board_tour_camera_poses`] in turn
    /// (`--motion board`): the recording a checkerboard calibration needs.
    board_tour: bool,
}

impl Motion {
    const HAND_HELD: Self = Self {
        rotation_speed: 1.0,
        in_place: false,
        board_tour: false,
    };
    const TURN_IN_PLACE: Self = Self {
        rotation_speed: 1.0,
        in_place: true,
        board_tour: false,
    };
    const BOARD_TOUR: Self = Self {
        rotation_speed: 1.0,
        in_place: false,
        board_tour: true,
    };

    /// The rig pose holding the camera (mounted at `CAMERA_OFFSET_M` with the true
    /// rotation) at the board tour's pose at `time_s`.
    fn board_tour_rig_pose(time_s: f64) -> (Quat, Vec3) {
        let poses = board_tour_camera_poses();
        let period = BOARD_HOLD_S + BOARD_MOVE_S;
        let step = (time_s.max(0.0) / period).floor();
        let index = step as usize % poses.len();
        let into = time_s.max(0.0) - step * period;
        let (mut rotation, mut position) = poses[index];
        if into > BOARD_HOLD_S {
            let t = ((into - BOARD_HOLD_S) / BOARD_MOVE_S).clamp(0.0, 1.0);
            let smooth = t * t * (3.0 - 2.0 * t);
            let (next_rotation, next_position) = poses[(index + 1) % poses.len()];
            rotation = rotation.slerp(next_rotation, smooth);
            position = position.lerp(next_position, smooth);
        }
        let camera = true_rig().camera;
        let rig_rotation = rotation * camera.rotation.inverse();
        (rig_rotation, position - rig_rotation * camera.translation_m)
    }

    /// The rig (LiDAR frame) pose in a z-up world.
    fn rig_pose_ros(self, time_s: f64) -> (Quat, Vec3) {
        if self.board_tour {
            return Self::board_tour_rig_pose(time_s);
        }
        let turn_s = self.rotation_speed * time_s;
        let rotation_vector = Vec3::new(
            0.5 * (1.3 * turn_s).sin(),
            0.45 * (1.1 * turn_s + 0.4).sin(),
            1.0 * (0.6 * turn_s).sin(),
        );
        let rotation = Quat::from_scaled_axis(rotation_vector);
        if self.in_place {
            let centre = Vec3::new(TURN_CENTRE_M[0], TURN_CENTRE_M[1], TURN_CENTRE_M[2]);
            let offset = Vec3::new(CAMERA_OFFSET_M[0], CAMERA_OFFSET_M[1], CAMERA_OFFSET_M[2]);
            return (rotation, centre - rotation * offset);
        }
        let position = Vec3::new(
            0.6 * (0.5 * time_s).sin() + 0.04 * time_s,
            0.5 * (0.35 * time_s + 0.2).sin(),
            1.3 + 0.15 * (0.8 * time_s).sin(),
        );
        (rotation, position)
    }

    /// World pose of the RNE LiDAR sensor frame (y-up scan frame) at a time.
    fn lidar_pose_rne(self, time_s: f64) -> Transform3 {
        let (rotation, position) = self.rig_pose_ros(time_s);
        let m = ros_to_rne();
        Transform3::from_translation_rotation(m * position, m * rotation * m.inverse())
    }

    /// World pose of a LiDAR mounted on the rig, in its RNE scan frame.
    fn mounted_lidar_pose_rne(self, time_s: f64, mount: Mount) -> Transform3 {
        self.lidar_pose_rne(time_s)
            .mul_transform(&mount.rne_converted())
    }

    /// World pose of the camera's RNE render frame (-z forward, y up).
    fn camera_render_pose_rne(self, time_s: f64, mount: Mount) -> Transform3 {
        let m = ros_to_rne();
        let lidar_from_render = Transform3::from_translation_rotation(
            m * mount.translation_m,
            m * mount.rotation * optical_from_render(),
        );
        self.lidar_pose_rne(time_s)
            .mul_transform(&lidar_from_render)
    }

    /// World-frame linear and angular velocity of the rig, by central differences.
    fn rig_velocity_rne(self, time_s: f64) -> (Vec3, Vec3) {
        const STEP_S: f64 = 1.0e-5;
        let before = self.lidar_pose_rne(time_s - STEP_S);
        let after = self.lidar_pose_rne(time_s + STEP_S);
        let linear = (after.translation - before.translation) / (2.0 * STEP_S);
        let angular =
            (after.rotation * before.rotation.inverse()).to_scaled_axis() / (2.0 * STEP_S);
        (linear, angular)
    }
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
    let mut boxes = vec![
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
    ];
    // A ring of floor-to-ceiling posts: wherever the camera looks, several stand
    // in front of a wall more than a metre behind them, so each LiDAR beam has
    // depth discontinuities in the image for camera-lidar to align.
    for (index, name) in POST_NAMES.iter().enumerate() {
        let k = index as f64;
        let azimuth = (k * 360.0 / POST_NAMES.len() as f64 + 7.0 * (1.7 * k).sin()).to_radians();
        let radius = [2.0, 2.7, 3.4, 4.1][index % 4];
        let half = 0.07 + 0.03 * (index % 3) as f64;
        let x = radius * azimuth.cos();
        let mut y = radius * azimuth.sin();
        // Keep the rig's path (|y| < 0.5 m, drifting toward +x) clear.
        if y.abs() < 1.8 && x > -1.5 {
            y = 1.8_f64.copysign(y);
        }
        // Two in three posts lean sideways (about the line to the room centre),
        // so their silhouettes are slanted: a vertical shift of the image moves a
        // slanted edge sideways, which vertical edges alone would not show.
        let lean = [0.0_f64, 35.0, -35.0][index % 3];
        boxes.push(boxed(
            name,
            [x, y, 1.5],
            [half, half, 1.5 / lean.to_radians().cos() + 0.1],
            Quat::from_rotation_z(y.atan2(x))
                * Quat::from_rotation_x(lean.to_radians())
                * Quat::from_rotation_z((25.0 * k).to_radians()),
        ));
    }
    boxes
}

/// The posts of [`room_boxes`], 10 degrees apart on average.
const POST_NAMES: [&str; 36] = [
    "post_00", "post_01", "post_02", "post_03", "post_04", "post_05", "post_06", "post_07",
    "post_08", "post_09", "post_10", "post_11", "post_12", "post_13", "post_14", "post_15",
    "post_16", "post_17", "post_18", "post_19", "post_20", "post_21", "post_22", "post_23",
    "post_24", "post_25", "post_26", "post_27", "post_28", "post_29", "post_30", "post_31",
    "post_32", "post_33", "post_34", "post_35",
];

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

/// Stands `board` in the room as a thin fixed collider; returns its entity.
fn spawn_board(world: &mut World, board: BoardSpec) -> Entity {
    let m = ros_to_rne();
    let (rotation, centre) = BoardSpec::pose_ros();
    let entity = spawn_named(world, "board");
    world.entity_mut(entity).insert((
        RigidBody {
            body_type: RigidBodyType::Fixed,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: board.half_extents_m(),
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(m * centre, m * rotation),
    ));
    entity
}

/// A CPU [`RenderBackend`] that ray-casts the cuboid colliders of an ECS world.
///
/// Every face has its own gray level, varied by a deterministic texture of
/// square cells and shaded by the angle of incidence: geometric edges are the
/// strongest image edges, to align with LiDAR depth edges, and the cells give
/// corners to track. It ignores the
/// [`RenderScene`] it is given: the colliders are the scene.
#[derive(Debug)]
struct ColliderRenderer {
    /// `(box_from_world, half extents, texture key)` of every cuboid collider.
    boxes: Vec<(Transform3, Vec3, u64)>,
    /// The checkerboard's texture key and pattern: its large faces show the squares.
    board: Option<(u64, BoardSpec)>,
    /// Rays per pixel along each axis, averaged (1: one ray through the centre).
    samples: u32,
}

impl ColliderRenderer {
    /// Reads every cuboid collider with a pose from `world`.
    fn from_world(world: &mut World) -> Self {
        let mut query = world.query::<(Entity, &Collider, &Transform3)>();
        let mut boxes: Vec<(Transform3, Vec3, u64)> = query
            .iter(world)
            .filter_map(|(entity, collider, transform)| match collider.shape {
                ColliderShape::Cuboid { half_extents_m } => Some((
                    transform_inverse(transform),
                    half_extents_m,
                    entity.to_bits(),
                )),
                _ => None,
            })
            .collect();
        boxes.sort_by_key(|entry| entry.2);
        Self {
            boxes,
            board: None,
            samples: 1,
        }
    }

    /// Draws `board`'s pattern on the collider of `entity` and renders `samples` x
    /// `samples` rays per pixel, so the squares' edges are antialiased.
    fn with_board(mut self, entity: Entity, board: BoardSpec, samples: u32) -> Self {
        self.board = Some((entity.to_bits(), board));
        self.samples = samples.max(1);
        self
    }

    /// The texture key of the nearest box along `origin + t * direction`.
    fn hit_key(&self, origin: Vec3, direction: Vec3) -> Option<u64> {
        let mut best: Option<(f64, u64)> = None;
        for (box_from_world, half, key) in &self.boxes {
            let local_origin = box_from_world.rotation * origin + box_from_world.translation;
            let local_direction = box_from_world.rotation * direction;
            if let Some((t, _)) = slab_hit(local_origin, local_direction, *half) {
                if best.is_none_or(|(nearest, _)| t < nearest) {
                    best = Some((t, *key));
                }
            }
        }
        best.map(|(_, key)| key)
    }

    /// Nearest hit along `origin + t * direction`: `(t, gray level)`.
    fn cast(&self, origin: Vec3, direction: Vec3) -> Option<(f64, u8)> {
        let mut best: Option<(f64, u8)> = None;
        for (box_from_world, half, key) in &self.boxes {
            let local_origin = box_from_world.rotation * origin + box_from_world.translation;
            let local_direction = box_from_world.rotation * direction;
            let Some((t, normal_axis)) = slab_hit(local_origin, local_direction, *half) else {
                continue;
            };
            if best.is_some_and(|(nearest, _)| t >= nearest) {
                continue;
            }
            let hit = local_origin + local_direction * t;
            let (a, b) = match normal_axis {
                0 => (hit.y, hit.z),
                1 => (hit.x, hit.z),
                _ => (hit.x, hit.y),
            };
            // Each face has its own base brightness, so geometric edges are the
            // strongest image edges; low-contrast cells on it give corners to track.
            let face = (normal_axis as u64) * 2 + u64::from(hit[normal_axis] > 0.0);
            let base = 0.25
                + 0.6 * ((mix64(key.wrapping_mul(31).wrapping_add(face)) % 1000) as f64 / 999.0);
            let cell = mix64(
                key.wrapping_mul(7)
                    .wrapping_add(face)
                    .wrapping_add(((a / TEXTURE_CELL_M).floor() as i64 as u64).wrapping_mul(0x9E37))
                    .wrapping_add(
                        ((b / TEXTURE_CELL_M).floor() as i64 as u64).wrapping_mul(0x85EB_CA6B),
                    ),
            );
            let mut albedo = base * (0.9 + 0.2 * ((cell % 1000) as f64 / 999.0));
            if let Some((board_key, board)) = self.board {
                if board_key == *key && normal_axis == 2 {
                    albedo = board.gray(hit.x, hit.y);
                }
            }
            let cosine = local_direction.normalize()[normal_axis].abs();
            let gray = (255.0 * albedo * (0.45 + 0.55 * cosine))
                .round()
                .clamp(0.0, 255.0) as u8;
            best = Some((t, gray));
        }
        best
    }
}

fn transform_inverse(transform: &Transform3) -> Transform3 {
    let rotation = transform.rotation.inverse();
    Transform3::from_translation_rotation(-(rotation * transform.translation), rotation)
}

/// Entry distance and face axis of a ray hitting an axis-aligned box centered
/// at the origin, for rays starting outside the box.
fn slab_hit(origin: Vec3, direction: Vec3, half: Vec3) -> Option<(f64, usize)> {
    let mut near = f64::NEG_INFINITY;
    let mut far = f64::INFINITY;
    let mut axis = 0;
    for i in 0..3 {
        if direction[i].abs() < 1e-12 {
            if origin[i].abs() > half[i] {
                return None;
            }
            continue;
        }
        let t0 = (-half[i] - origin[i]) / direction[i];
        let t1 = (half[i] - origin[i]) / direction[i];
        let (entry, exit) = if t0 < t1 { (t0, t1) } else { (t1, t0) };
        if entry > near {
            near = entry;
            axis = i;
        }
        far = far.min(exit);
    }
    (near <= far && near > 0.0).then_some((near, axis))
}

impl RenderBackend for ColliderRenderer {
    fn render_clear(
        &mut self,
        target: RenderTarget,
        clear_color: [f32; 4],
    ) -> Result<ImageFrame, RenderError> {
        let pixel = clear_color.map(|channel| (channel.clamp(0.0, 1.0) * 255.0).round() as u8);
        let count = (target.width * target.height) as usize;
        Ok(ImageFrame {
            width: target.width,
            height: target.height,
            rgba8: pixel.repeat(count),
        })
    }

    fn render_scene_camera(
        &mut self,
        camera: &Camera,
        view: &rne_math::Transform3,
        _scene: &RenderScene,
        clear_color: [f32; 4],
    ) -> Result<CameraPassOutput, RenderError> {
        let mut color = self.render_clear(camera.render_target(), clear_color)?;
        let mut depth = vec![camera.far_m as f32; (camera.width * camera.height) as usize];
        let focal = (f64::from(camera.height) / 2.0) / (camera.fov_y_rad / 2.0).tan();
        let samples = self.samples;
        for row in 0..camera.height {
            for column in 0..camera.width {
                // A ray through the pixel center with unit forward (-z) component,
                // so the hit distance is the linear depth.
                let ray = |dx: f64, dy: f64| {
                    Vec3::new(
                        (f64::from(column) + dx - f64::from(camera.width) / 2.0) / focal,
                        -(f64::from(row) + dy - f64::from(camera.height) / 2.0) / focal,
                        -1.0,
                    )
                };
                let Some((t, gray)) = self.cast(view.translation, view.rotation * ray(0.5, 0.5))
                else {
                    continue;
                };
                if t < camera.near_m || t > camera.far_m {
                    continue;
                }
                // Only the board needs antialiasing: its squares' corners are what a
                // checkerboard calibration measures; the room keeps one ray per pixel.
                let on_board = self.board.is_some_and(|(board_key, _)| {
                    self.hit_key(view.translation, view.rotation * ray(0.5, 0.5)) == Some(board_key)
                });
                let gray = if samples > 1 && on_board {
                    // Average a samples x samples grid of rays: the depth stays the
                    // centre ray's, so silhouettes keep sharp depth edges.
                    let mut sum = 0.0;
                    for sy in 0..samples {
                        for sx in 0..samples {
                            // Jittered within each sub-cell: a regular grid would put
                            // an edge parallel to the rows or columns (a board facing
                            // the camera) at one of `samples` levels, biasing its
                            // corners; jittered samples average to the true coverage.
                            let jitter = mix64(
                                u64::from(row) << 32
                                    | u64::from(column) << 8
                                    | u64::from(sy * samples + sx),
                            );
                            let jx = (jitter & 0xFFFF) as f64 / 65536.0;
                            let jy = ((jitter >> 16) & 0xFFFF) as f64 / 65536.0;
                            let dx = (f64::from(sx) + jx) / f64::from(samples);
                            let dy = (f64::from(sy) + jy) / f64::from(samples);
                            sum += self
                                .cast(view.translation, view.rotation * ray(dx, dy))
                                .map_or(f64::from(gray), |(_, g)| f64::from(g));
                        }
                    }
                    (sum / f64::from(samples * samples)).round() as u8
                } else {
                    gray
                };
                let index = (row * camera.width + column) as usize;
                depth[index] = t as f32;
                color.rgba8[4 * index..4 * index + 4].copy_from_slice(&[gray, gray, gray, 255]);
            }
        }
        Ok(CameraPassOutput {
            color,
            depth: DepthFrame::new(camera.width, camera.height, depth),
        })
    }
}

#[cfg(test)]
fn camera_spec() -> CameraSpec {
    camera_spec_with(CameraDistortion::default())
}

fn camera_spec_with(lens: CameraDistortion) -> CameraSpec {
    CameraSpec {
        width: CAMERA_WIDTH,
        height: CAMERA_HEIGHT,
        fov_y_rad: CAMERA_FOV_Y_DEG.to_radians(),
        seed: 13,
        read_noise_stddev: 0.01,
        distortion: lens,
        ..CameraSpec::default()
    }
}

/// What `/camera/camera_info` says (`--camera-info`).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
enum CameraInfoMode {
    /// The true intrinsics and distortion.
    #[default]
    True,
    /// An uncalibrated camera: `K` and `D` all zero, as a driver publishes before
    /// calibration.
    Zero,
    /// No `CameraInfo` topic at all.
    None,
}

/// The camera's lens and what its `CameraInfo` says.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct CameraOptions {
    lens: CameraDistortion,
    info: CameraInfoMode,
}

impl CameraOptions {
    /// `--lens K1,K2,P1,P2[,K3]` (OpenCV / ROS `plumb_bob` order).
    fn parse_lens(text: &str) -> Result<CameraDistortion, String> {
        let values = text
            .split(',')
            .map(|value| value.trim().parse::<f64>())
            .collect::<Result<Vec<_>, _>>()
            .map_err(|error| format!("--lens: {error}"))?;
        if !(values.len() == 4 || values.len() == 5) || values.iter().any(|v| !v.is_finite()) {
            return Err("--lens takes K1,K2,P1,P2[,K3]".to_string());
        }
        Ok(CameraDistortion {
            k1: values[0],
            k2: values[1],
            p1: values[2],
            p2: values[3],
            k3: values.get(4).copied().unwrap_or(0.0),
        })
    }

    fn parse_info(text: &str) -> Result<CameraInfoMode, String> {
        match text {
            "true" => Ok(CameraInfoMode::True),
            "zero" => Ok(CameraInfoMode::Zero),
            "none" => Ok(CameraInfoMode::None),
            other => Err(format!(
                "--camera-info takes true, zero or none, not {other:?}"
            )),
        }
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

/// Which sensors a bag records; the primary LiDAR is always recorded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Sensors {
    imu: bool,
    lidar2: bool,
    camera: bool,
    camera2: bool,
}

impl Sensors {
    const ALL: Self = Self {
        imu: true,
        lidar2: true,
        camera: true,
        camera2: false,
    };

    fn parse(list: &str) -> Result<Self, String> {
        let mut sensors = Self {
            imu: false,
            lidar2: false,
            camera: false,
            camera2: false,
        };
        for name in list.split(',').map(str::trim) {
            match name {
                "lidar" => {}
                "imu" => sensors.imu = true,
                "lidar2" => sensors.lidar2 = true,
                "camera" => sensors.camera = true,
                "camera2" => sensors.camera2 = true,
                other => return Err(format!("unknown sensor {other:?}")),
            }
        }
        Ok(sensors)
    }
}

/// Bag topics, in channel order.
#[derive(Clone, Copy, Debug)]
enum Topic {
    Imu,
    Lidar,
    Lidar2,
    Image,
    CameraInfo,
    TfStatic,
    Image2,
    CameraInfo2,
}

/// One recorded message, before it is sorted into the bag by log time.
struct Event {
    log_time_ns: u64,
    topic: Topic,
    payload: Vec<u8>,
}

/// What [`simulate`] recorded.
#[derive(Debug, Default)]
struct Recording {
    imu_messages: usize,
    lidar_messages: usize,
    lidar_points: usize,
    lidar2_messages: usize,
    lidar2_points: usize,
    camera_frames: usize,
    camera2_frames: usize,
}

/// Casts one full scan of a LiDAR mounted at `mount` starting at `start_s`,
/// returned in that LiDAR's ROS frame with per-point times after the scan start.
fn cast_scan(
    backend: &RapierBackend,
    physics_world: PhysicsWorldId,
    world: &World,
    motion: Motion,
    mount: Mount,
    (sensor_id, scan_index): (u64, u64),
    start_s: f64,
) -> PointCloud {
    let to_ros = ros_to_rne().inverse();
    let mut scan = PointCloud::new();
    for segment in 0..SEGMENTS_PER_SCAN {
        let segment_start_s = start_s + f64::from(segment) * segment_duration_s();
        let sweep = LidarSweep::new(
            motion.mounted_lidar_pose_rne(segment_start_s, mount),
            motion.mounted_lidar_pose_rne(segment_start_s + segment_duration_s(), mount),
        );
        let key = SensorNoiseKey::new(
            ROOT_SEED,
            0,
            sensor_id,
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
            let local = motion
                .mounted_lidar_pose_rne(point_time_s, mount)
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

/// The simulated scene and the rig moving through it.
struct Scene {
    world: World,
    backend: RapierBackend,
    physics_world: PhysicsWorldId,
    renderer: ColliderRenderer,
    rig_body: Entity,
    imu: Entity,
    motion: Motion,
}

impl Scene {
    fn new(rig: Rig, motion: Motion, board: Option<BoardSpec>) -> io::Result<Self> {
        let mut world = World::new();
        spawn_room(&mut world);
        let board_entity = board.map(|board| (spawn_board(&mut world, board), board));
        let mut backend = RapierBackend::new();
        let physics_world = backend
            .create_world(PhysicsWorldDesc::default())
            .map_err(io::Error::other)?;
        // The room is static: one sync builds the colliders and the ray-cast index.
        backend
            .sync_from_ecs(&mut world, physics_world)
            .map_err(io::Error::other)?;
        let mut renderer = ColliderRenderer::from_world(&mut world);
        if let Some((entity, board)) = board_entity {
            renderer = renderer.with_board(entity, board, 3);
        }

        // The rig body is the LiDAR frame; it carries no collider so rays never hit it.
        let rig_body = spawn_named(&mut world, "rig");
        world.entity_mut(rig_body).insert((
            motion.lidar_pose_rne(0.0),
            RigidBody {
                body_type: RigidBodyType::Kinematic,
                ..RigidBody::default()
            },
        ));
        let m = ros_to_rne();
        let imu = spawn_named(&mut world, "imu");
        world.entity_mut(imu).insert(ImuMount {
            body_entity: rig_body,
            body_from_sensor: Transform3::from_translation_rotation(
                m * rig.imu.translation_m,
                m * rig.imu.rotation,
            ),
        });
        Ok(Self {
            world,
            backend,
            physics_world,
            renderer,
            rig_body,
            imu,
            motion,
        })
    }

    /// Samples the IMU at its rate from the rig's rigid-body kinematics.
    fn record_imu(
        &mut self,
        duration_s: f64,
        events: &mut Vec<Event>,
        recording: &mut Recording,
    ) -> io::Result<()> {
        let spec = imu_spec();
        let mut imu_state = ImuState::default();
        let mut kinematic_state = ImuKinematicState::default();
        let imu_samples = (duration_s * IMU_RATE_HZ as f64).floor() as u64;
        for index in 0..imu_samples {
            let time_s = index as f64 / IMU_RATE_HZ as f64;
            set_rig_state(&mut self.world, self.rig_body, self.motion, time_s);
            let sample = sample_imu_stateful_diagnostic_with_kinematics(
                &self.world,
                self.imu,
                &spec,
                SensorNoiseKey::new(ROOT_SEED, 0, IMU_SENSOR_ID, index),
                SimTime::from_ticks(seconds_to_ns(time_s)),
                &mut imu_state,
                &mut kinematic_state,
            )
            .map_err(io::Error::other)?;
            let stamp = SimTime::from_ticks(BAG_EPOCH_NS + seconds_to_ns(time_s));
            events.push(Event {
                log_time_ns: stamp.ticks(),
                topic: Topic::Imu,
                payload: encode_imu(&to_ros_imu(&sample.measurement, stamp, IMU_FRAME)),
            });
            recording.imu_messages += 1;
        }
        Ok(())
    }

    /// Casts every scan of one mounted LiDAR.
    fn record_lidar(
        &self,
        duration_s: f64,
        (mount, sensor_id, frame, topic): (Mount, u64, &str, Topic),
        events: &mut Vec<Event>,
        recording: &mut Recording,
    ) {
        let scans = (duration_s * LIDAR_RATE_HZ as f64).floor() as u64;
        for scan_index in 0..scans.saturating_sub(1) {
            let start_s = scan_index as f64 / LIDAR_RATE_HZ as f64;
            let scan = cast_scan(
                &self.backend,
                self.physics_world,
                &self.world,
                self.motion,
                mount,
                (sensor_id, scan_index),
                start_s,
            );
            let stamp = SimTime::from_ticks(BAG_EPOCH_NS + seconds_to_ns(start_s));
            if matches!(topic, Topic::Lidar) {
                recording.lidar_messages += 1;
                recording.lidar_points += scan.points_m.len();
            } else {
                recording.lidar2_messages += 1;
                recording.lidar2_points += scan.points_m.len();
            }
            events.push(Event {
                log_time_ns: stamp.ticks(),
                topic,
                payload: encode_pointcloud2(&to_ros_pointcloud2(&scan, stamp, frame)),
            });
        }
    }

    /// Renders every camera frame with RNE's camera model, plus its `CameraInfo`.
    fn record_camera(
        &mut self,
        duration_s: f64,
        (mount, options): (Mount, CameraOptions),
        (image_topic, info_topic, frame, sensor_id): (Topic, Topic, &str, u64),
        events: &mut Vec<Event>,
    ) -> usize {
        let spec = camera_spec_with(options.lens);
        let frames = (duration_s * CAMERA_RATE_HZ as f64).floor() as u64;
        for frame_index in 0..frames {
            let time_s = frame_index as f64 / CAMERA_RATE_HZ as f64;
            let sample = sample_camera_rgbd_keyed(
                &mut self.renderer,
                &self.motion.camera_render_pose_rne(time_s, mount),
                &spec,
                SimTime::from_ticks(seconds_to_ns(time_s)),
                &RenderScene::new(),
                SensorNoiseKey::new(ROOT_SEED, 0, sensor_id, frame_index),
            );
            let stamp = SimTime::from_ticks(BAG_EPOCH_NS + seconds_to_ns(time_s));
            events.push(Event {
                log_time_ns: stamp.ticks(),
                topic: image_topic,
                payload: encode_image(&to_ros_mono8_image(&sample.rgb, stamp, frame)),
            });
            let info = match options.info {
                CameraInfoMode::True => Some(camera_info(
                    spec.width,
                    spec.height,
                    spec.fov_y_rad,
                    &spec.distortion,
                    stamp,
                    frame,
                )),
                CameraInfoMode::Zero => {
                    let mut info =
                        pinhole_camera_info(spec.width, spec.height, spec.fov_y_rad, stamp, frame);
                    info.k = [0.0; 9];
                    info.p = [0.0; 12];
                    Some(info)
                }
                CameraInfoMode::None => None,
            };
            if let Some(info) = info {
                events.push(Event {
                    log_time_ns: stamp.ticks(),
                    topic: info_topic,
                    payload: encode_camera_info(&info),
                });
            }
        }
        frames as usize
    }
}

/// The `/tf_static` message: every recorded sensor's true mounting in `lidar`.
fn tf_static_event(rig: Rig, sensors: Sensors) -> Event {
    let epoch = SimTime::from_ticks(BAG_EPOCH_NS);
    let transforms = [
        (sensors.imu, IMU_FRAME, rig.imu),
        (sensors.lidar2, LIDAR2_FRAME, rig.lidar2),
        (sensors.camera, CAMERA_FRAME, rig.camera),
        (sensors.camera2, CAMERA2_FRAME, rig.camera2),
    ]
    .into_iter()
    .filter(|(enabled, _, _)| *enabled)
    .map(|(_, frame, mount)| {
        to_ros_transform_stamped(
            LIDAR_FRAME,
            frame,
            Transform3::from_translation_rotation(mount.translation_m, mount.rotation),
            epoch,
        )
    })
    .collect();
    Event {
        log_time_ns: BAG_EPOCH_NS,
        topic: Topic::TfStatic,
        payload: encode_tf_message(&RosTfMessage { transforms }),
    }
}

/// Writes the events, in log-time order, as an MCAP bag.
fn write_bag(bag_path: &Path, mut events: Vec<Event>) -> io::Result<()> {
    // A recorder writes in arrival order; the sort is stable, so equal stamps
    // keep the order they were produced in.
    events.sort_by_key(|event| event.log_time_ns);
    let mut writer = McapWriter::new(BufWriter::new(File::create(bag_path)?))?;
    // Declare a channel only for a topic that has messages, as a recorder does,
    // so a bag of some sensors names no topic of the others.
    let declared = [
        (Topic::Imu, IMU_TOPIC, IMU_SCHEMA, false),
        (Topic::Lidar, LIDAR_TOPIC, POINTCLOUD2_SCHEMA, false),
        (Topic::Lidar2, LIDAR2_TOPIC, POINTCLOUD2_SCHEMA, false),
        (Topic::Image, IMAGE_TOPIC, IMAGE_SCHEMA, false),
        (
            Topic::CameraInfo,
            CAMERA_INFO_TOPIC,
            CAMERA_INFO_SCHEMA,
            false,
        ),
        (Topic::TfStatic, "/tf_static", TF_MESSAGE_SCHEMA, true),
        (Topic::Image2, IMAGE2_TOPIC, IMAGE_SCHEMA, false),
        (
            Topic::CameraInfo2,
            CAMERA2_INFO_TOPIC,
            CAMERA_INFO_SCHEMA,
            false,
        ),
    ];
    let mut channels = [None; 8];
    for (topic, name, schema, latched) in declared {
        if events
            .iter()
            .any(|event| event.topic as usize == topic as usize)
        {
            channels[topic as usize] = Some(writer.add_channel(name, schema, latched)?);
        }
    }
    for event in &events {
        let channel = channels[event.topic as usize].expect("every event's topic is declared");
        writer.write_message(
            channel,
            event.log_time_ns,
            event.log_time_ns,
            &event.payload,
        )?;
    }
    writer.finish()?;
    Ok(())
}

/// Simulates the rig for `duration_s` and writes the bag to `bag_path`.
fn simulate(
    bag_path: &Path,
    duration_s: f64,
    rig: Rig,
    sensors: Sensors,
    motion: Motion,
    camera: CameraOptions,
    board: Option<BoardSpec>,
) -> io::Result<Recording> {
    let mut scene = Scene::new(rig, motion, board)?;
    let mut events = Vec::new();
    let mut recording = Recording::default();
    if sensors.imu {
        scene.record_imu(duration_s, &mut events, &mut recording)?;
    }
    let primary = Mount {
        rotation: Quat::IDENTITY,
        translation_m: Vec3::ZERO,
    };
    scene.record_lidar(
        duration_s,
        (primary, LIDAR_SENSOR_ID, LIDAR_FRAME, Topic::Lidar),
        &mut events,
        &mut recording,
    );
    if sensors.lidar2 {
        scene.record_lidar(
            duration_s,
            (rig.lidar2, LIDAR2_SENSOR_ID, LIDAR2_FRAME, Topic::Lidar2),
            &mut events,
            &mut recording,
        );
    }
    if sensors.camera {
        recording.camera_frames = scene.record_camera(
            duration_s,
            (rig.camera, camera),
            (
                Topic::Image,
                Topic::CameraInfo,
                CAMERA_FRAME,
                CAMERA_SENSOR_ID,
            ),
            &mut events,
        );
    }
    if sensors.camera2 {
        recording.camera2_frames = scene.record_camera(
            duration_s,
            (rig.camera2, camera),
            (
                Topic::Image2,
                Topic::CameraInfo2,
                CAMERA2_FRAME,
                CAMERA2_SENSOR_ID,
            ),
            &mut events,
        );
    }
    events.push(tf_static_event(rig, sensors));
    write_bag(bag_path, events)?;
    Ok(recording)
}

fn set_rig_state(world: &mut World, rig: Entity, motion: Motion, time_s: f64) {
    let (linear, angular) = motion.rig_velocity_rne(time_s);
    if let Some(mut transform) = world.get_mut::<Transform3>(rig) {
        *transform = motion.lidar_pose_rne(time_s);
    }
    if let Some(mut body) = world.get_mut::<RigidBody>(rig) {
        body.linear_velocity_m_s = linear;
        body.angular_velocity_rad_s = angular;
    }
}

/// A Calibrex `--tf` frames file placing the recorded sensors in the LiDAR frame.
fn frames_yaml(rig: Rig, sensors: Sensors) -> String {
    let mut text = String::from("schema_version: slac.check_frames/v0.1\nframes:\n");
    for (enabled, frame, mount) in [
        (sensors.imu, IMU_FRAME, rig.imu),
        (sensors.lidar2, LIDAR2_FRAME, rig.lidar2),
        (sensors.camera, CAMERA_FRAME, rig.camera),
        (sensors.camera2, CAMERA2_FRAME, rig.camera2),
    ] {
        if !enabled {
            continue;
        }
        let t = mount.translation_m;
        let q = mount.rotation;
        text.push_str(&format!(
            "- name: {frame}\n  parent: {LIDAR_FRAME}\n  translation_m: [{:.9}, {:.9}, {:.9}]\n  rotation_quat_xyzw: [{:.12}, {:.12}, {:.12}, {:.12}]\n",
            t.x, t.y, t.z, q.x, q.y, q.z, q.w
        ));
    }
    text
}

/// The IMU turned about the LiDAR z axis.
fn turned_imu(rig: Rig, degrees: f64) -> Rig {
    Rig {
        imu: Mount {
            rotation: Quat::from_rotation_z(degrees.to_radians()) * rig.imu.rotation,
            ..rig.imu
        },
        ..rig
    }
}

/// The IMU moved along the LiDAR x axis: its lever arm is wrong, its rotation is not.
fn shifted_imu(rig: Rig, meters: f64) -> Rig {
    Rig {
        imu: Mount {
            translation_m: rig.imu.translation_m + Vec3::new(meters, 0.0, 0.0),
            ..rig.imu
        },
        ..rig
    }
}

/// The second LiDAR turned about its own z axis.
fn turned_lidar2(rig: Rig, degrees: f64) -> Rig {
    Rig {
        lidar2: Mount {
            rotation: rig.lidar2.rotation * Quat::from_rotation_z(degrees.to_radians()),
            ..rig.lidar2
        },
        ..rig
    }
}

/// The camera turned about its own (optical) x axis, a pitch of the view.
fn turned_camera(rig: Rig, degrees: f64) -> Rig {
    Rig {
        camera: Mount {
            rotation: rig.camera.rotation * Quat::from_rotation_x(degrees.to_radians()),
            ..rig.camera
        },
        ..rig
    }
}

struct Args {
    out: PathBuf,
    duration_s: f64,
    sensors: Sensors,
    motion: Motion,
    camera: CameraOptions,
    board: Option<BoardSpec>,
}

const USAGE: &str = "usage: 149_calibrex_rig_bag [--out DIR] [--duration-s SECONDS] \
     [--sensors lidar,imu,lidar2,camera,camera2] [--rotation-speed FACTOR] \
     [--motion hand-held|turn|board] [--lens K1,K2,P1,P2[,K3]] \
     [--camera-info true|zero|none] [--board COLSxROWSxSQUARE_M]";

fn exit_usage(error: &str) -> ! {
    eprintln!("{error}\n{USAGE}");
    std::process::exit(2);
}

fn parse_args() -> Args {
    let mut args = Args {
        out: PathBuf::from("target/calibrex_rig"),
        duration_s: 40.0,
        sensors: Sensors::ALL,
        motion: Motion::HAND_HELD,
        camera: CameraOptions::default(),
        board: None,
    };
    let mut iter = std::env::args().skip(1);
    while let Some(flag) = iter.next() {
        let value = iter.next();
        match (flag.as_str(), value) {
            ("--out", Some(value)) => args.out = PathBuf::from(value),
            ("--duration-s", Some(value)) => {
                args.duration_s = value.parse().expect("--duration-s takes seconds");
            }
            ("--rotation-speed", Some(value)) => {
                args.motion.rotation_speed = value
                    .parse()
                    .ok()
                    .filter(|speed: &f64| speed.is_finite() && *speed > 0.0)
                    .unwrap_or_else(|| {
                        eprintln!("--rotation-speed takes a positive factor\n{USAGE}");
                        std::process::exit(2);
                    });
            }
            ("--motion", Some(value)) => {
                let speed = args.motion.rotation_speed;
                args.motion = match value.as_str() {
                    "hand-held" => Motion::HAND_HELD,
                    "turn" => Motion::TURN_IN_PLACE,
                    "board" => Motion::BOARD_TOUR,
                    other => exit_usage(&format!("unknown --motion {other:?}")),
                };
                args.motion.rotation_speed = speed;
            }
            ("--lens", Some(value)) => {
                args.camera.lens =
                    CameraOptions::parse_lens(&value).unwrap_or_else(|error| exit_usage(&error));
            }
            ("--board", Some(value)) => {
                args.board =
                    Some(BoardSpec::parse(&value).unwrap_or_else(|error| exit_usage(&error)));
            }
            ("--camera-info", Some(value)) => {
                args.camera.info =
                    CameraOptions::parse_info(&value).unwrap_or_else(|error| exit_usage(&error));
            }
            ("--sensors", Some(value)) => {
                args.sensors = Sensors::parse(&value).unwrap_or_else(|error| {
                    eprintln!("{error}\n{USAGE}");
                    std::process::exit(2);
                });
            }
            _ => {
                eprintln!("{USAGE}");
                std::process::exit(2);
            }
        }
    }
    args
}

fn main() -> io::Result<()> {
    let args = parse_args();
    fs::create_dir_all(&args.out)?;
    let rig = true_rig();
    let bag = args.out.join("rig.mcap");
    let recording = simulate(
        &bag,
        args.duration_s,
        rig,
        args.sensors,
        args.motion,
        args.camera,
        args.board,
    )?;
    if args.motion.board_tour && args.board.is_none() {
        eprintln!("note: --motion board without --board holds the rig in front of nothing");
    }

    let mut candidates = vec![("good.yaml", rig)];
    if args.sensors.imu {
        candidates.push(("turned_imu.yaml", turned_imu(rig, TURN_DEG)));
        candidates.push(("turned.yaml", turned_imu(rig, TURN_DEG)));
        candidates.push(("shifted_imu.yaml", shifted_imu(rig, SHIFT_M)));
    }
    if args.sensors.lidar2 {
        candidates.push(("turned_lidar2.yaml", turned_lidar2(rig, TURN_DEG)));
    }
    if args.sensors.camera {
        candidates.push(("turned_camera.yaml", turned_camera(rig, TURN_DEG)));
    }
    for (name, candidate) in &candidates {
        fs::write(args.out.join(name), frames_yaml(*candidate, args.sensors))?;
    }

    println!("wrote {}: {recording:?}", bag.display());
    println!(
        "candidates in {}: {} (each turned_* turns one sensor {TURN_DEG} deg; \
         shifted_imu moves the IMU {SHIFT_M} m)",
        args.out.display(),
        candidates
            .iter()
            .map(|(name, _)| *name)
            .collect::<Vec<_>>()
            .join(", ")
    );
    if let (Some(board), true) = (args.board, args.sensors.camera) {
        let camchain = args.out.join("intrinsics_camchain.yaml");
        let spec = format!("{}x{}x{}", board.columns, board.rows, board.square_m);
        let right = if args.sensors.camera2 {
            format!(" --right-topic {IMAGE2_TOPIC}")
        } else {
            String::new()
        };
        println!(
            "next: calibrate the camera(s) with the board, then check with them:\n  calibrex \
             intrinsics guide --board {spec} --bag {bag} --topic {IMAGE_TOPIC}{right} \
             --no-window --output {camchain}\n  calibrex check {bag} --tf {good} \
             --intrinsics {camchain} --board {spec}",
            bag = bag.display(),
            camchain = camchain.display(),
            good = args.out.join("good.yaml").display(),
        );
    } else if args.sensors.camera && args.camera.info != CameraInfoMode::True {
        let camchain = args.out.join("intrinsics_camchain.yaml");
        println!(
            "next: the camera has no calibrated CameraInfo; calibrate it from the bag, then \
             check with it:\n  calibrex intrinsics rotate --bag {bag} --topic {IMAGE_TOPIC} \
             --output {camchain}\n  calibrex check {bag} --tf {good} --intrinsics {camchain}",
            bag = bag.display(),
            camchain = camchain.display(),
            good = args.out.join("good.yaml").display(),
        );
    } else {
        println!(
            "next: calibrex check {} --tf {}",
            bag.display(),
            args.out.join("good.yaml").display()
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_adapter_ros2::RosCameraInfo;
    use rne_physics::RaycastQuery;

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
    fn the_rig_keeps_clear_of_the_posts() {
        // Every sensor stays outside each post grown by 0.4 m, over two minutes
        // and at the rotation speeds the README uses, so no sensor starts inside
        // an object or sees it closer than its range floor.
        let posts: Vec<RoomBox> = room_boxes()
            .into_iter()
            .filter(|room_box| room_box.name.starts_with("post_"))
            .collect();
        assert_eq!(posts.len(), POST_NAMES.len());
        let rig = true_rig();
        for step in 0..=4800 {
            let motion = Motion {
                rotation_speed: if step % 2 == 0 { 1.0 } else { 2.0 },
                // --motion turn and board too: they hold the rig near the turn centre.
                in_place: step % 3 == 0,
                board_tour: step % 5 == 0,
            };
            let (rotation, position) = motion.rig_pose_ros(f64::from(step / 2) * 0.05);
            for offset in [
                Vec3::ZERO,
                rig.imu.translation_m,
                rig.lidar2.translation_m,
                rig.camera.translation_m,
            ] {
                let sensor = position + rotation * offset;
                for post in &posts {
                    let local = post.rotation.inverse() * (sensor - post.center_m);
                    let clear = (0..3).any(|i| local[i].abs() > post.half_extents_m[i] + 0.4);
                    assert!(clear, "{} at {sensor:?}, {motion:?}", post.name);
                }
            }
        }
    }

    #[test]
    fn rotation_speed_turns_faster_along_the_same_path() {
        let brisk = Motion {
            rotation_speed: 2.0,
            ..Motion::HAND_HELD
        };
        for time_s in [0.0, 0.7, 3.1, 12.5] {
            let (rotation, position) = brisk.rig_pose_ros(time_s);
            let (slow_rotation, _) = Motion::HAND_HELD.rig_pose_ros(2.0 * time_s);
            let (_, slow_position) = Motion::HAND_HELD.rig_pose_ros(time_s);
            assert!(rotation.angle_between(slow_rotation) < 1e-6);
            assert!(position.distance(slow_position) < 1e-12);
        }
    }

    #[test]
    fn short_bag_holds_every_topic_in_time_order() {
        let dir = std::env::temp_dir().join(format!("rne_calibrex_rig_{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let bag = dir.join("rig.mcap");
        let recording = simulate(
            &bag,
            1.0,
            true_rig(),
            Sensors::ALL,
            Motion::HAND_HELD,
            CameraOptions::default(),
            None,
        )
        .unwrap();
        assert_eq!(recording.imu_messages, 200);
        assert_eq!(recording.lidar_messages, 9);
        assert_eq!(recording.lidar2_messages, 9);
        assert_eq!(recording.camera_frames, 10);
        // Most rays of both LiDARs hit the closed room.
        let rays = 9 * (SEGMENTS_PER_SCAN * COLUMNS_PER_SEGMENT) as usize * usize::from(CHANNELS);
        assert!(recording.lidar_points > rays * 9 / 10, "{recording:?}");
        assert!(recording.lidar2_points > rays * 9 / 10, "{recording:?}");

        let bytes = fs::read(&bag).unwrap();
        let records = read_records(&bytes);
        let messages: Vec<&[u8]> = records
            .iter()
            .filter(|(opcode, _)| *opcode == 0x05)
            .map(|(_, content)| *content)
            .collect();
        assert_eq!(messages.len(), 200 + 9 + 9 + 10 + 10 + 1);
        let times: Vec<u64> = messages
            .iter()
            .map(|content| u64::from_le_bytes(content[6..14].try_into().unwrap()))
            .collect();
        assert!(times.windows(2).all(|pair| pair[0] <= pair[1]));
        assert_eq!(times[0], BAG_EPOCH_NS);
        fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn lidar_only_bag_records_only_the_primary_lidar() {
        let dir = std::env::temp_dir().join(format!("rne_calibrex_lidar_{}", std::process::id()));
        fs::create_dir_all(&dir).unwrap();
        let sensors = Sensors::parse("lidar").unwrap();
        let recording = simulate(
            &dir.join("rig.mcap"),
            0.5,
            true_rig(),
            sensors,
            Motion::HAND_HELD,
            CameraOptions::default(),
            None,
        )
        .unwrap();
        assert_eq!(recording.imu_messages, 0);
        assert_eq!(recording.lidar2_messages, 0);
        assert_eq!(recording.camera_frames, 0);
        assert_eq!(recording.lidar_messages, 4);
        assert!(!frames_yaml(true_rig(), sensors).contains("- name:"));
        assert!(Sensors::parse("lidar,radar").is_err());
        // Only the recorded topics are declared.
        let bytes = fs::read(dir.join("rig.mcap")).unwrap();
        let channels: Vec<String> = read_records(&bytes)
            .iter()
            .filter(|(opcode, _)| *opcode == 0x04)
            .map(|(_, content)| String::from_utf8_lossy(content).into_owned())
            .collect();
        assert_eq!(channels.len(), 2, "{channels:?}");
        assert!(channels.iter().any(|text| text.contains(LIDAR_TOPIC)));
        assert!(channels.iter().any(|text| text.contains("/tf_static")));
        fs::remove_dir_all(&dir).unwrap();
    }

    fn room_physics() -> (World, RapierBackend, PhysicsWorldId) {
        let mut world = World::new();
        spawn_room(&mut world);
        let mut backend = RapierBackend::new();
        let physics_world = backend.create_world(PhysicsWorldDesc::default()).unwrap();
        backend.sync_from_ecs(&mut world, physics_world).unwrap();
        (world, backend, physics_world)
    }

    #[test]
    fn lidar_points_of_both_lidars_lie_inside_the_room() {
        let (world, backend, physics_world) = room_physics();
        let start_s = 2.0;
        let primary = Mount {
            rotation: Quat::IDENTITY,
            translation_m: Vec3::ZERO,
        };
        for mount in [primary, true_rig().lidar2] {
            let scan = cast_scan(
                &backend,
                physics_world,
                &world,
                Motion::HAND_HELD,
                mount,
                (9, 0),
                start_s,
            );
            assert!(!scan.points_m.is_empty());
            for (point, offset_s) in scan.points_m.iter().zip(&scan.timestamps_s) {
                assert!((0.0..0.1).contains(offset_s));
                // Back to the z-up world with the pose at the point's own time.
                let (rotation, position) = Motion::HAND_HELD.rig_pose_ros(start_s + offset_s);
                let in_lidar = mount.rotation * *point + mount.translation_m;
                let world_point = rotation * in_lidar + position;
                assert!(world_point.x.abs() < 6.05 && world_point.y.abs() < 5.05);
                assert!(world_point.z > -0.05 && world_point.z < 3.05);
            }
        }
    }

    #[test]
    fn rendered_depth_matches_a_physics_raycast() {
        let (mut world, backend, physics_world) = room_physics();
        let mut renderer = ColliderRenderer::from_world(&mut world);
        assert_eq!(renderer.boxes.len(), room_boxes().len());
        let pose = Motion::HAND_HELD.camera_render_pose_rne(3.0, true_rig().camera);
        let spec = camera_spec();
        let camera = Camera::new(spec.width, spec.height, spec.fov_y_rad);
        let view = rne_math::Transform3 {
            translation: pose.translation,
            rotation: pose.rotation,
            scale: pose.scale,
        };
        let output = renderer
            .render_scene_camera(&camera, &view, &RenderScene::new(), [0.0; 4])
            .unwrap();
        let focal = (f64::from(spec.height) / 2.0) / (spec.fov_y_rad / 2.0).tan();
        for (column, row) in [(17, 23), (240, 180), (400, 300), (470, 5)] {
            let direction = Vec3::new(
                (f64::from(column) + 0.5 - f64::from(spec.width) / 2.0) / focal,
                -(f64::from(row) + 0.5 - f64::from(spec.height) / 2.0) / focal,
                -1.0,
            );
            let world_direction = pose.rotation * direction;
            let hits = backend
                .raycast(
                    physics_world,
                    RaycastQuery {
                        origin_m: pose.translation,
                        direction: world_direction.normalize(),
                        max_distance_m: 100.0,
                    },
                )
                .unwrap();
            let physics_depth = hits[0].distance_m / world_direction.length();
            let rendered = f64::from(output.depth.depth_m[(row * spec.width + column) as usize]);
            assert!(
                (rendered - physics_depth).abs() < 1e-3,
                "pixel ({column}, {row}): rendered {rendered} vs physics {physics_depth}"
            );
        }
        // The texture has contrast to track.
        let grays: std::collections::BTreeSet<u8> = output
            .color
            .rgba8
            .chunks_exact(4)
            .map(|pixel| pixel[0])
            .collect();
        assert!(grays.len() > 50);
    }

    /// With the rig held still, LiDAR points projected through the published
    /// `T_lidar_camera` and `CameraInfo` land where the camera rendered the
    /// same depth: the bag's camera and LiDAR geometry agree.
    #[test]
    fn lidar_points_project_onto_the_rendered_depth() {
        let (mut world, backend, physics_world) = room_physics();
        let mut renderer = ColliderRenderer::from_world(&mut world);
        let rig = true_rig();
        let time_s = 3.0;
        let spec = camera_spec();
        let pose = Motion::HAND_HELD.camera_render_pose_rne(time_s, rig.camera);
        let view = rne_math::Transform3 {
            translation: pose.translation,
            rotation: pose.rotation,
            scale: pose.scale,
        };
        let rendered = renderer
            .render_scene_camera(
                &Camera::new(spec.width, spec.height, spec.fov_y_rad),
                &view,
                &RenderScene::new(),
                [0.0; 4],
            )
            .unwrap();
        let info = pinhole_camera_info(
            spec.width,
            spec.height,
            spec.fov_y_rad,
            SimTime::from_ticks(0),
            "camera",
        );
        assert_lidar_lands_on_depth(
            (&world, &backend, physics_world),
            time_s,
            &rendered.depth.depth_m,
            &spec,
            &info,
        );
    }

    /// The same through a distorted lens: the frame RNE renders and its
    /// `CameraInfo` (`plumb_bob` D, projected as OpenCV does) agree.
    #[test]
    fn a_distorted_camera_info_describes_the_rendered_frame() {
        let (mut world, backend, physics_world) = room_physics();
        let mut renderer = ColliderRenderer::from_world(&mut world);
        let rig = true_rig();
        let time_s = 3.0;
        let lens = CameraOptions::parse_lens("-0.18,0.04,0.0008,-0.0006").unwrap();
        let spec = CameraSpec {
            read_noise_stddev: 0.0,
            ..camera_spec_with(lens)
        };
        // A scene with an item takes the depth-rendering path; the collider renderer
        // draws the room and ignores the item.
        let scene = RenderScene {
            items: vec![RenderScene::item_from_visual(
                Transform3::default(),
                rne_render::VisualShape::Sphere { radius_m: 0.01 },
                [1.0; 4],
                Transform3::default(),
            )],
        };
        let sample = sample_camera_rgbd_keyed(
            &mut renderer,
            &Motion::HAND_HELD.camera_render_pose_rne(time_s, rig.camera),
            &spec,
            SimTime::from_ticks(0),
            &scene,
            SensorNoiseKey::new(0, 0, 0, 0),
        );
        assert_eq!(
            (sample.depth.width, sample.depth.height),
            (spec.width, spec.height)
        );
        let info = camera_info(
            spec.width,
            spec.height,
            spec.fov_y_rad,
            &lens,
            SimTime::from_ticks(0),
            "camera",
        );
        assert_lidar_lands_on_depth(
            (&world, &backend, physics_world),
            time_s,
            &sample.depth.depth_m,
            &spec,
            &info,
        );
    }

    /// Projects the LiDAR's points at `time_s` through the published intrinsics and
    /// distortion (OpenCV's model) and compares them with the rendered depth.
    fn assert_lidar_lands_on_depth(
        (world, backend, physics_world): (&World, &RapierBackend, PhysicsWorldId),
        time_s: f64,
        depth_m: &[f32],
        spec: &CameraSpec,
        info: &RosCameraInfo,
    ) {
        let rig = true_rig();
        let (fx, cx, fy, cy) = (info.k[0], info.k[2], info.k[4], info.k[5]);
        let d = &info.d;
        let to_ros = ros_to_rne().inverse();
        let mut differences = Vec::new();
        for segment in 0..SEGMENTS_PER_SCAN {
            let cloud = sample_lidar_swept(
                backend,
                physics_world,
                world,
                &LidarSweep::stationary(Motion::HAND_HELD.lidar_pose_rne(time_s)),
                &LidarSpec {
                    range_noise_stddev_m: 0.0,
                    ..lidar_segment_spec(segment)
                },
                SensorNoiseKey::new(0, 0, 0, u64::from(segment)),
            );
            for point in &cloud.points_m {
                // World -> LiDAR (ROS) -> camera optical frame, as Calibrex does.
                let lidar = to_ros
                    * Motion::HAND_HELD
                        .lidar_pose_rne(time_s)
                        .to_matrix()
                        .inverse()
                        .transform_point3(*point);
                let optical = rig.camera.rotation.inverse() * (lidar - rig.camera.translation_m);
                if optical.z < 0.5 {
                    continue;
                }
                let (x, y) = (optical.x / optical.z, optical.y / optical.z);
                let r2 = x * x + y * y;
                let radial = 1.0 + d[0] * r2 + d[1] * r2 * r2 + d[4] * r2 * r2 * r2;
                let x_d = x * radial + 2.0 * d[2] * x * y + d[3] * (r2 + 2.0 * x * x);
                let y_d = y * radial + d[2] * (r2 + 2.0 * y * y) + 2.0 * d[3] * x * y;
                let (u, v) = (fx * x_d + cx, fy * y_d + cy);
                let (column, row) = (u.round(), v.round());
                if column < 1.0
                    || row < 1.0
                    || column >= f64::from(spec.width) - 1.0
                    || row >= f64::from(spec.height) - 1.0
                {
                    continue;
                }
                // The nearest rendered depth within a pixel: next to a post's
                // silhouette the rounded pixel may show what is behind it.
                let difference = (-1..=1)
                    .flat_map(|dr| (-1..=1).map(move |dc| (dr, dc)))
                    .map(|(dr, dc)| {
                        let index =
                            (row as i64 + dr) as u32 * spec.width + (column as i64 + dc) as u32;
                        (f64::from(depth_m[index as usize]) - optical.z).abs()
                    })
                    .fold(f64::INFINITY, f64::min);
                differences.push(difference);
            }
        }
        assert!(
            differences.len() > 1000,
            "{} points in view",
            differences.len()
        );
        differences.sort_by(f64::total_cmp);
        let median = differences[differences.len() / 2];
        let close =
            differences.iter().filter(|d| **d < 0.02).count() as f64 / differences.len() as f64;
        assert!(median < 0.005, "median depth difference {median}");
        assert!(close > 0.95, "{close} of points within 2 cm");
    }

    #[test]
    fn turning_in_place_holds_the_camera_centre_and_keeps_clear_of_the_posts() {
        let rig = true_rig();
        let centre = Vec3::new(TURN_CENTRE_M[0], TURN_CENTRE_M[1], TURN_CENTRE_M[2]);
        let mut turned = 0.0_f64;
        for step in 0..400 {
            let time_s = f64::from(step) * 0.1;
            let (rotation, position) = Motion::TURN_IN_PLACE.rig_pose_ros(time_s);
            let camera = position + rotation * rig.camera.translation_m;
            assert!(
                (camera - centre).length() < 1e-9,
                "camera moved at {time_s} s"
            );
            turned = turned.max(rotation.to_scaled_axis().length());
        }
        assert!(turned.to_degrees() > 45.0);
    }

    #[test]
    fn every_board_tour_pose_sees_the_whole_board_unoccluded_and_holds_still() {
        let board = BoardSpec::parse("7x5x0.08").unwrap();
        let mut world = World::new();
        spawn_room(&mut world);
        let entity = spawn_board(&mut world, board);
        let renderer = ColliderRenderer::from_world(&mut world).with_board(entity, board, 1);
        let (board_rotation, centre) = BoardSpec::pose_ros();
        let half = board.half_extents_m();
        let spec = camera_spec();
        let focal = f64::from(spec.height) / 2.0 / (spec.fov_y_rad / 2.0).tan();
        let m = ros_to_rne();
        let poses = board_tour_camera_poses();
        assert!(poses.len() >= 28);
        for (index, (rotation, position)) in poses.iter().enumerate() {
            // The board's outer corners image inside the frame, with a margin.
            for (sx, sy) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
                let corner = centre + board_rotation * Vec3::new(sx * half.x, sy * half.y, 0.0);
                let optical = rotation.inverse() * (corner - position);
                let (u, v) = (focal * optical.x / optical.z, focal * optical.y / optical.z);
                assert!(optical.z > 0.3, "pose {index}: corner behind the camera");
                assert!(
                    u.abs() < f64::from(spec.width) / 2.0 - 4.0
                        && v.abs() < f64::from(spec.height) / 2.0 - 4.0,
                    "pose {index}: corner at ({u:.0}, {v:.0}) px off the centre"
                );
            }
            // Nothing stands between the camera and the board.
            let direction = (centre - *position).normalize();
            let (t, _) = renderer.cast(m * *position, m * direction).unwrap();
            let expected = (centre - *position).dot(board_rotation * -Vec3::Z).abs()
                / direction.dot(board_rotation * -Vec3::Z).abs();
            assert!(
                (t - expected).abs() < 0.02,
                "pose {index}: hit {t:.3} m, board {expected:.3} m"
            );
            // The rig holds still while it shows the pose.
            let start = index as f64 * (BOARD_HOLD_S + BOARD_MOVE_S);
            let (r0, p0) = Motion::BOARD_TOUR.rig_pose_ros(start + 0.01);
            let (r1, p1) = Motion::BOARD_TOUR.rig_pose_ros(start + BOARD_HOLD_S - 0.01);
            assert!(r0.angle_between(r1) < 1e-6 && (p0 - p1).length() < 1e-9);
            let camera = true_rig().camera;
            assert!((p0 + r0 * camera.translation_m - *position).length() < 1e-9);
        }
    }

    #[test]
    fn the_board_pattern_has_a_white_margin_and_alternating_squares() {
        let board = BoardSpec::parse("7x5x0.08").unwrap();
        let half = board.half_extents_m();
        let s = board.square_m;
        // The margin square is white, the first pattern square dark, its neighbour light.
        assert!(board.gray(-half.x + 0.5 * s, -half.y + 0.5 * s) > 0.5);
        assert!(board.gray(-half.x + 1.5 * s, -half.y + 1.5 * s) < 0.5);
        assert!(board.gray(-half.x + 2.5 * s, -half.y + 1.5 * s) > 0.5);
        assert!(BoardSpec::parse("7x5").is_err() && BoardSpec::parse("1x5x0.1").is_err());
    }

    #[test]
    fn a_second_camera_is_recorded_with_its_own_topics_and_mounting() {
        let dir = std::env::temp_dir().join("calibrex_rig_camera2");
        fs::create_dir_all(&dir).unwrap();
        let bag = dir.join("rig.mcap");
        let recording = simulate(
            &bag,
            0.2,
            true_rig(),
            Sensors::parse("camera,camera2").unwrap(),
            Motion::BOARD_TOUR,
            CameraOptions::default(),
            Some(BoardSpec::parse("7x5x0.08").unwrap()),
        )
        .unwrap();
        assert_eq!((recording.camera_frames, recording.camera2_frames), (2, 2));
        let bytes = fs::read(&bag).unwrap();
        for topic in [IMAGE2_TOPIC, CAMERA2_INFO_TOPIC] {
            assert!(read_records(&bytes).iter().any(|(op, body)| *op == 0x04
                && body.windows(topic.len()).any(|w| w == topic.as_bytes())));
        }
        let yaml = frames_yaml(true_rig(), Sensors::parse("camera,camera2").unwrap());
        assert!(yaml.contains("name: camera2"));
        let rig = true_rig();
        assert!(
            ((rig.camera.translation_m - rig.camera2.translation_m).length() - 0.10).abs() < 1e-12
        );
    }

    #[test]
    fn camera_info_follows_the_lens_and_the_mode() {
        let dir = std::env::temp_dir().join("calibrex_rig_camera_info_modes");
        fs::create_dir_all(&dir).unwrap();
        let sensors = Sensors::parse("camera").unwrap();
        let lens = CameraOptions::parse_lens("-0.1,0.02,0.0,0.0,-0.003").unwrap();
        for (mode, expected) in [
            (CameraInfoMode::True, Some(true)),
            (CameraInfoMode::Zero, Some(false)),
            (CameraInfoMode::None, None),
        ] {
            let bag = dir.join(format!("{mode:?}.mcap"));
            simulate(
                &bag,
                0.2,
                true_rig(),
                sensors,
                Motion::TURN_IN_PLACE,
                CameraOptions { lens, info: mode },
                None,
            )
            .unwrap();
            let bytes = fs::read(&bag).unwrap();
            let declared = read_records(&bytes).iter().any(|(op, body)| {
                *op == 0x04
                    && body
                        .windows(CAMERA_INFO_TOPIC.len())
                        .any(|w| w == CAMERA_INFO_TOPIC.as_bytes())
            });
            assert_eq!(declared, expected.is_some(), "{mode:?}");
        }
        assert!(CameraOptions::parse_lens("1,2,3").is_err());
        assert!(CameraOptions::parse_info("maybe").is_err());
        assert_eq!(lens.k3, -0.003);
    }

    #[test]
    fn stationary_imu_reports_gravity_in_its_own_frame() {
        let mount = true_rig().imu;
        let mut world = World::new();
        let rig = spawn_named(&mut world, "rig");
        let (rotation, position) = Motion::HAND_HELD.rig_pose_ros(0.0);
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
                m * mount.translation_m,
                m * mount.rotation,
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
        let up_in_imu = (rotation * mount.rotation).inverse() * Vec3::Z;
        let expected = up_in_imu * 9.81;
        let measured = sample.measurement.linear_acceleration_m_s2;
        assert!(
            (measured - expected).length() < 1.0e-9,
            "{measured:?} vs {expected:?}"
        );
    }

    #[test]
    fn camera_optical_axis_points_along_the_lidar_x_axis() {
        // With the small tilt removed, the optical z axis is the LiDAR x axis.
        let rig = true_rig();
        let tilt = intrinsic_xyz_deg(4.0, -2.5, 1.5);
        let level = rig.camera.rotation * tilt.inverse();
        assert!((level * Vec3::Z - Vec3::X).length() < 1e-12);
        assert!((level * Vec3::Y - -Vec3::Z).length() < 1e-12);
        // The render frame looks down its -z, which is the optical +z.
        let render = rig.camera.rotation * optical_from_render();
        assert!((render * -Vec3::Z - rig.camera.rotation * Vec3::Z).length() < 1e-12);
    }

    #[test]
    fn each_turned_candidate_moves_one_sensor_by_the_turn_angle() {
        let good = true_rig();
        for (bad, moved) in [
            (turned_imu(good, TURN_DEG), "imu"),
            (turned_lidar2(good, TURN_DEG), "lidar2"),
            (turned_camera(good, TURN_DEG), "camera"),
        ] {
            for (name, a, b) in [
                ("imu", good.imu, bad.imu),
                ("lidar2", good.lidar2, bad.lidar2),
                ("camera", good.camera, bad.camera),
            ] {
                let angle = a.rotation.angle_between(b.rotation).to_degrees();
                let expected = if name == moved { TURN_DEG } else { 0.0 };
                assert!(
                    (angle - expected).abs() < 1e-4,
                    "{moved}: {name} moved {angle}"
                );
                assert_eq!(a.translation_m, b.translation_m);
            }
        }
        let yaml = frames_yaml(good, Sensors::ALL);
        assert_eq!(yaml.matches("parent: lidar").count(), 3);
    }
}
