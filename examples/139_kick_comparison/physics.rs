//! Headless recordings of two dynamic robots under bounded external wrenches.
//!
//! Robot motion comes entirely from the existing scenes and position motors.
//! The animated human is added by the renderer and never participates in physics.

use rne_ai::{build_visual_render_scene, UrdfJointPositionTarget, UrdfSceneSim};
use rne_math::Vec3;
use rne_physics::RigidBody;
use rne_robot::{Joint, JointKind, Link};
use rne_world::world_transform_of;
use serde_json::{json, Value};
use std::{error::Error, fs, io, path::Path};

const PUSH_STEPS: usize = 12;
const RECOVERY_STEPS: usize = 240;
const PRE_ROLL_STEPS: usize = 120;
const CAPTURE_INTERVAL_STEPS: usize = 2;
const PUSH_OFFSET_Y_M: f64 = 0.06;
const GO2_FEET: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];
const G1_FEET: [&str; 2] = ["left_ankle_roll_link", "right_ankle_roll_link"];

type CaptureResult<T> = Result<T, Box<dyn Error>>;

#[derive(Clone, Copy, Debug)]
enum Robot {
    Go2,
    G1,
}

impl Robot {
    fn name(self) -> &'static str {
        match self {
            Self::Go2 => "Go2",
            Self::G1 => "G1",
        }
    }

    fn base_link(self) -> &'static str {
        match self {
            Self::Go2 => "base",
            Self::G1 => "pelvis",
        }
    }

    fn force_n(self) -> f64 {
        match self {
            Self::Go2 => 120.0,
            Self::G1 => 50.0,
        }
    }

    fn feet(self) -> &'static [&'static str] {
        match self {
            Self::Go2 => &GO2_FEET,
            Self::G1 => &G1_FEET,
        }
    }

    fn targets(self) -> Vec<UrdfJointPositionTarget<'static>> {
        match self {
            Self::Go2 => [
                ("FL_hip", "FL_thigh", "FL_calf"),
                ("FR_hip", "FR_thigh", "FR_calf"),
                ("RL_hip", "RL_thigh", "RL_calf"),
                ("RR_hip", "RR_thigh", "RR_calf"),
            ]
            .into_iter()
            .flat_map(|(hip, thigh, calf)| [(hip, 0.0), (thigh, 0.8), (calf, -1.5)])
            .map(|(link_name, position)| UrdfJointPositionTarget {
                link_name,
                position,
            })
            .collect(),
            Self::G1 => [
                ("left_hip_pitch_link", -0.18),
                ("left_hip_roll_link", 0.0),
                ("left_knee_link", 0.36),
                ("left_ankle_pitch_link", -0.18),
                ("right_hip_pitch_link", -0.18),
                ("right_hip_roll_link", 0.0),
                ("right_knee_link", 0.36),
                ("right_ankle_pitch_link", -0.18),
                ("torso_link", 0.0),
                ("left_shoulder_pitch_link", 0.0),
                ("right_shoulder_pitch_link", 0.0),
                ("left_elbow_link", 0.42),
                ("right_elbow_link", 0.42),
            ]
            .into_iter()
            .map(|(link_name, position)| UrdfJointPositionTarget {
                link_name,
                position,
            })
            .collect(),
        }
    }

    fn load(self) -> CaptureResult<(UrdfSceneSim, Value)> {
        // These helpers resolve the checked-out assets relative to rne_ai's
        // manifest directory, without depending on the caller's working directory.
        let scene_path = match self {
            Self::Go2 => UrdfSceneSim::unitree_go2_dynamic_scene_path(),
            Self::G1 => UrdfSceneSim::unitree_g1_dynamic_scene_path(),
        };
        let mut sim = UrdfSceneSim::from_scene_path(&scene_path)?;
        let motors = match self {
            Self::Go2 => {
                sim.configure_position_motors(180.0, 18.0, 23.7);
                json!({"stiffness_parameter":180.0,"damping_parameter":18.0,
                    "effort_cap_nm":23.7})
            }
            Self::G1 => configure_g1_motors(&mut sim)?,
        };
        require(
            sim.fixed_delta().ticks() == 16_666_666,
            "the recorded scene must use its existing 60 Hz fixed step",
        )?;
        Ok((sim, motors))
    }

    fn tilt_rad(self, sim: &UrdfSceneSim) -> CaptureResult<f64> {
        match self {
            Self::Go2 => {
                let observation = sim.observe();
                Ok(observation
                    .base_relative_roll_rad
                    .abs()
                    .max(observation.base_relative_pitch_rad.abs()))
            }
            Self::G1 => {
                let pelvis = sim
                    .named_transform("pelvis")
                    .ok_or_else(|| io::Error::other("the G1 pelvis is missing"))?;
                Ok((pelvis.rotation * Vec3::Z).y.clamp(-1.0, 1.0).acos())
            }
        }
    }
}

fn require(condition: bool, message: &str) -> CaptureResult<()> {
    if condition {
        Ok(())
    } else {
        Err(io::Error::other(message).into())
    }
}

fn configure_g1_motors(sim: &mut UrdfSceneSim) -> CaptureResult<Value> {
    sim.configure_position_motors(220.0, 24.0, 88.0);
    let mut limits = Vec::new();
    for entity in sim.world().iter_entities() {
        let Some(joint) = entity.get::<Joint>() else {
            continue;
        };
        if joint.kind == JointKind::Fixed {
            continue;
        }
        let child = sim
            .world()
            .get::<Link>(joint.child_link)
            .ok_or_else(|| io::Error::other("a G1 joint has no named child link"))?;
        limits.push((child.name.clone(), joint.limits));
    }
    limits.sort_by(|a, b| a.0.cmp(&b.0));
    let mut recorded_limits = Vec::new();
    for (name, limits) in limits {
        let cap_nm = limits.max_effort.min(88.0);
        require(
            cap_nm.is_finite() && cap_nm >= 0.0,
            "a G1 motor effort cap is invalid",
        )?;
        require(
            sim.configure_named_position_motor(&name, 220.0, 24.0, cap_nm),
            "a G1 named motor could not be configured",
        )?;
        recorded_limits.push(json!({"link":name,
            "authored_effort_limit_nm":limits.max_effort,
            "configured_effort_limit_nm":cap_nm,
            "authored_speed_limit_rad_s":limits.max_velocity}));
    }
    Ok(json!({"stiffness_parameter":220.0,"damping_parameter":24.0,
        "limits":recorded_limits,"authored_effort_caps_applied":true}))
}

#[derive(Debug)]
struct Recording {
    trace: Value,
    state_bits: Vec<Vec<u64>>,
}

#[derive(Debug)]
struct Metrics {
    peak_tilt_rad: f64,
    min_height_m: f64,
    ever_fallen: bool,
    min_contact_feet: usize,
}

impl Metrics {
    fn measure(&mut self, robot: Robot, sim: &UrdfSceneSim) -> CaptureResult<()> {
        self.peak_tilt_rad = self.peak_tilt_rad.max(robot.tilt_rad(sim)?);
        let height_m = sim.observe().base_y_m;
        self.min_height_m = self.min_height_m.min(height_m);
        self.ever_fallen |= height_m < 0.35;
        self.min_contact_feet = self.min_contact_feet.min(contact_feet(robot, sim));
        Ok(())
    }
}

fn contact_feet(robot: Robot, sim: &UrdfSceneSim) -> usize {
    robot
        .feet()
        .iter()
        .filter(|foot| sim.link_contact_impulse_ns(foot) > 0.0)
        .count()
}

/// Observed state is ordered by link name, not ECS or map iteration order.
fn state_bits(robot: Robot, sim: &UrdfSceneSim) -> Vec<u64> {
    let mut links: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|entity| {
            entity
                .get::<Link>()
                .map(|link| (link.name.as_str(), entity.id()))
        })
        .collect();
    links.sort_by(|a, b| a.0.cmp(b.0));
    let mut bits = Vec::new();
    for (name, entity) in links {
        let transform = world_transform_of(sim.world(), entity);
        bits.extend(
            transform
                .translation
                .to_array()
                .into_iter()
                .chain(transform.rotation.to_array())
                .chain(transform.scale.to_array())
                .map(f64::to_bits),
        );
        if let Some(body) = sim.world().get::<RigidBody>(entity) {
            bits.push(1);
            bits.extend(
                body.linear_velocity_m_s
                    .to_array()
                    .into_iter()
                    .map(f64::to_bits),
            );
            bits.extend(
                body.angular_velocity_rad_s
                    .to_array()
                    .into_iter()
                    .map(f64::to_bits),
            );
        } else {
            bits.push(0);
        }
        for joint_value in [
            sim.named_joint_position(name),
            sim.named_joint_velocity(name),
        ] {
            if let Some(value) = joint_value {
                bits.extend([1, value.to_bits()]);
            } else {
                bits.push(0);
            }
        }
    }
    bits.extend(
        robot
            .feet()
            .iter()
            .map(|foot| sim.link_contact_impulse_ns(foot).to_bits()),
    );
    bits
}

fn stable_hash(states: &[Vec<u64>]) -> String {
    let hash = states
        .iter()
        .flatten()
        .flat_map(|bits| bits.to_le_bytes())
        .fold(0xcbf2_9ce4_8422_2325_u64, |hash, byte| {
            (hash ^ u64::from(byte)).wrapping_mul(0x0000_0100_0000_01b3)
        });
    format!("{hash:016x}")
}

fn frame(
    sim: &UrdfSceneSim,
    step: usize,
    force_n: f64,
    application_point_world_m: Option<Vec3>,
) -> Value {
    let mut items: Vec<_> = build_visual_render_scene(sim.world())
        .items
        .into_iter()
        .map(|item| {
            json!({"shape":item.shape,"color_rgba":item.color_rgba,
                "transform":{"translation":item.transform.translation.to_array(),
                    "rotation":item.transform.rotation.to_array(),
                    "scale":item.transform.scale.to_array()}})
        })
        .collect();
    // Canonicalize presentation order only; simulation accumulation is untouched.
    items.sort_by_cached_key(Value::to_string);
    json!({"step":step,"time_s":step as f64 * sim.fixed_delta().as_seconds().value(),
        "force_n":force_n,"force_world_n":[0.0,0.0,force_n],
        "application_point_world_m":application_point_world_m.map(|point|point.to_array()),
        "items":items})
}

fn settle(
    robot: Robot,
    sim: &mut UrdfSceneSim,
    targets: &[UrdfJointPositionTarget<'_>],
    states: &mut Vec<Vec<u64>>,
    frames: &mut Vec<Value>,
) {
    let steps = match robot {
        Robot::Go2 => 180,
        Robot::G1 => 240,
    };
    for step in 1..=steps {
        sim.step_joint_position_targets(targets);
        states.push(state_bits(robot, sim));
        if matches!(robot, Robot::Go2) && step >= 60 && step % CAPTURE_INTERVAL_STEPS == 0 {
            frames.push(frame(sim, step - 60, 0.0, None));
        }
    }
    if matches!(robot, Robot::G1) {
        frames.push(frame(sim, 0, 0.0, None));
    }
}

fn record(robot: Robot) -> CaptureResult<Recording> {
    let (mut sim, motors) = robot.load()?;
    let targets = robot.targets();
    let dt_s = sim.fixed_delta().as_seconds().value();
    let mut states = Vec::new();
    let mut frames = Vec::new();
    settle(robot, &mut sim, &targets, &mut states, &mut frames);
    let settled = sim.observe();
    let mut metrics = Metrics {
        peak_tilt_rad: if matches!(robot, Robot::G1) {
            robot.tilt_rad(&sim)?
        } else {
            0.0
        },
        min_height_m: settled.base_y_m,
        ever_fallen: false,
        min_contact_feet: robot.feet().len(),
    };
    let extra_pre_roll_steps = if matches!(robot, Robot::G1) {
        PRE_ROLL_STEPS
    } else {
        0
    };
    let mut force_history = Vec::new();
    for step in 1..=extra_pre_roll_steps + PUSH_STEPS + RECOVERY_STEPS {
        let pushing = step > extra_pre_roll_steps && step <= extra_pre_roll_steps + PUSH_STEPS;
        let force_n = if pushing { robot.force_n() } else { 0.0 };
        let application_point_world_m = if pushing {
            let position_m = sim
                .named_link_position_m(robot.base_link())
                .ok_or_else(|| io::Error::other("the dynamic robot base is missing"))?;
            let point_m = position_m + Vec3::new(0.0, PUSH_OFFSET_Y_M, 0.0);
            require(
                sim.apply_named_link_wrench(
                    robot.base_link(),
                    point_m,
                    Vec3::new(0.0, 0.0, force_n),
                    Vec3::ZERO,
                ),
                "the dynamic robot base rejected its external wrench",
            )?;
            Some(point_m)
        } else {
            None
        };
        sim.step_joint_position_targets(&targets);
        metrics.measure(robot, &sim)?;
        states.push(state_bits(robot, &sim));
        let display_step = match robot {
            Robot::Go2 => PRE_ROLL_STEPS + step,
            Robot::G1 => step,
        };
        force_history.push(
            json!({"step":display_step,"time_s":display_step as f64 * dt_s,
            "force_world_n":[0.0,0.0,force_n],"dt_s":dt_s,
            "application_point_world_m":application_point_world_m.map(|point|point.to_array())}),
        );
        if display_step % CAPTURE_INTERVAL_STEPS == 0 {
            frames.push(frame(
                &sim,
                display_step,
                force_n,
                application_point_world_m,
            ));
        }
    }
    let final_state = sim.observe();
    let residual_tilt_rad = robot.tilt_rad(&sim)?;
    let final_speed_m_s = Vec3::new(
        final_state.base_linear_velocity_x_m_s,
        final_state.base_linear_velocity_y_m_s,
        final_state.base_linear_velocity_z_m_s,
    )
    .length();
    let final_contact_feet = contact_feet(robot, &sim);
    let recovered = match robot {
        Robot::Go2 => {
            final_state.base_y_m > 0.18 && residual_tilt_rad < 0.20 && final_contact_feet == 4
        }
        Robot::G1 => {
            !metrics.ever_fallen
                && final_state.base_y_m > 0.70
                && residual_tilt_rad < 0.20
                && final_contact_feet == 2
                && final_speed_m_s < 0.10
        }
    };
    require(
        recovered,
        &format!("{} did not meet its unchanged recovery gate", robot.name()),
    )?;
    require(
        frames.len() == 187,
        "the capture must contain exactly 187 frames",
    )?;
    require(
        frames.iter().position(|frame| frame["force_n"] != 0.0) == Some(61),
        "the first force frame must be frame 61",
    )?;
    let mesh_package_roots: Vec<_> = sim
        .mesh_package_roots()
        .iter()
        .map(|root| root.to_string_lossy().into_owned())
        .collect();
    let summary = json!({"recovered":recovered,"peak_tilt_rad":metrics.peak_tilt_rad,
        "residual_tilt_rad":residual_tilt_rad,"final_tilt_rad":residual_tilt_rad,
        "final_base_height_m":final_state.base_y_m,"final_pelvis_height_m":final_state.base_y_m,
        "final_speed_m_s":final_speed_m_s,"min_height_m":metrics.min_height_m,
        "lateral_displacement_m":final_state.base_z_m - settled.base_z_m,
        "final_contact_feet":final_contact_feet,"min_contact_feet":metrics.min_contact_feet});
    Ok(Recording {
        trace: json!({"schema_version":1,"robot":robot.name(),"backend":"Rapier through UrdfSceneSim",
            "coordinate_system":"Y-up","dt_s":dt_s,"capture_interval_steps":CAPTURE_INTERVAL_STEPS,
            "push_start_s":PRE_ROLL_STEPS as f64 * dt_s,"push_duration_s":PUSH_STEPS as f64 * dt_s,
            "force_n":robot.force_n(),"impulse_n_s":robot.force_n() * PUSH_STEPS as f64 * dt_s,
            "force_link":robot.base_link(),"application_offset_m":[0.0,PUSH_OFFSET_Y_M,0.0],
            "motors":motors,"mesh_package_roots":mesh_package_roots,
            "mechanism":"Actual external wrench; animated human is render-only; robot poses are never edited",
            "summary":summary,"force_history":force_history,"frames":frames,
            "observed_state_definition":"Link-name order: world pose/scale, body linear/angular velocity, joint q/qd with presence tags; fixed foot order normal impulse. IEEE f64 bits; excludes hidden solver caches.",
            "observed_state_hash":stable_hash(&states),"observed_state_bits":states}),
        state_bits: states,
    })
}

/// Records the Go2 and G1 demonstrations and verifies their complete observed replays.
///
/// Writes `go2-trace.json`, `g1-trace.json`, and `summary.json` into `output_dir`.
/// The unequal forces are preserved explicitly: Go2 receives 120 N and G1 50 N,
/// both over twelve fixed steps. Neither the animated human nor pose corrections
/// participate in robot dynamics. Recovery or literal state-bit mismatch is an error.
pub(super) fn capture(output_dir: &Path) -> CaptureResult<()> {
    fs::create_dir_all(output_dir)?;
    let mut summaries = Vec::new();
    for robot in [Robot::Go2, Robot::G1] {
        let mut first = record(robot)?;
        let repeat = record(robot)?;
        require(
            first.state_bits == repeat.state_bits,
            "the observed physics replay differs bit for bit",
        )?;
        require(
            first.trace["frames"] == repeat.trace["frames"],
            "the canonical render frames differ on replay",
        )?;
        require(
            first.trace["observed_state_hash"] == repeat.trace["observed_state_hash"],
            "the stable replay hashes differ",
        )?;
        first.trace["observed_state_bits_exact_repeat"] = json!(true);
        first.trace["canonical_render_frames_exact_repeat"] = json!(true);
        let filename = format!("{}-trace.json", robot.name().to_lowercase());
        fs::write(
            output_dir.join(&filename),
            serde_json::to_vec(&first.trace)?,
        )?;
        summaries.push(
            json!({"robot":robot.name(),"trace":filename,"force_n":robot.force_n(),
            "dt_s":first.trace["dt_s"],"duration_s":first.trace["push_duration_s"],
            "impulse_n_s":first.trace["impulse_n_s"],"summary":first.trace["summary"],
            "observed_state_hash":first.trace["observed_state_hash"],
            "observed_state_bits_exact_repeat":true,"canonical_render_frames_exact_repeat":true}),
        );
    }
    fs::write(
        output_dir.join("summary.json"),
        serde_json::to_vec_pretty(&json!({
        "note":"Measured dynamic simulations with different explicit force magnitudes; this is not a common-strength comparison or a simulated human collision.",
        "cases":summaries}))?,
    )?;
    Ok(())
}
