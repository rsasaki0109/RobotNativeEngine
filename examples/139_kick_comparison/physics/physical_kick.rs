//! Recovery from a simulated kicking leg instead of a prescribed wrench.
//!
//! The same plant, controller, settlement and recovery predicate as the
//! prescribed-wrench experiment are used. After settlement a physical leg
//! (see [`crate::kicker`]) is placed beside the robot and follows the animated
//! kick under torque limits. Whatever the shoe delivers through contact is the
//! disturbance; it is measured, never normalized to a declared impulse.

use super::*;
use crate::kicker::{ignore_kicker_contacts, KickContact, KickLeg, KickLegModel, KickSchedule};
use rne_physics::{Collider, ColliderShape};
use rne_render::GltfSceneAsset;

/// Recording-time speed-up of the authored 350 ms extension; 2.0 keeps the
/// 175 ms extension of the earlier visual kick.
pub(crate) const DEFAULT_SPEED_SCALE: f64 = 2.0;
/// How far the fully extended reference shoe would pass the near body surface.
pub(crate) const DEFAULT_FOLLOW_THROUGH_M: f64 = 0.10;
const SWEEP_SPEED_SCALES: [f64; 5] = [1.0, 1.5, 2.0, 2.5, 3.0];
/// Window for the reported smoothed peak force, comparable to force-plate
/// bandwidths; the raw per-step peak of a rigid impact depends on the step.
const FORCE_WINDOW_S: f64 = 0.01;

/// One physical kick: extension speed and aim depth.
#[derive(Clone, Copy, Debug)]
pub(crate) struct PhysicalKick {
    pub(crate) speed_scale: f64,
    pub(crate) follow_through_m: f64,
}

impl PhysicalKick {
    fn name(self) -> String {
        format!(
            "speed_{:.2}_follow_{:.3}m",
            self.speed_scale, self.follow_through_m
        )
    }

    fn validate(self) -> CaptureResult<KickSchedule> {
        require(
            self.follow_through_m.is_finite() && (0.0..=0.3).contains(&self.follow_through_m),
            "follow-through must be within 0..=0.3 m",
        )?;
        KickSchedule::new(self.speed_scale)
    }
}

#[derive(Debug)]
struct PhysicalRecording {
    trace: Value,
    state_bits: Vec<Vec<u64>>,
    kick_words: Vec<Vec<u64>>,
    control_state_words: Vec<Vec<u64>>,
}

/// Contact statistics accumulated over a recording.
#[derive(Debug, Default)]
struct ContactLog {
    impulse_on_robot_ns: Vec3,
    peak_force_n: f64,
    first_step: Option<usize>,
    last_step: Option<usize>,
    contact_steps: usize,
    episodes: usize,
    shoe_steps: usize,
    shank_or_thigh_steps: usize,
    shoe_speed_before_contact_m_s: Option<Vec3>,
    window: std::collections::VecDeque<Vec3>,
    peak_window_force_n: f64,
    history: Vec<Value>,
}

impl ContactLog {
    fn record(
        &mut self,
        step: usize,
        dt_s: f64,
        contact: KickContact,
        shoe_velocity_before_m_s: Vec3,
    ) {
        self.window.push_back(contact.impulse_on_robot_ns);
        if self.window.len() > (FORCE_WINDOW_S / dt_s).round() as usize {
            self.window.pop_front();
        }
        let window_impulse = self.window.iter().fold(Vec3::ZERO, |sum, i| sum + *i);
        self.peak_window_force_n = self
            .peak_window_force_n
            .max(window_impulse.length() / FORCE_WINDOW_S);
        if contact.touching_pairs == 0 {
            return;
        }
        if self.last_step != Some(step - 1) {
            self.episodes += 1;
        }
        self.first_step.get_or_insert(step);
        self.shoe_speed_before_contact_m_s
            .get_or_insert(shoe_velocity_before_m_s);
        self.last_step = Some(step);
        self.contact_steps += 1;
        self.shoe_steps += usize::from(contact.shoe_touching);
        self.shank_or_thigh_steps += usize::from(contact.shank_or_thigh_touching);
        self.impulse_on_robot_ns += contact.impulse_on_robot_ns;
        let force = contact.impulse_on_robot_ns / dt_s;
        self.peak_force_n = self.peak_force_n.max(force.length());
        self.history
            .push(json!({"step":step,"time_s":step as f64*dt_s,
            "force_on_robot_world_n":force.to_array(),"touching_pairs":contact.touching_pairs,
            "shoe":contact.shoe_touching,"shank_or_thigh":contact.shank_or_thigh_touching}));
    }

    fn summary(&self, dt_s: f64) -> Value {
        let span_s = match (self.first_step, self.last_step) {
            (Some(first), Some(last)) => Some((last - first + 1) as f64 * dt_s),
            _ => None,
        };
        json!({"contact_occurred":self.contact_steps > 0,
            "normal_impulse_on_robot_world_ns":self.impulse_on_robot_ns.to_array(),
            "normal_impulse_magnitude_ns":self.impulse_on_robot_ns.length(),
            "peak_normal_force_per_step_n":self.peak_force_n,
            "peak_normal_force_10ms_average_n":self.peak_window_force_n,
            "first_contact_time_s":self.first_step.map(|s| s as f64 * dt_s),
            "last_contact_time_s":self.last_step.map(|s| s as f64 * dt_s),
            "contact_span_s":span_s,"contact_time_s":self.contact_steps as f64 * dt_s,
            "contact_episodes":self.episodes,"shoe_contact_steps":self.shoe_steps,
            "shank_or_thigh_contact_steps":self.shank_or_thigh_steps,
            "shoe_velocity_before_first_contact_world_m_s":self.shoe_speed_before_contact_m_s.map(|v| v.to_array()),
            "shoe_speed_before_first_contact_m_s":self.shoe_speed_before_contact_m_s.map(|v| v.length()),
            "impulse_scope":"sum of solver normal impulses between kicker colliders and robot links; friction impulse excluded"})
    }
}

/// Near body surface the kick aims at: the root's X, the face nearest the human.
fn target_surface(robot: Robot, sim: &UrdfSceneSim) -> CaptureResult<Vec3> {
    let base = sim
        .world()
        .iter_entities()
        .find(|entity| {
            entity
                .get::<Link>()
                .is_some_and(|l| l.name == robot.base_link())
        })
        .ok_or_else(|| io::Error::other("missing robot base link"))?;
    let collider = base
        .get::<Collider>()
        .ok_or_else(|| io::Error::other("the kicked base has no collider"))?;
    let ColliderShape::Cuboid { half_extents_m } = collider.shape else {
        return Err(io::Error::other("the kicked base must be a box").into());
    };
    let pose = world_transform_of(sim.world(), base.id());
    let rotation = pose.rotation * collider.local_offset.rotation;
    let center = pose.translation + pose.rotation * collider.local_offset.translation;
    let half_z: f64 = [Vec3::X, Vec3::Y, Vec3::Z]
        .into_iter()
        .zip(half_extents_m.to_array())
        .map(|(axis, half)| (rotation * axis).z.abs() * half)
        .sum();
    Ok(Vec3::new(center.x, center.y, center.z - half_z))
}

fn record_physical(
    robot: Robot,
    kick: PhysicalKick,
    capture_frames: bool,
    asset: &GltfSceneAsset,
    model: &KickLegModel,
) -> CaptureResult<PhysicalRecording> {
    let schedule = kick.validate()?;
    require(
        matches!(robot, Robot::Go2),
        "the physical kick currently supports the Go2 box body only",
    )?;
    let case = Case {
        name: "physical_kick",
        impulse_ns: 0.0,
        ..NOMINAL
    };
    let mut sim = robot.load(case.dt_ticks)?;
    let initial_plant_contract = robot.validate(&sim)?;
    let mut controller = Controller::new(robot, &mut sim, true, Profile::ideal(case.dt_ticks))?;
    let mut motor = None;
    let dt_s = sim.fixed_delta().as_seconds().value();
    let mut states = Vec::new();
    let mut control_states = Vec::new();
    let mut settlement_telemetry = Vec::new();
    let ground_index = ground_body_index(&sim)?;
    let settlement = settle(
        robot,
        &mut sim,
        &mut controller,
        &mut states,
        &mut control_states,
        None,
        (&mut settlement_telemetry, &mut motor),
    )?;
    let settlement_summary = settlement_metadata(
        robot,
        &sim,
        &controller,
        &settlement,
        settlement_telemetry,
        0.0,
    )?;
    require(
        settlement.settled_upright,
        "the physical plant did not settle upright before the kick",
    )?;
    let initial_root = sim
        .named_link_position_m(robot.base_link())
        .expect("validated root");
    let target = target_surface(robot, &sim)?;
    let (origin, placement) = model.placement(asset, target, kick.follow_through_m);
    let ground = sim
        .world()
        .iter_entities()
        .find(|entity| entity.id().index() == ground_index)
        .map(|entity| entity.id())
        .ok_or_else(|| io::Error::other("missing ground"))?;
    ignore_kicker_contacts(&mut sim, ground);
    let mut leg = KickLeg::spawn(&mut sim, asset, model.clone(), schedule, origin)?;
    let mut metrics = Metrics::new(
        robot,
        &sim,
        (
            settlement.max_fixed_position_error_m,
            settlement.max_fixed_rotation_error_rad,
        ),
    )?;
    let mut kick_words = Vec::new();
    let mut contacts = ContactLog::default();
    let mut telemetry = Vec::new();
    let mut frames = Vec::new();
    let leg_frame = |leg: &KickLeg, sim: &UrdfSceneSim, time_s: f64| {
        json!({"animation_time_s":leg.schedule().asset_time_s(time_s),
            "human_origin_world_m":leg.origin().to_array(),
            "segments":leg.segment_poses(sim).map(|(p, q)| json!({"translation":p.to_array(),"rotation":q.to_array()}))})
    };
    if capture_frames {
        let mut first = frame(&sim, 0, Vec3::ZERO, None);
        first["kick_leg"] = leg_frame(&leg, &sim, 0.0);
        frames.push(first);
    }
    let steps = ((FRAME_COUNT - 1) as f64 / FRAME_RATE_HZ / dt_s).round() as usize;
    for step in 1..=steps {
        let start_s = (step - 1) as f64 * dt_s;
        let shoe_before = leg.shoe_velocity_m_s(&sim);
        leg.command(&mut sim, asset, start_s, dt_s)?;
        controller.step(&mut sim)?;
        let contact = leg.contact(&sim, dt_s)?;
        contacts.record(step, dt_s, contact, shoe_before);
        let (tilt, height) = metrics.measure(robot, &sim, step, steps, ground_index)?;
        states.push(state_bits(robot, &sim));
        kick_words.push(leg.state_words(&sim));
        control_states.push(controller.replay_words());
        if step % (0.02 / dt_s).round() as usize == 0 {
            let mut sample =
                measured_telemetry(robot, &sim, &controller, step as f64 * dt_s, tilt, height)?;
            sample["kick_leg"] = json!({"joint_state_rad_rad_s":leg.joint_state(&sim),
                "efforts_nm":leg.efforts_nm()});
            telemetry.push(sample);
        }
        if capture_frames
            && frames.len() < FRAME_COUNT
            && step == (frames.len() as f64 / FRAME_RATE_HZ / dt_s).round() as usize
        {
            let force = contact.impulse_on_robot_ns / dt_s;
            let mut captured = frame(&sim, step, force, None);
            captured["kick_leg"] = leg_frame(&leg, &sim, step as f64 * dt_s);
            frames.push(captured);
        }
    }
    let mut summary = metrics.recording_summary(robot, &sim, case, initial_root, true)?;
    let contact_summary = contacts.summary(dt_s);
    summary["impact_applied"] = contact_summary["contact_occurred"].clone();
    let roots: Vec<_> = sim
        .mesh_package_roots()
        .iter()
        .map(|p| p.to_string_lossy().into_owned())
        .collect();
    println!(
        "{} {} contact={} impulse={:.3}N.s peak_10ms={:.1}N recovered={}",
        robot.name(),
        kick.name(),
        contact_summary["contact_occurred"],
        contacts.impulse_on_robot_ns.length(),
        contacts.peak_window_force_n,
        summary["recovered"]
    );
    Ok(PhysicalRecording {
        trace: json!({"schema_version":1,"robot":robot.name(),"case":kick.name(),
            "compiled_source_sha256":compiled_source_hashes(),
            "physical_kick_source_sha256":physical_kick_source_hashes(),
            "backend":"Rapier through UrdfSceneSim","coordinate_system":"Y-up","dt_s":dt_s,
            "solver_iterations":32,"mechanism":"Simulated kicking leg; contact impulse is measured; robot poses are not edited",
            "kick":{"configuration":leg.configuration(),"placement":placement,
                "speed_scale":kick.speed_scale,"follow_through_m":kick.follow_through_m,
                "extension_end_s":leg.schedule().extension_end_s(),
                "measured":contact_summary,"leg_tracking":leg.telemetry()},
            "contact_history":contacts.history,
            "plant":initial_plant_contract,"controller":controller.configuration(),
            "observation_pipeline":controller.pipeline.configuration(),
            "settlement":settlement_summary,"mesh_package_roots":roots,
            "summary":summary,"telemetry":telemetry,"frames":frames,
            "observed_state_hash":stable_hash(&states),
            "observed_state_word_count":states.iter().map(Vec::len).sum::<usize>(),
            "kick_leg_state_hash":stable_hash(&kick_words),
            "observation_control_state_hash":stable_hash(&control_states)}),
        state_bits: states,
        kick_words,
        control_state_words: control_states,
    })
}

fn physical_kick_source_hashes() -> Value {
    [
        (
            "examples/139_kick_comparison/kicker.rs",
            include_str!("../kicker.rs"),
        ),
        (
            "examples/139_kick_comparison/physics/physical_kick.rs",
            include_str!("physical_kick.rs"),
        ),
    ]
    .into_iter()
    .map(|(name, contents)| {
        (
            name.to_owned(),
            json!(format!("{:x}", Sha256::digest(contents.as_bytes()))),
        )
    })
    .collect::<serde_json::Map<_, _>>()
    .into()
}

/// Records a kick twice from fresh plants and requires literal equality.
fn replay_physical(
    robot: Robot,
    kick: PhysicalKick,
    capture_frames: bool,
    asset: &GltfSceneAsset,
    model: &KickLegModel,
) -> CaptureResult<Value> {
    let mut first = record_physical(robot, kick, capture_frames, asset, model)?;
    let repeat = record_physical(robot, kick, capture_frames, asset, model)?;
    require(
        first.state_bits == repeat.state_bits,
        "the robot state did not replay exactly",
    )?;
    require(
        first.kick_words == repeat.kick_words,
        "the kicking leg did not replay exactly",
    )?;
    require(
        first.control_state_words == repeat.control_state_words,
        "the controller state did not replay exactly",
    )?;
    require(
        first.trace["frames"] == repeat.trace["frames"]
            && first.trace["contact_history"] == repeat.trace["contact_history"],
        "captured frames or contact history did not replay exactly",
    )?;
    first.trace["exact_replay"] = json!({"robot_state_words":true,"kick_leg_state_words":true,
        "controller_state_words":true,"frames_and_contacts":true});
    Ok(first.trace)
}

fn load_model() -> CaptureResult<(GltfSceneAsset, KickLegModel)> {
    let asset = rne_render::load_gltf_scene(Path::new(crate::HUMAN_ASSET))?;
    let model = KickLegModel::from_asset(&asset)?;
    Ok((asset, model))
}

/// Records the nominal physical kick with frames for rendering.
pub(crate) fn capture_physical(output: &Path, kick: PhysicalKick) -> CaptureResult<()> {
    fs::create_dir_all(output)?;
    let (asset, model) = load_model()?;
    let trace = replay_physical(Robot::Go2, kick, true, &asset, &model)?;
    require(
        trace["frames"]
            .as_array()
            .is_some_and(|f| f.len() == FRAME_COUNT),
        "incorrect frame count",
    )?;
    fs::write(
        output.join("go2-physical-trace.json"),
        serde_json::to_vec(&trace)?,
    )?;
    fs::write(
        output.join("physical-kick-summary.json"),
        serde_json::to_vec_pretty(&json!({"schema_version":1,"robot":"Go2",
            "kick":trace["kick"]["measured"],"speed_scale":kick.speed_scale,
            "follow_through_m":kick.follow_through_m,"summary":trace["summary"],
            "leg_tracking":trace["kick"]["leg_tracking"],"exact_replay":trace["exact_replay"],
            "trace":"go2-physical-trace.json"}))?,
    )?;
    require(
        trace["kick"]["measured"]["contact_occurred"] == true,
        "the simulated shoe never reached the robot; trace retained",
    )
}

/// Sweeps extension speed at the default aim; failed recoveries are retained.
pub(crate) fn sweep_physical(output: &Path, follow_through_m: f64) -> CaptureResult<()> {
    fs::create_dir_all(output)?;
    let (asset, model) = load_model()?;
    let mut cases = Vec::new();
    for speed_scale in SWEEP_SPEED_SCALES {
        let kick = PhysicalKick {
            speed_scale,
            follow_through_m,
        };
        let trace = replay_physical(Robot::Go2, kick, false, &asset, &model)?;
        cases.push(json!({"case":kick.name(),"speed_scale":speed_scale,
            "follow_through_m":follow_through_m,"kick":trace["kick"]["measured"],
            "leg_tracking":trace["kick"]["leg_tracking"],"summary":trace["summary"],
            "exact_replay":trace["exact_replay"],"observed_state_hash":trace["observed_state_hash"],
            "kick_leg_state_hash":trace["kick_leg_state_hash"]}));
    }
    fs::write(
        output.join("physical-kick-sweep.json"),
        serde_json::to_vec_pretty(&json!({"schema_version":1,"robot":"Go2",
            "note":"Measured kicks from one simulated leg; impulse and force are outputs, recovery is reported, not assumed",
            "leg":model.configuration(),"cases":cases}))?,
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Short check that fits a debug test run: the spawned leg holds the
    /// animation's standing pose beside the robot, touches nothing on the
    /// robot, and replays exactly. The full kick is the release capture.
    fn standing_words() -> (Vec<Vec<u64>>, Value) {
        let (asset, model) = load_model().unwrap();
        let mut sim = Robot::Go2.load(NOMINAL.dt_ticks).unwrap();
        let target = target_surface(Robot::Go2, &sim).unwrap();
        let (origin, _) = model.placement(&asset, target, DEFAULT_FOLLOW_THROUGH_M);
        let schedule = KickSchedule::new(DEFAULT_SPEED_SCALE).unwrap();
        let mut leg = KickLeg::spawn(&mut sim, &asset, model, schedule, origin).unwrap();
        let dt_s = sim.fixed_delta().as_seconds().value();
        let mut words = Vec::new();
        for step in 0..100 {
            leg.command(&mut sim, &asset, step as f64 * dt_s, dt_s)
                .unwrap();
            sim.step_joint_position_actuation_targets(&[]);
            assert_eq!(leg.contact(&sim, dt_s).unwrap().touching_pairs, 0);
            words.push(leg.state_words(&sim));
        }
        (words, leg.telemetry())
    }

    #[test]
    fn standing_kick_leg_tracks_its_pose_and_replays_exactly() {
        let (first, telemetry) = standing_words();
        let (repeat, _) = standing_words();
        assert_eq!(first, repeat);
        for joint in ["hip_abduction", "hip_flexion", "knee", "ankle"] {
            let error = telemetry["max_reference_error_rad"][joint]
                .as_f64()
                .unwrap();
            assert!(error < 0.05, "{joint} tracking error {error} rad");
        }
    }
}
