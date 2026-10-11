//! Clocked synthetic state estimates for a simulation-only observation probe.
//!
//! These estimates start from simulator truth; their bounded errors and timing
//! are declared sensitivity settings, not identified hardware sensor properties.
//! Raw IMU fusion, contact estimation and the actuator's inner PD are out of scope.

use super::model;
use rne_ai::UrdfSceneSim;
use rne_core::{KeyedRandom, SimDuration, SimTime};
use rne_data::{DataBus, Frame, FramePayload, InMemoryDataBus, StreamId};
use rne_ecs::Entity;
use rne_math::{Quat, Vec3};
use rne_robot::{Joint, JointKind, Link};
use rne_world::WorldRandom;
use serde_json::{json, Value};
use std::{borrow::Cow, collections::VecDeque, error::Error, io};

type ObservationResult<T> = Result<T, Box<dyn Error>>;
const ESTIMATE_STREAM: StreamId = StreamId::new(0x139);
const ESTIMATE_NOISE_DOMAIN: u64 = 0x4b49_434b_4553_5431;

#[derive(Clone, Debug)]
pub(super) struct JointObservation {
    pub(super) link_name: String,
    pub(super) position_rad: Option<f64>,
    pub(super) velocity_rad_s: Option<f64>,
}

/// One consistently captured estimate; joint coordinates are telemetry only.
#[derive(Clone, Debug)]
pub(super) struct Observation {
    pub(super) com_world_m: Vec3,
    pub(super) com_velocity_world_m_s: Vec3,
    pub(super) mass_kg: f64,
    pub(super) up_world: Vec3,
    pub(super) forward_unit: Vec3,
    pub(super) lateral_unit: Vec3,
    pub(super) roll_rad: f64,
    pub(super) pitch_rad: f64,
    pub(super) angular_velocity_world_rad_s: Vec3,
    pub(super) roll_rate_rad_s: f64,
    pub(super) foot_positions_world_m: Vec<Vec3>,
    pub(super) foot_normal_loads_n: Vec<f64>,
    pub(super) joint_states: Vec<JointObservation>,
    base_rotation_world: Quat,
    go2: bool,
    load_saturations: u64,
}

impl FramePayload for Observation {}

impl Observation {
    /// Captures one completed world tick; evaluator callers must not feed it back.
    pub(super) fn capture(
        sim: &UrdfSceneSim,
        base_link: &str,
        feet: &[&str],
    ) -> ObservationResult<Self> {
        let (com_world_m, com_velocity_world_m_s, mass_kg) = model::center_of_mass(sim)?;
        let base_rotation_world = sim
            .named_transform(base_link)
            .ok_or_else(|| io::Error::other("the estimate base pose is missing"))?
            .rotation;
        let state = sim.observe();
        let angular_velocity_world_rad_s = Vec3::new(
            state.base_angular_velocity_x_rad_s,
            state.base_angular_velocity_y_rad_s,
            state.base_angular_velocity_z_rad_s,
        );
        let foot_positions_world_m = feet
            .iter()
            .map(|name| {
                sim.named_transform(name)
                    .map(|pose| pose.translation)
                    .ok_or_else(|| io::Error::other("an estimate foot pose is missing"))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let foot_normal_loads_n = feet
            .iter()
            .map(|name| {
                sim.named_body_contact_loads(name)
                    .iter()
                    .map(|(_, force_n)| force_n.max(0.0))
                    .sum()
            })
            .collect();
        let mut joint_states: Vec<_> = sim
            .world()
            .iter_entities()
            .filter_map(|entity| {
                let joint = entity.get::<Joint>()?;
                if joint.kind == JointKind::Fixed {
                    return None;
                }
                let link = sim.world().get::<Link>(joint.child_link)?;
                Some(JointObservation {
                    link_name: link.name.clone(),
                    position_rad: sim.named_joint_position(&link.name),
                    velocity_rad_s: sim.named_joint_velocity(&link.name),
                })
            })
            .collect();
        joint_states.sort_unstable_by(|a, b| a.link_name.cmp(&b.link_name));
        let mut estimate = Self {
            com_world_m,
            com_velocity_world_m_s,
            mass_kg,
            up_world: Vec3::ZERO,
            forward_unit: Vec3::ZERO,
            lateral_unit: Vec3::ZERO,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            angular_velocity_world_rad_s,
            roll_rate_rad_s: 0.0,
            foot_positions_world_m,
            foot_normal_loads_n,
            joint_states,
            base_rotation_world,
            go2: base_link == "base",
            load_saturations: 0,
        };
        estimate.derive_orientation();
        estimate.validate()?;
        Ok(estimate)
    }

    fn derive_orientation(&mut self) {
        self.up_world = self.base_rotation_world * Vec3::Z;
        self.forward_unit = self.base_rotation_world * Vec3::X;
        self.lateral_unit = Vec3::new(self.forward_unit.x, 0.0, self.forward_unit.z)
            .normalize_or_zero()
            .cross(Vec3::Y);
        self.roll_rad = if self.go2 {
            self.up_world.dot(self.lateral_unit).atan2(self.up_world.y)
        } else {
            self.up_world.z.atan2(self.up_world.y)
        };
        self.pitch_rad = self.up_world.x.atan2(self.up_world.y);
        self.roll_rate_rad_s = if self.go2 {
            self.angular_velocity_world_rad_s.dot(self.forward_unit)
        } else {
            self.angular_velocity_world_rad_s.x
        };
    }

    /// Projects only orientation from the original captured world angular rate.
    ///
    /// The caller validates capture age. Zero age or zero angular rotation
    /// returns the original payload without quaternion arithmetic. Other
    /// channels, including captured roll rate, retain their original bits.
    pub(super) fn orientation_projected(&self, age_ticks: u64) -> Cow<'_, Self> {
        if age_ticks == 0 {
            return Cow::Borrowed(self);
        }
        let rotation_vector_rad = self.angular_velocity_world_rad_s * (age_ticks as f64 * 1.0e-9);
        if rotation_vector_rad == Vec3::ZERO {
            return Cow::Borrowed(self);
        }
        let mut projected = self.clone();
        // A world-frame angular increment acts on the left of the body pose.
        projected.base_rotation_world =
            Quat::from_scaled_axis(rotation_vector_rad) * self.base_rotation_world;
        projected.derive_orientation();
        // Reusing the established pose convention must not project rate channels.
        projected.roll_rate_rad_s = self.roll_rate_rad_s;
        Cow::Owned(projected)
    }

    fn perturb(&mut self, bounds: NoiseBounds, random: &KeyedRandom, sequence: u64) {
        let draw = |slot| random.sample_signed_f64(ESTIMATE_STREAM.0, sequence, slot);
        // Zero bounds do not perform extra orientation arithmetic in the ideal path.
        if bounds.attitude_bound_rad != 0.0 {
            self.base_rotation_world = Quat::from_rotation_x(draw(0) * bounds.attitude_bound_rad)
                * Quat::from_rotation_z(draw(1) * bounds.attitude_bound_rad)
                * self.base_rotation_world;
        }
        if bounds.angular_rate_bound_rad_s != 0.0 {
            self.angular_velocity_world_rad_s +=
                Vec3::new(draw(10), draw(11), draw(12)) * bounds.angular_rate_bound_rad_s;
        }
        if bounds.position_bound_m != 0.0 {
            self.com_world_m += Vec3::new(draw(20), draw(21), draw(22)) * bounds.position_bound_m;
            for (index, foot) in self.foot_positions_world_m.iter_mut().enumerate() {
                let slot = 100 + index as u64 * 6;
                *foot +=
                    Vec3::new(draw(slot), draw(slot + 1), draw(slot + 2)) * bounds.position_bound_m;
            }
        }
        if bounds.velocity_bound_m_s != 0.0 {
            self.com_velocity_world_m_s +=
                Vec3::new(draw(30), draw(31), draw(32)) * bounds.velocity_bound_m_s;
        }
        if self.go2 && bounds.normal_load_bound_n != 0.0 {
            for (index, load) in self.foot_normal_loads_n.iter_mut().enumerate() {
                let noisy = *load + draw(103 + index as u64 * 6) * bounds.normal_load_bound_n;
                if noisy < 0.0 {
                    self.load_saturations += 1;
                }
                *load = noisy.max(0.0);
            }
        }
        if bounds.attitude_bound_rad != 0.0 || bounds.angular_rate_bound_rad_s != 0.0 {
            self.derive_orientation();
        }
    }

    fn validate(&self) -> ObservationResult<()> {
        if !self.com_world_m.is_finite()
            || !self.com_velocity_world_m_s.is_finite()
            || !self.angular_velocity_world_rad_s.is_finite()
            || !self.base_rotation_world.is_finite()
            || !self.up_world.is_finite()
            || !self.forward_unit.is_finite()
            || !self.lateral_unit.is_finite()
            || ![
                self.mass_kg,
                self.roll_rad,
                self.pitch_rad,
                self.roll_rate_rad_s,
            ]
            .iter()
            .all(|value| value.is_finite())
            || self.mass_kg <= 0.0
            || self
                .foot_positions_world_m
                .iter()
                .any(|foot| !foot.is_finite())
            || self.foot_normal_loads_n.len() != self.foot_positions_world_m.len()
            || self
                .foot_normal_loads_n
                .iter()
                .any(|load| !load.is_finite() || *load < 0.0)
            || self.joint_states.iter().any(|joint| {
                joint.position_rad.is_some_and(|value| !value.is_finite())
                    || joint.velocity_rad_s.is_some_and(|value| !value.is_finite())
            })
        {
            return Err(io::Error::other("the captured synthetic estimate is invalid").into());
        }
        Ok(())
    }

    /// Serializes the estimate channels without recapturing or redrawing noise.
    pub(super) fn diagnostic_fields(&self) -> Value {
        json!({
            "com_world_m":self.com_world_m.to_array(),
            "com_velocity_world_m_s":self.com_velocity_world_m_s.to_array(),
            "mass_kg":self.mass_kg,"base_rotation_world_xyzw":self.base_rotation_world.to_array(),
            "up_world":self.up_world.to_array(),"forward_unit":self.forward_unit.to_array(),
            "lateral_unit":self.lateral_unit.to_array(),"roll_rad":self.roll_rad,
            "pitch_rad":self.pitch_rad,"roll_rate_rad_s":self.roll_rate_rad_s,
            "angular_velocity_world_rad_s":self.angular_velocity_world_rad_s.to_array(),
            "foot_positions_world_m":self.foot_positions_world_m.iter().map(|p|p.to_array()).collect::<Vec<_>>(),
            "foot_normal_loads_n":self.foot_normal_loads_n,
            "joint_states":self.joint_states.iter().map(|joint|json!({"link":joint.link_name,
                "position_rad":joint.position_rad,"velocity_rad_s":joint.velocity_rad_s})).collect::<Vec<_>>()
        })
    }

    pub(super) fn replay_words(&self) -> Vec<u64> {
        let mut words = vec![u64::from(self.go2), self.load_saturations];
        words.extend(
            self.com_world_m
                .to_array()
                .into_iter()
                .chain(self.com_velocity_world_m_s.to_array())
                .chain([self.mass_kg])
                .chain(self.base_rotation_world.to_array())
                .chain(self.up_world.to_array())
                .chain(self.forward_unit.to_array())
                .chain(self.lateral_unit.to_array())
                .chain([self.roll_rad, self.pitch_rad, self.roll_rate_rad_s])
                .chain(self.angular_velocity_world_rad_s.to_array())
                .map(f64::to_bits),
        );
        words.push(self.foot_positions_world_m.len() as u64);
        for (foot, load) in self
            .foot_positions_world_m
            .iter()
            .zip(&self.foot_normal_loads_n)
        {
            words.extend(foot.to_array().into_iter().chain([*load]).map(f64::to_bits));
        }
        words.push(self.joint_states.len() as u64);
        for joint in &self.joint_states {
            // Joint names are a static sorted model contract; preserve them too.
            words.push(joint.link_name.len() as u64);
            words.extend(joint.link_name.bytes().map(u64::from));
            for value in [joint.position_rad, joint.velocity_rad_s] {
                words.push(u64::from(value.is_some()));
                if let Some(value) = value {
                    words.push(value.to_bits());
                }
            }
        }
        words
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub(super) struct NoiseBounds {
    attitude_bound_rad: f64,
    angular_rate_bound_rad_s: f64,
    position_bound_m: f64,
    velocity_bound_m_s: f64,
    normal_load_bound_n: f64,
}

impl NoiseBounds {
    fn values(self) -> [f64; 5] {
        [
            self.attitude_bound_rad,
            self.angular_rate_bound_rad_s,
            self.position_bound_m,
            self.velocity_bound_m_s,
            self.normal_load_bound_n,
        ]
    }

    fn configuration(self) -> Value {
        json!({"distribution":"independent bounded signed-uniform synthetic estimate errors",
            "attitude_world_x_z_bound_rad":self.attitude_bound_rad,
            "angular_velocity_axis_bound_rad_s":self.angular_rate_bound_rad_s,
            "com_and_foot_position_axis_bound_m":self.position_bound_m,
            "com_velocity_axis_bound_m_s":self.velocity_bound_m_s,
            "go2_foot_normal_load_bound_n":self.normal_load_bound_n,
            "load_saturation":"zero lower bound, saturation counts recorded",
            "identified_hardware_parameters":false})
    }
}

const BOUNDED_ERROR: NoiseBounds = NoiseBounds {
    attitude_bound_rad: 0.01,
    angular_rate_bound_rad_s: 0.02,
    position_bound_m: 0.005,
    velocity_bound_m_s: 0.02,
    normal_load_bound_n: 2.0,
};

#[derive(Clone, Copy, Debug)]
pub(super) struct Profile {
    pub(super) name: &'static str,
    pub(super) sample_period_ticks: u64,
    pub(super) latency_ticks: u64,
    pub(super) zero_input: bool,
    noise: NoiseBounds,
}

impl Profile {
    pub(super) fn ideal(physics_dt_ticks: u64) -> Self {
        Self {
            name: "ideal_reference",
            sample_period_ticks: physics_dt_ticks,
            latency_ticks: 0,
            zero_input: false,
            noise: NoiseBounds::default(),
        }
    }

    fn validate(self, physics_dt_ticks: u64) -> ObservationResult<usize> {
        if physics_dt_ticks == 0
            || self.sample_period_ticks == 0
            || !self.sample_period_ticks.is_multiple_of(physics_dt_ticks)
            || self.sample_period_ticks > 1_000_000_000
            || self.latency_ticks > 1_000_000_000
            || self
                .noise
                .values()
                .iter()
                .any(|value| !value.is_finite() || *value < 0.0)
        {
            return Err(
                io::Error::other("invalid synthetic estimate timing or noise bounds").into(),
            );
        }
        // Keep the latest arrived frame in addition to all newer pending frames.
        let in_flight = self.latency_ticks.div_ceil(self.sample_period_ticks);
        usize::try_from(in_flight + 1)
            .map_err(|_| io::Error::other("synthetic estimate retention exceeds usize").into())
    }

    pub(super) fn configuration(self) -> Value {
        json!({"profile":self.name,"sample_period_ticks":self.sample_period_ticks,
            "latency_ticks":self.latency_ticks,"phase_offset_ticks":0,
            "observation_source":"synthetic simulator-derived state estimate, not raw IMU fusion",
            "noise":self.noise.configuration(),"zero_input":self.zero_input,
            "inner_position_pd_feedback":"ideal instantaneous simulator joint state",
            "hardware_validation":false})
    }
}

pub(super) fn profiles(filter: Option<&str>) -> ObservationResult<Vec<Profile>> {
    let ideal = Profile::ideal(1_000_000);
    let all = [
        ideal,
        Profile {
            name: "sample_500hz",
            sample_period_ticks: 2_000_000,
            ..ideal
        },
        Profile {
            name: "sample_250hz",
            sample_period_ticks: 4_000_000,
            ..ideal
        },
        Profile {
            name: "delay_5ms",
            latency_ticks: 5_000_000,
            ..ideal
        },
        Profile {
            name: "delay_20ms_limit",
            latency_ticks: 20_000_000,
            ..ideal
        },
        Profile {
            name: "bounded_error",
            noise: BOUNDED_ERROR,
            ..ideal
        },
        Profile {
            name: "combined",
            sample_period_ticks: 4_000_000,
            latency_ticks: 5_000_000,
            noise: BOUNDED_ERROR,
            ..ideal
        },
        Profile {
            name: "combined_zero_input",
            sample_period_ticks: 4_000_000,
            latency_ticks: 5_000_000,
            noise: BOUNDED_ERROR,
            zero_input: true,
        },
    ];
    let selected: Vec<_> = all
        .into_iter()
        .filter(|profile| filter.is_none_or(|name| name == profile.name))
        .collect();
    if selected.is_empty() {
        return Err(io::Error::other("unknown --observation-profile").into());
    }
    Ok(selected)
}

/// Bounded capture/arrival pipeline; the same queue spans settlement and recovery.
#[derive(Debug)]
pub(super) struct Pipeline {
    profile: Profile,
    bus: InMemoryDataBus,
    retained: VecDeque<Frame<Observation>>,
    capacity: usize,
    random: KeyedRandom,
    next_capture_ticks: u64,
    sequence: u64,
    last_consumed_sequence: Option<u64>,
    last_visible: Option<Frame<Observation>>,
    last_consumer_ticks: Option<u64>,
    captures: u64,
    decisions: u64,
    held_decisions: u64,
    no_observation_decisions: u64,
    max_age_ticks: u64,
    load_saturations: u64,
}

impl Pipeline {
    pub(super) fn new(profile: Profile, sim: &UrdfSceneSim) -> ObservationResult<Self> {
        let root_seed = sim
            .world()
            .get_resource::<WorldRandom>()
            .ok_or_else(|| io::Error::other("the estimate pipeline requires WorldRandom"))?
            .seed();
        Self::with_seed(profile, sim.fixed_delta().ticks(), root_seed)
    }

    fn with_seed(profile: Profile, dt_ticks: u64, root_seed: u64) -> ObservationResult<Self> {
        let capacity = profile.validate(dt_ticks)?;
        Ok(Self {
            profile,
            bus: InMemoryDataBus::with_capacity_per_stream(capacity)?,
            retained: VecDeque::new(),
            capacity,
            random: KeyedRandom::new(root_seed, ESTIMATE_NOISE_DOMAIN),
            next_capture_ticks: 0,
            sequence: 0,
            last_consumed_sequence: None,
            last_visible: None,
            last_consumer_ticks: None,
            captures: 0,
            decisions: 0,
            held_decisions: 0,
            no_observation_decisions: 0,
            max_age_ticks: 0,
            load_saturations: 0,
        })
    }

    pub(super) fn capture_if_due(
        &mut self,
        sim: &UrdfSceneSim,
        base_link: &str,
        feet: &[&str],
    ) -> ObservationResult<()> {
        let now = sim.sim_time();
        if now.ticks() < self.next_capture_ticks {
            return Ok(());
        }
        if now.ticks() != self.next_capture_ticks {
            return Err(io::Error::other("a synthetic estimate capture tick was skipped").into());
        }
        let source = sim
            .world()
            .iter_entities()
            .find(|entity| {
                entity
                    .get::<Link>()
                    .is_some_and(|link| link.name == base_link)
            })
            .ok_or_else(|| io::Error::other("synthetic estimate source is missing"))?
            .id();
        let estimate = Observation::capture(sim, base_link, feet)?;
        self.publish(now, source, estimate)
    }

    fn publish(
        &mut self,
        now: SimTime,
        source: Entity,
        mut estimate: Observation,
    ) -> ObservationResult<()> {
        let sequence = self
            .sequence
            .checked_add(1)
            .ok_or_else(|| io::Error::other("synthetic estimate sequence overflow"))?;
        let next_capture_ticks = now
            .ticks()
            .checked_add(self.profile.sample_period_ticks)
            .ok_or_else(|| io::Error::other("synthetic estimate schedule overflow"))?;
        now.ticks()
            .checked_add(self.profile.latency_ticks)
            .ok_or_else(|| io::Error::other("synthetic estimate arrival time overflow"))?;
        estimate.perturb(self.profile.noise, &self.random, sequence);
        estimate.validate()?;
        self.sequence = sequence;
        self.next_capture_ticks = next_capture_ticks;
        self.load_saturations += estimate.load_saturations;
        let frame = Frame::new(ESTIMATE_STREAM, source, self.sequence, now, estimate)
            .with_latency(SimDuration::from_ticks(self.profile.latency_ticks));
        if self.retained.len() == self.capacity {
            self.retained.pop_front();
        }
        self.retained.push_back(frame.clone());
        self.bus.publish(frame);
        self.captures += 1;
        Ok(())
    }

    pub(super) fn consume(
        &mut self,
        now: SimTime,
    ) -> ObservationResult<Option<Frame<Observation>>> {
        if self
            .last_consumer_ticks
            .is_some_and(|ticks| now.ticks() < ticks)
        {
            return Err(
                io::Error::other("synthetic estimate consumer time moved backwards").into(),
            );
        }
        let frame = self
            .bus
            .latest_available::<Observation>(ESTIMATE_STREAM, now);
        self.decisions += 1;
        if let Some(frame) = &frame {
            if frame.available_time > now || frame.capture_time > now {
                return Err(
                    io::Error::other("a synthetic estimate arrived from the future").into(),
                );
            }
            if self.last_consumed_sequence == Some(frame.sequence) {
                self.held_decisions += 1;
            }
            self.max_age_ticks = self
                .max_age_ticks
                .max(now.ticks() - frame.capture_time.ticks());
            self.last_consumed_sequence = Some(frame.sequence);
        } else {
            self.no_observation_decisions += 1;
        }
        self.last_visible = frame.clone();
        self.last_consumer_ticks = Some(now.ticks());
        Ok(frame)
    }

    pub(super) fn configuration(&self) -> Value {
        let mut value = self.profile.configuration();
        value["world_seed"] = json!(self.random.root_seed());
        value["noise_domain"] = json!(self.random.domain());
        value["stream_id"] = json!(ESTIMATE_STREAM.0);
        value["retained_frames_capacity"] = json!(self.capacity);
        value
    }

    pub(super) fn telemetry(&self) -> Value {
        json!({"capture_sequence":self.last_visible.as_ref().map(|frame| frame.sequence),
            "capture_ticks":self.last_visible.as_ref().map(|frame| frame.capture_time.ticks()),
            "available_ticks":self.last_visible.as_ref().map(|frame| frame.available_time.ticks()),
            "consumer_ticks":self.last_consumer_ticks,
            "observation_age_ticks":self.last_visible.as_ref().map(|frame| self.last_consumer_ticks.unwrap_or(0) - frame.capture_time.ticks()),
            "captures":self.captures,"decisions":self.decisions,"held_decisions":self.held_decisions,
            "no_observation_decisions":self.no_observation_decisions,"max_observation_age_ticks":self.max_age_ticks,
            "load_saturations":self.load_saturations})
    }

    pub(super) fn replay_words(&self) -> Vec<u64> {
        let mut words = vec![
            self.random.root_seed(),
            self.random.domain(),
            self.profile.sample_period_ticks,
            self.profile.latency_ticks,
            self.next_capture_ticks,
            self.sequence,
            self.captures,
            self.decisions,
            self.held_decisions,
            self.no_observation_decisions,
            self.max_age_ticks,
            self.load_saturations,
            self.last_consumed_sequence.unwrap_or(0),
            self.retained.len() as u64,
        ];
        words.extend([
            u64::from(self.last_consumer_ticks.is_some()),
            self.last_consumer_ticks.unwrap_or(0),
        ]);
        words.extend(self.profile.noise.values().map(f64::to_bits));
        for frame in &self.retained {
            frame_words(&mut words, frame);
        }
        words.push(u64::from(self.last_visible.is_some()));
        if let Some(frame) = &self.last_visible {
            frame_words(&mut words, frame);
        }
        words
    }
}

fn frame_words(words: &mut Vec<u64>, frame: &Frame<Observation>) {
    words.extend([
        u64::from(frame.entity.index()),
        frame.sequence,
        frame.capture_time.ticks(),
        frame.available_time.ticks(),
    ]);
    let payload = frame.payload.replay_words();
    words.push(payload.len() as u64);
    words.extend(payload);
}

#[cfg(test)]
mod tests {
    use super::super::feedback::Policy;
    use super::*;
    use std::path::Path;

    fn pure_orientation_fixture(go2: bool, rotation: Quat, gyro: Vec3) -> Observation {
        let mut observation = Observation {
            com_world_m: Vec3::new(0.21, 0.78, -0.0),
            com_velocity_world_m_s: Vec3::new(-0.02, -0.0, 0.03),
            mass_kg: 33.4,
            up_world: Vec3::ZERO,
            forward_unit: Vec3::ZERO,
            lateral_unit: Vec3::ZERO,
            roll_rad: 0.0,
            pitch_rad: 0.0,
            angular_velocity_world_rad_s: gyro,
            roll_rate_rad_s: 0.0,
            foot_positions_world_m: vec![Vec3::new(0.1, -0.0, -0.2), Vec3::new(0.1, 0.0, 0.2)],
            foot_normal_loads_n: vec![12.0, 18.0],
            joint_states: vec![JointObservation {
                link_name: "left_hip_roll_link".to_owned(),
                position_rad: Some(-0.0),
                velocity_rad_s: None,
            }],
            base_rotation_world: rotation,
            go2,
            load_saturations: 3,
        };
        observation.derive_orientation();
        observation
    }

    fn pure_frame(
        observation: Observation,
        capture_ticks: u64,
        latency_ticks: u64,
    ) -> Frame<Observation> {
        let mut world = rne_ecs::World::new();
        Frame::new(
            ESTIMATE_STREAM,
            world.spawn_empty().id(),
            7,
            SimTime::from_ticks(capture_ticks),
            observation,
        )
        .with_latency(SimDuration::from_ticks(latency_ticks))
    }

    fn upright_g1(gyro: Vec3) -> Observation {
        pure_orientation_fixture(
            false,
            Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
            gyro,
        )
    }

    #[test]
    fn orientation_feedback_uses_capture_age_instead_of_arrival_age() {
        let frame = pure_frame(upright_g1(Vec3::X * 2.0), 1_000_000, 5_000_000);
        let effective = Policy::OrientationKnownAge
            .effective(Some(&frame), SimTime::from_ticks(7_000_000))
            .expect("arrived frame")
            .expect("effective orientation");
        // Six milliseconds since capture, although arrival was one millisecond ago.
        assert!((effective.roll_rad - 0.012).abs() < 1.0e-12);
        assert!((effective.roll_rad - 0.002).abs() > 0.009);
        assert_eq!(frame.sim_time.ticks(), 6_000_000);
    }

    #[test]
    fn missing_then_first_arrived_frame_has_no_truth_fallback() {
        for policy in [Policy::RawReference, Policy::OrientationKnownAge] {
            assert!(policy
                .effective(None, SimTime::ZERO)
                .expect("missing observations")
                .is_none());
        }
        let frame = pure_frame(upright_g1(Vec3::X * 2.0), 0, 5_000_000);
        assert!(Policy::OrientationKnownAge
            .effective(Some(&frame), SimTime::from_ticks(4_000_000))
            .is_err());
        let first = Policy::OrientationKnownAge
            .effective(Some(&frame), SimTime::from_ticks(5_000_000))
            .expect("first arrived frame")
            .expect("effective first frame");
        assert!((first.roll_rad - 0.01).abs() < 1.0e-12);
    }

    #[test]
    fn orientation_feedback_rejects_future_times_and_unsupported_age() {
        let future_capture = pure_frame(upright_g1(Vec3::X), u64::MAX, 0);
        assert!(Policy::OrientationKnownAge
            .effective(Some(&future_capture), SimTime::ZERO)
            .is_err());
        let future_arrival = pure_frame(upright_g1(Vec3::X), 0, 5_000_000);
        assert!(Policy::OrientationKnownAge
            .effective(Some(&future_arrival), SimTime::from_ticks(4_999_999))
            .is_err());
        assert!(Policy::OrientationKnownAge
            .effective(Some(&future_arrival), SimTime::from_ticks(20_000_000))
            .is_ok());
        assert!(Policy::OrientationKnownAge
            .effective(Some(&future_arrival), SimTime::from_ticks(20_000_001))
            .is_err());
    }

    #[test]
    fn raw_zero_age_and_zero_angular_rotation_borrow_exact_original_words() {
        let moving = pure_frame(upright_g1(Vec3::new(1.2, -0.4, 3.1)), 0, 0);
        let stationary = pure_frame(upright_g1(Vec3::new(0.0, -0.0, 0.0)), 0, 0);
        for (policy, frame, now) in [
            (Policy::RawReference, &moving, 20_000_000),
            (Policy::OrientationKnownAge, &moving, 0),
            (Policy::OrientationKnownAge, &stationary, 20_000_000),
        ] {
            let effective = policy
                .effective(Some(frame), SimTime::from_ticks(now))
                .expect("identity feedback")
                .expect("original observation");
            match effective {
                Cow::Borrowed(payload) => {
                    assert!(std::ptr::eq(payload, &frame.payload));
                    assert_eq!(payload.replay_words(), frame.payload.replay_words());
                }
                Cow::Owned(_) => panic!("identity paths must not clone or recompute the payload"),
            }
        }
    }

    #[test]
    fn world_gyro_projection_uses_noncommuting_left_rotation() {
        let raw = upright_g1(Vec3::Z * 4.0);
        let projected = raw.orientation_projected(20_000_000);
        let delta = Quat::from_rotation_z(0.08);
        let expected_up = delta * (raw.base_rotation_world * Vec3::Z);
        let body_frame_up = raw.base_rotation_world * (delta * Vec3::Z);
        assert!((projected.up_world - expected_up).length() < 1.0e-12);
        assert!((projected.up_world - body_frame_up).length() > 0.07);
        assert!((projected.pitch_rad + 0.08).abs() < 1.0e-12);
    }

    #[test]
    fn projected_quaternion_preserves_xyzw_and_y_up_orientation_conventions() {
        let raw = upright_g1(Vec3::X * 5.0);
        let projected = raw.orientation_projected(20_000_000);
        let half_angle = (-std::f64::consts::FRAC_PI_2 + 0.1) * 0.5;
        let expected_xyzw = [half_angle.sin(), 0.0, 0.0, half_angle.cos()];
        for (actual, expected) in projected
            .base_rotation_world
            .to_array()
            .into_iter()
            .zip(expected_xyzw)
        {
            assert!((actual - expected).abs() < 1.0e-12);
        }
        assert!((projected.roll_rad - 0.1).abs() < 1.0e-12);
        assert!(projected.pitch_rad.abs() < 1.0e-12);
        assert!(
            (projected.up_world - Vec3::new(0.0, 0.1_f64.cos(), 0.1_f64.sin())).length() < 1.0e-12
        );
        assert!((projected.forward_unit - Vec3::X).length() < 1.0e-12);
        assert!((projected.lateral_unit - Vec3::Z).length() < 1.0e-12);
    }

    #[test]
    fn orientation_projection_preserves_every_non_orientation_channel_word() {
        let raw = pure_orientation_fixture(
            true,
            Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2) * Quat::from_rotation_z(0.37),
            Vec3::new(1.7, -0.2, 3.1),
        );
        let mut restored = raw.orientation_projected(7_000_000).into_owned();
        assert_ne!(restored.base_rotation_world, raw.base_rotation_world);
        restored.base_rotation_world = raw.base_rotation_world;
        restored.up_world = raw.up_world;
        restored.forward_unit = raw.forward_unit;
        restored.lateral_unit = raw.lateral_unit;
        restored.roll_rad = raw.roll_rad;
        restored.pitch_rad = raw.pitch_rad;
        // This covers gyro, roll rate, signed zeros, optional joints and metadata.
        assert_eq!(restored.replay_words(), raw.replay_words());
    }

    #[test]
    fn held_frame_projection_recomputes_from_original_without_compounding() {
        let frame = pure_frame(upright_g1(Vec3::X * 2.0), 1_000_000, 5_000_000);
        let original_words = frame.payload.replay_words();
        let original_metadata = (
            frame.sim_time,
            frame.capture_time,
            frame.available_time,
            frame.sequence,
            frame.entity,
        );
        let first = Policy::OrientationKnownAge
            .effective(Some(&frame), SimTime::from_ticks(6_000_000))
            .expect("first arrived use")
            .expect("effective first frame");
        let held = Policy::OrientationKnownAge
            .effective(Some(&frame), SimTime::from_ticks(8_000_000))
            .expect("held original sample")
            .expect("effective held frame");
        assert!((first.roll_rad - 0.01).abs() < 1.0e-12);
        assert!((held.roll_rad - 0.014).abs() < 1.0e-12);
        assert!((held.roll_rad - 0.024).abs() > 0.009);
        assert_eq!(frame.payload.replay_words(), original_words);
        assert_eq!(
            (
                frame.sim_time,
                frame.capture_time,
                frame.available_time,
                frame.sequence,
                frame.entity,
            ),
            original_metadata
        );
    }

    fn fixture() -> (UrdfSceneSim, Observation) {
        let sim = UrdfSceneSim::from_scene_path_with_solver_iterations_and_fixed_delta(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("go2.rne.scene.toml"),
            32,
            SimDuration::from_ticks(1_000_000),
        )
        .expect("Go2 fixture");
        let estimate =
            Observation::capture(&sim, "base", &["FL_foot", "FR_foot", "RL_foot", "RR_foot"])
                .expect("captured fixture");
        (sim, estimate)
    }

    fn source(sim: &UrdfSceneSim) -> Entity {
        sim.world()
            .iter_entities()
            .find(|entity| entity.get::<Link>().is_some_and(|link| link.name == "base"))
            .expect("base source")
            .id()
    }

    #[test]
    fn pending_frames_do_not_hide_the_latest_arrived_estimate() {
        let (sim, original) = fixture();
        let mut profile = Profile::ideal(1_000_000);
        profile.latency_ticks = 5_000_000;
        let mut pipeline = Pipeline::new(profile, &sim).expect("pipeline");
        for tick in 0..=9 {
            let now = SimTime::from_ticks(tick * 1_000_000);
            let mut estimate = original.clone();
            estimate.com_world_m.x = tick as f64;
            pipeline
                .publish(now, source(&sim), estimate)
                .expect("publish");
            let arrived = pipeline.consume(now).expect("consume");
            if tick < 5 {
                assert!(
                    arrived.is_none(),
                    "startup must not fall back to pending truth"
                );
            } else {
                let arrived = arrived.expect("an arrived frame survives pending retention");
                assert_eq!(arrived.capture_time.ticks(), (tick - 5) * 1_000_000);
                assert_eq!(arrived.payload.com_world_m.x, (tick - 5) as f64);
            }
        }
        assert_eq!(pipeline.capacity, 6);
        assert_eq!(pipeline.no_observation_decisions, 5);
    }

    #[test]
    fn held_noise_is_not_redrawn_and_the_seed_changes_the_estimate() {
        let (sim, original) = fixture();
        let profile = profiles(Some("bounded_error")).expect("profile")[0];
        let mut first = Pipeline::with_seed(profile, 1_000_000, 2003).expect("pipeline");
        let mut repeat = Pipeline::with_seed(profile, 1_000_000, 2003).expect("pipeline");
        let mut different = Pipeline::with_seed(profile, 1_000_000, 2004).expect("pipeline");
        for pipeline in [&mut first, &mut repeat, &mut different] {
            pipeline
                .publish(SimTime::from_ticks(0), source(&sim), original.clone())
                .expect("publish");
        }
        let received = first
            .consume(SimTime::from_ticks(0))
            .expect("consume")
            .expect("arrived");
        let held = first
            .consume(SimTime::from_ticks(1_000_000))
            .expect("consume")
            .expect("held");
        assert_eq!(received.payload.replay_words(), held.payload.replay_words());
        assert_eq!(first.held_decisions, 1);
        let replay = repeat
            .consume(SimTime::from_ticks(0))
            .expect("consume")
            .expect("arrived");
        let other = different
            .consume(SimTime::from_ticks(0))
            .expect("consume")
            .expect("arrived");
        assert_eq!(
            received.payload.replay_words(),
            replay.payload.replay_words()
        );
        assert_ne!(
            received.payload.replay_words(),
            other.payload.replay_words()
        );
        assert!(received
            .payload
            .foot_normal_loads_n
            .iter()
            .all(|load| *load >= 0.0));
    }

    #[test]
    fn ideal_pipeline_preserves_every_observation_word() {
        let (sim, original) = fixture();
        let mut pipeline =
            Pipeline::new(Profile::ideal(sim.fixed_delta().ticks()), &sim).expect("pipeline");
        pipeline
            .publish(SimTime::from_ticks(0), source(&sim), original.clone())
            .expect("publish");
        let received = pipeline
            .consume(SimTime::from_ticks(0))
            .expect("consume")
            .expect("arrived");
        assert_eq!(original.replay_words(), received.payload.replay_words());
    }

    #[test]
    fn invalid_timing_and_noise_are_rejected_before_capture() {
        let ideal = Profile::ideal(1_000_000);
        assert!(ideal.validate(0).is_err());
        for profile in [
            Profile {
                sample_period_ticks: 0,
                ..ideal
            },
            Profile {
                sample_period_ticks: 1_500_000,
                ..ideal
            },
            Profile {
                latency_ticks: u64::MAX,
                ..ideal
            },
            Profile {
                noise: NoiseBounds {
                    position_bound_m: -0.1,
                    ..NoiseBounds::default()
                },
                ..ideal
            },
            Profile {
                noise: NoiseBounds {
                    attitude_bound_rad: f64::NAN,
                    ..NoiseBounds::default()
                },
                ..ideal
            },
        ] {
            assert!(profile.validate(1_000_000).is_err());
        }
        assert!(profiles(Some("unknown")).is_err());
        assert_eq!(profiles(None).expect("profiles").len(), 8);
    }

    #[test]
    fn captures_follow_completed_simulation_ticks_and_hold_between_samples() {
        let (mut sim, _) = fixture();
        let profile = profiles(Some("sample_250hz")).expect("profile")[0];
        let mut pipeline = Pipeline::new(profile, &sim).expect("pipeline");
        let feet = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];
        pipeline
            .capture_if_due(&sim, "base", &feet)
            .expect("initial capture");
        let initial = pipeline
            .consume(sim.sim_time())
            .expect("consume")
            .expect("arrived");
        for _ in 0..3 {
            sim.step_joint_position_actuation_targets(&[]);
            pipeline
                .capture_if_due(&sim, "base", &feet)
                .expect("not due");
            let held = pipeline
                .consume(sim.sim_time())
                .expect("consume")
                .expect("held");
            assert_eq!(held.sequence, initial.sequence);
            assert_eq!(held.payload.replay_words(), initial.payload.replay_words());
        }
        sim.step_joint_position_actuation_targets(&[]);
        pipeline
            .capture_if_due(&sim, "base", &feet)
            .expect("second capture");
        let next = pipeline
            .consume(sim.sim_time())
            .expect("consume")
            .expect("arrived");
        assert_eq!(next.sequence, 2);
        assert_eq!(next.capture_time.ticks(), 4_000_000);
        assert_eq!(pipeline.telemetry()["consumer_ticks"], 4_000_000);
        assert_eq!(pipeline.held_decisions, 3);
        // A driver that skips an exact acquisition time must not silently backfill truth.
        sim.set_fixed_delta(SimDuration::from_ticks(5_000_000));
        sim.step_joint_position_actuation_targets(&[]);
        assert!(pipeline.capture_if_due(&sim, "base", &feet).is_err());
    }

    #[test]
    fn noisy_load_saturation_is_counted_at_capture_only() {
        let (sim, mut original) = fixture();
        original.foot_normal_loads_n = vec![0.0; 4];
        let profile = profiles(Some("bounded_error")).expect("profile")[0];
        let mut pipeline = Pipeline::new(profile, &sim).expect("pipeline");
        let expected = (0..4)
            .filter(|index| {
                pipeline
                    .random
                    .sample_signed_f64(ESTIMATE_STREAM.0, 1, 103 + index * 6)
                    < 0.0
            })
            .count() as u64;
        assert!(
            expected > 0,
            "fixture seed must exercise negative load clipping"
        );
        pipeline
            .publish(SimTime::from_ticks(0), source(&sim), original)
            .expect("publish");
        assert_eq!(pipeline.load_saturations, expected);
        pipeline.consume(SimTime::from_ticks(0)).expect("consume");
        pipeline
            .consume(SimTime::from_ticks(1_000_000))
            .expect("held consume");
        assert_eq!(pipeline.load_saturations, expected);
        assert_eq!(
            pipeline.configuration()["world_seed"],
            sim.world().resource::<WorldRandom>().seed()
        );
    }

    #[test]
    fn real_capture_schedule_combines_rate_latency_and_continuous_sample_identity() {
        let (mut sim, _) = fixture();
        let profile = profiles(Some("combined")).expect("profile")[0];
        let mut pipeline = Pipeline::new(profile, &sim).expect("pipeline");
        let feet = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];
        let mut first_words = None;
        for tick in 0..=9 {
            pipeline
                .capture_if_due(&sim, "base", &feet)
                .expect("clocked capture");
            let received = pipeline.consume(sim.sim_time()).expect("clocked consume");
            if tick < 5 {
                assert!(received.is_none());
            } else {
                let frame = received.expect("arrived estimate");
                let expected_capture = if tick < 9 { 0 } else { 4_000_000 };
                assert_eq!(frame.capture_time.ticks(), expected_capture);
                assert_eq!(frame.available_time.ticks(), expected_capture + 5_000_000);
                assert_eq!(frame.sequence, if tick < 9 { 1 } else { 2 });
                assert_eq!(
                    pipeline.telemetry()["observation_age_ticks"],
                    tick * 1_000_000 - expected_capture
                );
                if tick == 5 {
                    first_words = Some(frame.payload.replay_words());
                }
                if (6..=8).contains(&tick) {
                    assert_eq!(
                        first_words.as_ref().expect("first delivered words"),
                        &frame.payload.replay_words()
                    );
                }
            }
            sim.step_joint_position_actuation_targets(&[]);
        }
        assert_eq!(pipeline.captures, 3);
        assert_eq!(pipeline.no_observation_decisions, 5);
        assert_eq!(pipeline.held_decisions, 3);
        assert_eq!(pipeline.max_age_ticks, 8_000_000);
        assert!(pipeline.consume(SimTime::from_ticks(0)).is_err());
    }
}
