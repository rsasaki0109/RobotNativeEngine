//! Bounded evaluator-only, all-tick evidence for the fixed G1 observation study.
//!
//! This module never supplies feedback inputs. Contact samples describe the
//! previous completed solver step, while poses and estimate truth are captured
//! at the current consumer tick. JSONL is streamed rather than retained in RAM.

use super::{model, observation::Observation};
use rne_ai::UrdfSceneSim;
use rne_math::Vec3;
use rne_physics::{RigidBody, RigidBodyType};
use rne_robot::Link;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::{BTreeMap, VecDeque},
    error::Error,
    fs::File,
    io::{self, BufReader, BufWriter, Read, Write},
    path::Path,
};

type DiagnosticResult<T> = Result<T, Box<dyn Error>>;
pub(super) const PROFILES: [&str; 4] = [
    "ideal_reference",
    "delay_5ms",
    "delay_20ms_limit",
    "bounded_error",
];
pub(super) const DT_TICKS: u64 = 1_000_000;
const SETTLE_TICKS: u64 = 2_000_000_000;
pub(super) const MAX_ROWS: usize = 8200;
const MAX_ROW_BYTES: usize = 64 * 1024;
const MAX_DECISION_WORDS_PER_ROW: usize = 4096;
const MAX_CONTACT_SAMPLES_PER_FOOT: usize = 64;
const TRUTH_HISTORY_ROWS: usize = 21;
const FEET: [&str; 2] = ["left_ankle_roll_link", "right_ankle_roll_link"];
type GroundEvent = (String, u32, u32, Vec3, f64);

fn sort_foot_samples(samples: &mut [(Vec3, f64)]) {
    samples.sort_unstable_by(|(a, force_a), (b, force_b)| {
        a.x.total_cmp(&b.x)
            .then(a.y.total_cmp(&b.y))
            .then(a.z.total_cmp(&b.z))
            .then(force_a.total_cmp(force_b))
    });
}

fn aggregate_ground_events(mut events: Vec<GroundEvent>) -> BTreeMap<String, f64> {
    events.sort_unstable_by(|a, b| {
        a.0.cmp(&b.0)
            .then(a.1.cmp(&b.1))
            .then(a.2.cmp(&b.2))
            .then(a.3.x.total_cmp(&b.3.x))
            .then(a.3.y.total_cmp(&b.3.y))
            .then(a.3.z.total_cmp(&b.3.z))
            .then(a.4.total_cmp(&b.4))
    });
    let mut impulses = BTreeMap::<String, f64>::new();
    for (name, _, _, _, impulse) in events {
        *impulses.entry(name).or_default() += impulse;
    }
    impulses
}

/// Selects only the predeclared four profiles, preserving their declared order.
pub(super) fn profile_names(filter: Option<&str>) -> DiagnosticResult<Vec<&'static str>> {
    match filter {
        None => Ok(PROFILES.to_vec()),
        Some(name) => PROFILES
            .into_iter()
            .find(|profile| *profile == name)
            .map(|name| vec![name])
            .ok_or_else(|| io::Error::other("unknown G1 diagnosis --observation-profile").into()),
    }
}

/// Selects the fixed zero/kick doses; this does not change the pipeline profile.
pub(super) fn inputs(filter: Option<&str>) -> DiagnosticResult<Vec<(&'static str, f64)>> {
    match filter {
        None => Ok(vec![("zero", 0.0), ("kick", 40.0)]),
        Some("zero") => Ok(vec![("zero", 0.0)]),
        Some("kick") => Ok(vec![("kick", 40.0)]),
        _ => Err(io::Error::other("--diagnosis-input must be zero or kick").into()),
    }
}

/// Captures evaluator truth at the same pre-step tick as the feedback decision.
pub(super) fn evaluator_snapshot(sim: &UrdfSceneSim) -> DiagnosticResult<Value> {
    let observation = Observation::capture(sim, "pelvis", &FEET)?;
    if observation.joint_states.len() != 23 || observation.foot_positions_world_m.len() != 2 {
        return Err(
            io::Error::other("G1 diagnosis requires exactly 23 joints and two feet").into(),
        );
    }
    let pose = sim
        .named_transform("pelvis")
        .ok_or_else(|| io::Error::other("missing diagnosis pelvis"))?;
    let mut foot_contacts = Vec::with_capacity(2);
    for name in FEET {
        let mut samples = sim.named_body_contact_loads(name);
        if samples.len() > MAX_CONTACT_SAMPLES_PER_FOOT
            || samples
                .iter()
                .any(|(point, force_n)| !point.is_finite() || !force_n.is_finite())
        {
            return Err(io::Error::other(
                "G1 diagnostic contact samples exceed the declared bound or are invalid",
            )
            .into());
        }
        sort_foot_samples(&mut samples);
        foot_contacts.push(json!({"link":name,"samples":samples.iter().map(|(point,force_n)|
            json!({"point_world_m":point.to_array(),"normal_force_n":force_n})).collect::<Vec<_>>() }));
    }
    let ground_indices: Vec<_> = sim
        .world()
        .iter_entities()
        .filter_map(|entity| {
            let body = entity.get::<RigidBody>()?;
            (body.body_type == RigidBodyType::Fixed && entity.get::<Link>().is_none())
                .then_some(entity.id().index())
        })
        .collect();
    if ground_indices.len() != 1 {
        return Err(io::Error::other("G1 diagnosis requires exactly one ground body").into());
    }
    let mut ground_events = Vec::new();
    for contact in sim.physics_contact_events()? {
        if !contact.impulse.is_finite() || !contact.normal.is_finite() {
            return Err(io::Error::other("non-finite G1 diagnostic contact impulse").into());
        }
        if contact.impulse <= 1.0e-8 {
            continue;
        }
        let other = if contact.entity_a.index() == ground_indices[0] {
            Some(contact.entity_b)
        } else if contact.entity_b.index() == ground_indices[0] {
            Some(contact.entity_a)
        } else {
            None
        };
        if let Some(link) = other.and_then(|entity| sim.world().get::<Link>(entity)) {
            let a = contact.entity_a.index();
            let b = contact.entity_b.index();
            let normal = if a <= b {
                contact.normal
            } else {
                -contact.normal
            };
            ground_events.push((
                link.name.clone(),
                a.min(b),
                a.max(b),
                normal,
                f64::from(contact.impulse),
            ));
        }
    }
    let ground_impulses = aggregate_ground_events(ground_events);
    let (fixed_position_error_m, fixed_rotation_error_rad) = model::fixed_joint_error(sim)?;
    Ok(json!({
        "scope":"evaluator-only; never a feedback input",
        "consumer_ticks":sim.sim_time().ticks(),"observation":observation.diagnostic_fields(),
        "pelvis_position_world_m":pose.translation.to_array(),
        "fixed_position_error_m":fixed_position_error_m,"fixed_rotation_error_rad":fixed_rotation_error_rad,
        "contact_solution_available":sim.sim_time().ticks() > 0,
        "contact_solution_completed_ticks":(sim.sim_time().ticks() > 0).then_some(sim.sim_time().ticks()),
        "contact_solution_start_ticks":sim.sim_time().ticks().checked_sub(DT_TICKS),
        "contact_scope":"all-counterpart solved positive-normal foot body-contact samples from the previous completed step; not a ground-only support polygon",
        "foot_body_contact_samples":foot_contacts,
        "ground_pair_scope":"separate previous-step ground-pair impulses above 1e-8 N.s; summed after canonical link/pair/normal/impulse sorting",
        "ground_pair_positive_impulse_ns":ground_impulses
    }))
}

/// Encodes sorted JSON decision fields, including literal IEEE f64 bits.
pub(super) fn decision_words(value: &Value) -> Vec<u64> {
    fn append(value: &Value, words: &mut Vec<u64>) {
        match value {
            Value::Null => words.push(0),
            Value::Bool(value) => words.extend([1, u64::from(*value)]),
            Value::Number(value) => {
                if let Some(value) = value.as_u64() {
                    words.extend([2, value]);
                } else if let Some(value) = value.as_i64() {
                    words.extend([3, value as u64]);
                } else {
                    words.extend([4, value.as_f64().expect("JSON number").to_bits()]);
                }
            }
            Value::String(value) => {
                words.extend([5, value.len() as u64]);
                words.extend(value.as_bytes().chunks(8).map(|chunk| {
                    let mut bytes = [0; 8];
                    bytes[..chunk.len()].copy_from_slice(chunk);
                    u64::from_le_bytes(bytes)
                }));
            }
            Value::Array(values) => {
                words.extend([6, values.len() as u64]);
                for value in values {
                    append(value, words);
                }
            }
            Value::Object(values) => {
                words.extend([7, values.len() as u64]);
                let mut entries: Vec<_> = values.iter().collect();
                entries.sort_unstable_by_key(|(key, _)| *key);
                for (key, value) in entries {
                    append(&Value::String(key.clone()), words);
                    append(value, words);
                }
            }
        }
    }
    let mut words = Vec::new();
    append(value, &mut words);
    words
}

#[derive(Debug)]
pub(super) struct JsonlWriter {
    output: BufWriter<File>,
    hash: Sha256,
    rows: usize,
    bytes: usize,
    truth_history: VecDeque<(u64, Value)>,
}

impl JsonlWriter {
    /// Opens one streamed artifact; the study retains no all-tick JSON array.
    pub(super) fn new(path: &Path) -> DiagnosticResult<Self> {
        Ok(Self {
            output: BufWriter::new(File::create(path)?),
            hash: Sha256::new(),
            rows: 0,
            bytes: 0,
            truth_history: VecDeque::with_capacity(TRUTH_HISTORY_ROWS),
        })
    }

    fn write(
        &mut self,
        timing: Value,
        estimate: Value,
        truth: Value,
        decision: &Value,
    ) -> DiagnosticResult<()> {
        let ticks = timing["consumer_ticks"]
            .as_u64()
            .ok_or_else(|| io::Error::other("diagnosis consumer ticks are missing"))?;
        if self.rows >= MAX_ROWS || ticks != self.rows as u64 * DT_TICKS {
            return Err(io::Error::other(
                "diagnosis exceeded its 8200-row bound or skipped a control tick",
            )
            .into());
        }
        self.truth_history
            .push_back((ticks, truth["observation"].clone()));
        if self.truth_history.len() > TRUTH_HISTORY_ROWS {
            self.truth_history.pop_front();
        }
        let capture_truth = match timing["capture_ticks"].as_u64() {
            Some(capture_ticks) => Some(
                self.truth_history
                    .iter()
                    .find(|(tick, _)| *tick == capture_ticks)
                    .ok_or_else(|| {
                        io::Error::other(
                            "delivered capture truth is outside the declared history bound",
                        )
                    })?
                    .1
                    .clone(),
            ),
            None => None,
        };
        let row = json!({
            "schema_version":1,"g1_observation_diagnosis":true,
            "decision_index":self.rows,"phase":if ticks < SETTLE_TICKS {"settlement"} else {"recording"},
            "consumer_ticks":ticks,"command_completed_ticks":ticks + DT_TICKS,
            "recording_relative_ticks":ticks.checked_sub(SETTLE_TICKS),
            "observation_timing":timing,"delivered_estimate":estimate,
            "evaluator_current_truth":truth,"evaluator_delivered_capture_truth":capture_truth,
            "comparison_scope":"estimate versus current truth includes staleness and bounded error; estimate versus delivered-capture truth isolates injected estimate error. Both truth channels are evaluator-only.",
            "decision":decision
        });
        let mut bytes = serde_json::to_vec(&row)?;
        bytes.push(b'\n');
        if bytes.len() > MAX_ROW_BYTES {
            return Err(io::Error::other("diagnosis row exceeded its 64 KiB bound").into());
        }
        self.output.write_all(&bytes)?;
        self.hash.update(&bytes);
        self.rows += 1;
        self.bytes += bytes.len();
        Ok(())
    }

    fn finish(mut self) -> DiagnosticResult<Value> {
        self.output.flush()?;
        Ok(
            json!({"rows":self.rows,"bytes":self.bytes,"sha256":format!("{:x}",self.hash.finalize()),
            "maximum_rows":MAX_ROWS,"maximum_row_bytes":MAX_ROW_BYTES,
            "maximum_file_bytes":MAX_ROWS*MAX_ROW_BYTES,
            "maximum_decision_words_per_row":MAX_DECISION_WORDS_PER_ROW,
            "maximum_contact_samples_per_foot":MAX_CONTACT_SAMPLES_PER_FOOT,
            "truth_history_rows":TRUTH_HISTORY_ROWS,
            "phase_origin":"absolute simulation ticks; recording_relative_ticks starts after 2 s settlement",
            "control_tick_scope":"one pre-step decision per 1 ms tick, including settlement; no reduced controller frequency"}),
        )
    }
}

#[derive(Debug, Default)]
pub(super) struct RunDiagnostics {
    pub(super) writer: Option<JsonlWriter>,
    pub(super) decision_words: Vec<Vec<u64>>,
}

impl RunDiagnostics {
    /// Enables streamed truth evidence while collecting exact existing decisions.
    pub(super) fn enabled(path: &Path) -> DiagnosticResult<Self> {
        Ok(Self {
            writer: Some(JsonlWriter::new(path)?),
            decision_words: Vec::new(),
        })
    }

    /// Records one existing decision and, when enabled, its aligned evaluator row.
    pub(super) fn record(
        &mut self,
        timing: Value,
        estimate: Value,
        truth: Option<Value>,
        decision: &Value,
    ) -> DiagnosticResult<()> {
        if self.decision_words.len() >= MAX_ROWS {
            return Err(io::Error::other("diagnosis decision words exceeded 8200 ticks").into());
        }
        let words = decision_words(decision);
        if words.len() > MAX_DECISION_WORDS_PER_ROW {
            return Err(io::Error::other("diagnosis decision exceeded its 4096-word bound").into());
        }
        self.decision_words.push(words);
        if let Some(writer) = &mut self.writer {
            writer.write(
                timing,
                estimate,
                truth.ok_or_else(|| io::Error::other("missing aligned evaluator truth"))?,
                decision,
            )?;
        }
        Ok(())
    }

    /// Closes the artifact before its byte comparison and releases the truth ring.
    pub(super) fn finish(self) -> DiagnosticResult<FinishedDiagnostics> {
        Ok(FinishedDiagnostics {
            artifact: self.writer.map(JsonlWriter::finish).transpose()?,
            decision_words: self.decision_words,
        })
    }
}

#[derive(Debug)]
pub(super) struct FinishedDiagnostics {
    pub(super) artifact: Option<Value>,
    pub(super) decision_words: Vec<Vec<u64>>,
}

/// Compares literal artifact bytes with bounded buffers, rather than just hashes.
pub(super) fn require_same_bytes(first: &Path, repeat: &Path) -> DiagnosticResult<()> {
    fn read_chunk(input: &mut impl Read, buffer: &mut [u8]) -> io::Result<usize> {
        let mut count = 0;
        while count < buffer.len() {
            let read = input.read(&mut buffer[count..])?;
            if read == 0 {
                break;
            }
            count += read;
        }
        Ok(count)
    }
    let mut first = BufReader::new(File::open(first)?);
    let mut repeat = BufReader::new(File::open(repeat)?);
    let mut a = [0; 8192];
    let mut b = [0; 8192];
    loop {
        let a_count = read_chunk(&mut first, &mut a)?;
        let b_count = read_chunk(&mut repeat, &mut b)?;
        if a_count != b_count || a[..a_count] != b[..b_count] {
            return Err(io::Error::other(
                "all-tick diagnostic JSONL differs byte for byte on fresh replay",
            )
            .into());
        }
        if a_count == 0 {
            return Ok(());
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn selections_reject_unplanned_profiles_and_doses() {
        assert_eq!(profile_names(None).unwrap(), PROFILES);
        assert!(profile_names(Some("sample_250hz")).is_err());
        assert!(inputs(Some("41")).is_err());
        assert_eq!(inputs(None).unwrap(), [("zero", 0.0), ("kick", 40.0)]);
    }

    #[test]
    fn decision_words_preserve_float_bits_and_presence() {
        assert_ne!(
            decision_words(&json!({"q":0.0})),
            decision_words(&json!({"q":-0.0}))
        );
        assert_ne!(
            decision_words(&json!({"q":null})),
            decision_words(&json!({"q":0.0}))
        );
        assert_ne!(
            decision_words(&json!({"q":1.0})),
            decision_words(&json!({"q":f64::from_bits(1.0_f64.to_bits()+1)}))
        );
        assert_eq!(
            decision_words(&json!({"a":1,"b":2})),
            decision_words(&json!({"b":2,"a":1}))
        );
    }

    #[test]
    fn streamed_artifact_rejects_skipped_ticks_and_oversized_rows() {
        let path =
            std::env::temp_dir().join(format!("rne-diagnosis-bounds-{}.jsonl", std::process::id()));
        let mut writer = JsonlWriter::new(&path).unwrap();
        let truth = json!({"observation":{"com_world_m":[0.0,0.0,0.0]}});
        writer
            .write(
                json!({"consumer_ticks":0,"capture_ticks":null}),
                Value::Null,
                truth.clone(),
                &json!({}),
            )
            .unwrap();
        assert!(writer
            .write(
                json!({"consumer_ticks":2*DT_TICKS,"capture_ticks":null}),
                Value::Null,
                truth.clone(),
                &json!({})
            )
            .is_err());
        assert!(writer
            .write(
                json!({"consumer_ticks":DT_TICKS,"capture_ticks":null}),
                Value::Null,
                truth.clone(),
                &json!({"oversized":"x".repeat(MAX_ROW_BYTES)})
            )
            .is_err());
        let artifact = writer.finish().unwrap();
        let bytes = std::fs::read(&path).unwrap();
        assert_eq!(artifact["rows"], 1);
        assert_eq!(artifact["bytes"], bytes.len());
        assert_eq!(artifact["sha256"], format!("{:x}", Sha256::digest(&bytes)));
        assert_eq!(bytes.iter().filter(|byte| **byte == b'\n').count(), 1);
        std::fs::remove_file(path).unwrap();
    }

    #[test]
    fn evaluator_contact_order_is_independent_of_backend_event_order() {
        let events = vec![
            ("foot".into(), 1, 2, Vec3::Y, 1e16),
            ("foot".into(), 1, 2, Vec3::Y, 1.0),
            ("foot".into(), 1, 2, Vec3::Y, 2.0),
            ("pelvis".into(), 1, 3, Vec3::X, 4.0),
        ];
        let expected = decision_words(&json!(aggregate_ground_events(events.clone())));
        let mut reversed = events;
        reversed.reverse();
        assert_eq!(
            expected,
            decision_words(&json!(aggregate_ground_events(reversed)))
        );
        let mut samples = vec![(Vec3::ZERO, 3.0), (Vec3::ZERO, 1.0), (Vec3::Y, 2.0)];
        let mut reversed = samples.clone();
        reversed.reverse();
        sort_foot_samples(&mut samples);
        sort_foot_samples(&mut reversed);
        assert_eq!(samples, reversed);
        assert_eq!(samples[0], (Vec3::ZERO, 1.0));
    }
}
