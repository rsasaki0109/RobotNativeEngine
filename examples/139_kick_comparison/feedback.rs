//! Feedback policies over arrived synthetic observation frames.
//!
//! Known-age orientation projection assumes a constant captured world angular
//! rate. It produces a model estimate, not a newly measured robot state.

use super::observation::Observation;
use rne_core::SimTime;
use rne_data::Frame;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{borrow::Cow, error::Error, io};

type FeedbackResult<T> = Result<T, Box<dyn Error>>;
const MAX_ORIENTATION_AGE_TICKS: u64 = 20_000_000;
const LATERAL_GYRO_EMA_TAU_S: f64 = 0.005;
const LATERAL_GYRO_EMA_REPLAY_TAG: u64 = 0x454d_4135_4759_5258;

/// Explicit choice independent of observation profile or input dose.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) enum Policy {
    /// Uses the delivered captured payload without any extra arithmetic.
    #[default]
    RawReference,
    /// Projects orientation using the known age and captured world angular rate.
    OrientationKnownAge,
    /// Adds a fixed 5 ms EMA to world gyro X after the unchanged pose projection.
    OrientationKnownAgeLateralGyroEma5ms,
}

impl Policy {
    /// Stable name recorded with each feedback decision and experiment case.
    pub(super) fn name(&self) -> &'static str {
        match self {
            Self::RawReference => "raw_reference",
            Self::OrientationKnownAge => "orientation_known_age",
            Self::OrientationKnownAgeLateralGyroEma5ms => {
                "orientation_known_age_lateral_gyro_ema_5ms"
            }
        }
    }

    /// Selects one named policy or both policies in the fixed experiment order.
    pub(super) fn selected(filter: Option<&str>) -> FeedbackResult<Vec<Self>> {
        let selected: Vec<_> = [Self::RawReference, Self::OrientationKnownAge]
            .into_iter()
            .filter(|policy| filter.is_none_or(|name| name == policy.name()))
            .collect();
        if selected.is_empty() {
            return Err(io::Error::other("unknown --feedback-policy").into());
        }
        Ok(selected)
    }

    /// Selects the two policies in the fixed lateral-gyro experiment order.
    pub(super) fn selected_lateral(filter: Option<&str>) -> FeedbackResult<Vec<Self>> {
        let selected: Vec<_> = [
            Self::OrientationKnownAge,
            Self::OrientationKnownAgeLateralGyroEma5ms,
        ]
        .into_iter()
        .filter(|policy| filter.is_none_or(|name| name == policy.name()))
        .collect();
        if selected.is_empty() {
            return Err(
                io::Error::other("unknown --feedback-policy for lateral-gyro study").into(),
            );
        }
        Ok(selected)
    }

    /// Declares temporal and channel scope without claiming measured recovery.
    pub(super) fn configuration(&self) -> Value {
        match self {
            Self::RawReference => json!({
                "policy":self.name(),
                "stateful":false,
                "orientation_time_scope":"original capture time",
                "other_channels_time_scope":"original capture time",
                "payload_arithmetic":"none",
            }),
            Self::OrientationKnownAge => json!({
                "policy":self.name(),
                "stateful":false,
                "capture_age":"(consumer_ticks - capture_ticks) * 1e-9 seconds",
                "max_capture_age_ticks":MAX_ORIENTATION_AGE_TICKS,
                "orientation_time_scope":"consumer-time model estimate using constant captured world angular rate",
                "other_channels_time_scope":"original capture time",
                "rotation_order":"rotation_vector_quaternion(world_gyro * age_s) * captured_rotation",
                "exact_identity_paths":["zero capture age","zero angular rotation"],
                "unchanged_channels":["CoM position","CoM velocity","foot positions","foot loads",
                    "mass","joint positions","joint velocities","world angular velocity","roll rate"],
                "held_sample":"recomputed from the original arrived payload without mutation",
                "unsupported_age_behavior":"explicit error",
                "fresh_measurement":false,
            }),
            Self::OrientationKnownAgeLateralGyroEma5ms => json!({
                "policy":self.name(),
                "stateful":true,
                "capture_age":"(consumer_ticks - capture_ticks) * 1e-9 seconds",
                "max_capture_age_ticks":MAX_ORIENTATION_AGE_TICKS,
                "orientation_projection":"unchanged orientation_known_age using original captured world gyro",
                "orientation_time_scope":"consumer-time model estimate using constant captured world angular rate",
                "filtered_channel":"controller-facing world angular velocity X only at positive capture age",
                "filtered_rate_time_scope":"EMA over delivered captures, not a current gyro measurement",
                "tau_s":LATERAL_GYRO_EMA_TAU_S,
                "new_capture_update":"first raw X exactly; then alpha = 1 - exp(-capture_spacing_s / tau_s)",
                "held_sample":"reuse cached filtered X; project orientation from the original payload again",
                "missing_sample":"no history update; retain nominal-stance startup behavior",
                "zero_age":"borrow original payload with no arithmetic or history update",
                "other_channels_time_scope":"original capture time",
                "unchanged_channels":["gyro Y/Z","captured roll rate","CoM position","CoM velocity",
                    "foot positions","foot loads","mass","joint positions","joint velocities"],
                "unsupported_age_behavior":"explicit error without state mutation",
                "fresh_measurement":false,
            }),
        }
    }

    /// Produces effective feedback without modifying the frame or its queue.
    ///
    /// Missing frames preserve the controller's nominal-stance startup path.
    /// The candidate requires an arrived frame no older than 20 ms, and uses
    /// capture time rather than arrival time to project orientation.
    pub(super) fn effective<'a>(
        &self,
        frame: Option<&'a Frame<Observation>>,
        now: SimTime,
    ) -> FeedbackResult<Option<Cow<'a, Observation>>> {
        if *self == Self::OrientationKnownAgeLateralGyroEma5ms {
            return Err(io::Error::other(
                "lateral-gyro EMA feedback requires a controller-owned Estimator",
            )
            .into());
        }
        let Some(frame) = frame else {
            return Ok(None);
        };
        if *self == Self::RawReference {
            return Ok(Some(Cow::Borrowed(&frame.payload)));
        }
        if frame.capture_time > now || frame.available_time > now {
            return Err(io::Error::other("orientation feedback requires an arrived frame").into());
        }
        let age_ticks = now.ticks() - frame.capture_time.ticks();
        if age_ticks > MAX_ORIENTATION_AGE_TICKS {
            return Err(io::Error::other("orientation feedback capture age exceeds 20 ms").into());
        }
        Ok(Some(frame.payload.orientation_projected(age_ticks)))
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct CaptureIdentity {
    stream_id: u64,
    entity_bits: u64,
    sequence: u64,
    capture_ticks: u64,
    available_ticks: u64,
    frame_sim_time_ticks: u64,
}

impl CaptureIdentity {
    fn from_frame(frame: &Frame<Observation>) -> Self {
        Self {
            stream_id: frame.stream_id.0,
            entity_bits: frame.entity.to_bits(),
            sequence: frame.sequence,
            capture_ticks: frame.capture_time.ticks(),
            available_ticks: frame.available_time.ticks(),
            frame_sim_time_ticks: frame.sim_time.ticks(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct LateralGyroState {
    identity: CaptureIdentity,
    raw_payload_fingerprint: [u64; 4],
    raw_gyro_x_rad_s: f64,
    filtered_gyro_x_rad_s: f64,
    previous_capture_ticks: Option<u64>,
    capture_spacing_ticks: Option<u64>,
    alpha: Option<f64>,
}

/// Constant-size filter history owned by one controller and one source stream.
///
/// Raw and orientation-only policies delegate without touching this history.
/// Missing, held, and zero-age captures do not update it. Candidate errors are
/// transactional: the state is committed only after all checks succeed.
#[derive(Debug, Default)]
pub(super) struct Estimator {
    lateral_gyro: Option<LateralGyroState>,
}

impl Estimator {
    /// Returns effective feedback while keeping raw delivered frames immutable.
    pub(super) fn effective<'a>(
        &mut self,
        policy: Policy,
        frame: Option<&'a Frame<Observation>>,
        now: SimTime,
    ) -> FeedbackResult<Option<Cow<'a, Observation>>> {
        if policy != Policy::OrientationKnownAgeLateralGyroEma5ms {
            return policy.effective(frame, now);
        }
        let Some(frame) = frame else {
            return Ok(None);
        };
        if frame.capture_time > now || frame.available_time > now {
            return Err(io::Error::other("lateral-gyro feedback requires an arrived frame").into());
        }
        if frame.available_time < frame.capture_time || frame.sim_time != frame.available_time {
            return Err(io::Error::other(
                "lateral-gyro feedback frame timestamps are inconsistent",
            )
            .into());
        }
        let age_ticks = now.ticks() - frame.capture_time.ticks();
        if age_ticks > MAX_ORIENTATION_AGE_TICKS {
            return Err(io::Error::other("lateral-gyro feedback capture age exceeds 20 ms").into());
        }
        let identity = CaptureIdentity::from_frame(frame);
        let held = if let Some(previous) = self.lateral_gyro {
            if identity.stream_id != previous.identity.stream_id
                || identity.entity_bits != previous.identity.entity_bits
            {
                return Err(
                    io::Error::other("lateral-gyro feedback source identity changed").into(),
                );
            }
            if identity.sequence < previous.identity.sequence {
                return Err(
                    io::Error::other("lateral-gyro feedback sequence moved backwards").into(),
                );
            }
            if identity.sequence == previous.identity.sequence {
                if identity != previous.identity {
                    return Err(io::Error::other("a held lateral-gyro capture was modified").into());
                }
                true
            } else {
                if identity.capture_ticks <= previous.identity.capture_ticks
                    || identity.available_ticks < previous.identity.available_ticks
                {
                    return Err(io::Error::other(
                        "lateral-gyro capture timestamps did not advance",
                    )
                    .into());
                }
                false
            }
        } else {
            false
        };
        frame.payload.validate()?;
        if age_ticks == 0 {
            // The pipeline owns raw payload immutability. This identity path
            // performs no fingerprinting, projection, or filter arithmetic.
            return Ok(Some(Cow::Borrowed(&frame.payload)));
        }
        let fingerprint = payload_fingerprint(&frame.payload);
        let raw_gyro_x_rad_s = frame.payload.angular_velocity_world_rad_s.x;
        if let Some(previous) = self.lateral_gyro {
            if held
                && (raw_gyro_x_rad_s.to_bits() != previous.raw_gyro_x_rad_s.to_bits()
                    || fingerprint != previous.raw_payload_fingerprint)
            {
                return Err(io::Error::other("a held lateral-gyro payload was modified").into());
            }
        }
        // Reject finite inputs whose rotation-vector norm would overflow before
        // invoking the unchanged quaternion projection implementation.
        let rotation_vector_rad =
            frame.payload.angular_velocity_world_rad_s * (age_ticks as f64 * 1.0e-9);
        if !rotation_vector_rad.length_squared().is_finite() {
            return Err(
                io::Error::other("lateral-gyro orientation projection would overflow").into(),
            );
        }
        let next = match self.lateral_gyro {
            Some(previous) if held => previous,
            Some(previous) => {
                let capture_spacing_ticks =
                    identity.capture_ticks - previous.identity.capture_ticks;
                let capture_spacing_s = capture_spacing_ticks as f64 * 1.0e-9;
                let alpha = 1.0 - (-capture_spacing_s / LATERAL_GYRO_EMA_TAU_S).exp();
                let filtered_gyro_x_rad_s = previous.filtered_gyro_x_rad_s
                    + alpha * (raw_gyro_x_rad_s - previous.filtered_gyro_x_rad_s);
                if !alpha.is_finite() || !filtered_gyro_x_rad_s.is_finite() {
                    return Err(
                        io::Error::other("lateral-gyro EMA arithmetic is not finite").into(),
                    );
                }
                LateralGyroState {
                    identity,
                    raw_payload_fingerprint: fingerprint,
                    raw_gyro_x_rad_s,
                    filtered_gyro_x_rad_s,
                    previous_capture_ticks: Some(previous.identity.capture_ticks),
                    capture_spacing_ticks: Some(capture_spacing_ticks),
                    alpha: Some(alpha),
                }
            }
            None => LateralGyroState {
                identity,
                raw_payload_fingerprint: fingerprint,
                raw_gyro_x_rad_s,
                filtered_gyro_x_rad_s: raw_gyro_x_rad_s,
                previous_capture_ticks: None,
                capture_spacing_ticks: None,
                alpha: None,
            },
        };
        let mut effective = frame.payload.orientation_projected(age_ticks);
        if effective.angular_velocity_world_rad_s.x.to_bits()
            != next.filtered_gyro_x_rad_s.to_bits()
        {
            effective.to_mut().angular_velocity_world_rad_s.x = next.filtered_gyro_x_rad_s;
        }
        effective.validate()?;
        self.lateral_gyro = Some(next);
        Ok(Some(effective))
    }

    /// Encodes every initialized state field as a tagged canonical replay suffix.
    ///
    /// An uninitialized estimator contributes no words, preserving old control
    /// vectors and literal zero-age policy equality.
    pub(super) fn replay_words(&self) -> Vec<u64> {
        let Some(state) = self.lateral_gyro else {
            return Vec::new();
        };
        let mut words = vec![
            LATERAL_GYRO_EMA_REPLAY_TAG,
            1,
            state.identity.stream_id,
            state.identity.entity_bits,
            state.identity.sequence,
            state.identity.capture_ticks,
            state.identity.available_ticks,
            state.identity.frame_sim_time_ticks,
        ];
        words.extend(state.raw_payload_fingerprint);
        words.extend([
            state.raw_gyro_x_rad_s.to_bits(),
            state.filtered_gyro_x_rad_s.to_bits(),
            u64::from(state.previous_capture_ticks.is_some()),
            state.previous_capture_ticks.unwrap_or(0),
            u64::from(state.capture_spacing_ticks.is_some()),
            state.capture_spacing_ticks.unwrap_or(0),
            u64::from(state.alpha.is_some()),
            state.alpha.unwrap_or(0.0).to_bits(),
        ]);
        words
    }

    /// Reports immutable filter state before or after a feedback decision.
    ///
    /// Consumer clock and age belong to the decision's frame metadata; they are
    /// not filter history and are not advanced by a held or missing sample.
    pub(super) fn diagnostic_state(&self) -> Value {
        match self.lateral_gyro {
            None => json!({"initialized":false}),
            Some(state) => json!({
                "initialized":true,
                "stream_id":state.identity.stream_id,
                "entity_bits":state.identity.entity_bits,
                "capture_sequence":state.identity.sequence,
                "capture_ticks":state.identity.capture_ticks,
                "available_ticks":state.identity.available_ticks,
                "frame_sim_time_ticks":state.identity.frame_sim_time_ticks,
                "raw_payload_fingerprint_words":state.raw_payload_fingerprint,
                "raw_world_gyro_x_rad_s":state.raw_gyro_x_rad_s,
                "filtered_world_gyro_x_rad_s":state.filtered_gyro_x_rad_s,
                "previous_capture_ticks":state.previous_capture_ticks,
                "capture_spacing_ticks":state.capture_spacing_ticks,
                "update_alpha":state.alpha,
                "tau_s":LATERAL_GYRO_EMA_TAU_S,
                "rate_time_scope":"EMA over delivered captures, not a current gyro measurement",
            }),
        }
    }
}

fn payload_fingerprint(payload: &Observation) -> [u64; 4] {
    let mut digest = Sha256::new();
    for word in payload.replay_words() {
        digest.update(word.to_le_bytes());
    }
    let bytes = digest.finalize();
    let mut words = [0; 4];
    for (word, chunk) in words.iter_mut().zip(bytes.chunks_exact(8)) {
        *word = u64::from_le_bytes(chunk.try_into().expect("SHA256 has four exact word chunks"));
    }
    words
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn raw_is_the_runtime_default_and_policy_selection_is_fixed() {
        assert_eq!(Policy::default(), Policy::RawReference);
        assert_eq!(
            Policy::selected(None).expect("fixed policy matrix"),
            [Policy::RawReference, Policy::OrientationKnownAge]
        );
        assert_eq!(
            Policy::selected(Some("orientation_known_age")).expect("known policy"),
            [Policy::OrientationKnownAge]
        );
        assert!(Policy::selected(Some("unknown")).is_err());
    }

    #[test]
    fn lateral_matrix_is_opt_in_and_cannot_use_a_stateless_candidate() {
        assert_eq!(
            Policy::selected_lateral(None).expect("lateral policy matrix"),
            [
                Policy::OrientationKnownAge,
                Policy::OrientationKnownAgeLateralGyroEma5ms,
            ]
        );
        let candidate = Policy::OrientationKnownAgeLateralGyroEma5ms;
        assert!(Policy::selected(Some(candidate.name())).is_err());
        assert_eq!(
            Policy::selected_lateral(Some(candidate.name())).expect("candidate selection"),
            [candidate]
        );
        assert!(Policy::selected_lateral(Some("raw_reference")).is_err());
        assert!(candidate.effective(None, SimTime::ZERO).is_err());
    }

    #[test]
    fn initialized_replay_suffix_covers_all_fixed_state_fields_and_ieee_bits() {
        let state = LateralGyroState {
            identity: CaptureIdentity {
                stream_id: 313,
                entity_bits: 42,
                sequence: 7,
                capture_ticks: 1_000_000,
                available_ticks: 6_000_000,
                frame_sim_time_ticks: 6_000_000,
            },
            raw_payload_fingerprint: [11, 12, 13, 14],
            raw_gyro_x_rad_s: -0.0,
            filtered_gyro_x_rad_s: -0.0,
            previous_capture_ticks: None,
            capture_spacing_ticks: None,
            alpha: None,
        };
        let baseline = Estimator {
            lateral_gyro: Some(state),
        }
        .replay_words();
        assert_eq!(baseline.len(), 20);
        assert_eq!(baseline[0], LATERAL_GYRO_EMA_REPLAY_TAG);
        let mutators: [fn(&mut LateralGyroState); 16] = [
            |state| state.identity.stream_id += 1,
            |state| state.identity.entity_bits += 1,
            |state| state.identity.sequence += 1,
            |state| state.identity.capture_ticks += 1,
            |state| state.identity.available_ticks += 1,
            |state| state.identity.frame_sim_time_ticks += 1,
            |state| state.raw_payload_fingerprint[0] += 1,
            |state| state.raw_payload_fingerprint[1] += 1,
            |state| state.raw_payload_fingerprint[2] += 1,
            |state| state.raw_payload_fingerprint[3] += 1,
            |state| state.raw_gyro_x_rad_s = 0.0,
            |state| state.filtered_gyro_x_rad_s = 0.0,
            |state| state.previous_capture_ticks = Some(0),
            |state| state.capture_spacing_ticks = Some(0),
            |state| state.alpha = Some(-0.0),
            |state| state.alpha = Some(0.0),
        ];
        for mutate in mutators {
            let mut changed = state;
            mutate(&mut changed);
            assert_ne!(
                Estimator {
                    lateral_gyro: Some(changed),
                }
                .replay_words(),
                baseline
            );
        }
        assert!(Estimator::default().replay_words().is_empty());
    }
}
