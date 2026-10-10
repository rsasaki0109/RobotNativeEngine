//! Stateless feedback policies over arrived synthetic observation frames.
//!
//! Known-age orientation projection assumes a constant captured world angular
//! rate. It produces a model estimate, not a newly measured robot state.

use super::observation::Observation;
use rne_core::SimTime;
use rne_data::Frame;
use serde_json::{json, Value};
use std::{borrow::Cow, error::Error, io};

type FeedbackResult<T> = Result<T, Box<dyn Error>>;
const MAX_ORIENTATION_AGE_TICKS: u64 = 20_000_000;

/// Explicit, stateless choice independent of observation profile or input dose.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(super) enum Policy {
    /// Uses the delivered captured payload without any extra arithmetic.
    #[default]
    RawReference,
    /// Projects orientation using the known age and captured world angular rate.
    OrientationKnownAge,
}

impl Policy {
    /// Stable name recorded with each feedback decision and experiment case.
    pub(super) fn name(&self) -> &'static str {
        match self {
            Self::RawReference => "raw_reference",
            Self::OrientationKnownAge => "orientation_known_age",
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
}
