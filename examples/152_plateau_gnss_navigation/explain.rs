//! What the localizer did and why, in words, for the render's HUD: the
//! verdict on each GNSS solution, the LiDAR match, and the reason for every
//! change of source.

use crate::gnss::GnssOutput;
use rne_nav::Pose2d;
use rne_slam::{FixKind, GnssDecision, LocalizationSource};

/// How long a change of source stays on screen, seconds.
const EVENT_SHOWN_S: f64 = 6.0;

/// The words for one frame.
#[derive(Clone, Debug, Default)]
#[cfg_attr(not(feature = "capture"), allow(dead_code))]
pub(crate) struct Status {
    pub gnss: String,
    pub lidar: String,
    /// The latest change of source and why, while it is recent.
    pub event: Option<String>,
}

/// Turns the localizer's inputs and decisions into words.
#[derive(Clone, Debug)]
pub(crate) struct Explainer {
    gnss: String,
    lidar: String,
    source: LocalizationSource,
    /// Why the latest GNSS solutions were turned down.
    last_rejection: String,
    events: Vec<(f64, String)>,
}

fn kind_name(kind: FixKind) -> &'static str {
    match kind {
        FixKind::Fixed => "RTK fixed",
        FixKind::Float => "RTK float",
        FixKind::Single => "single point",
    }
}

impl Explainer {
    pub(crate) fn new() -> Self {
        Self {
            gnss: "GNSS: starting".to_string(),
            lidar: "LiDAR: not in use".to_string(),
            source: LocalizationSource::Gnss,
            last_rejection: String::new(),
            events: Vec::new(),
        }
    }

    /// A GNSS epoch and the hybrid's verdict on it (`None` for the factor
    /// graph, which weighs every solution instead of judging it).
    pub(crate) fn gnss(
        &mut self,
        output: &GnssOutput,
        decision: Option<GnssDecision>,
        estimate: Pose2d,
    ) {
        let Some(fix) = output.fix.as_ref() else {
            self.gnss = "GNSS: no solution (too few satellites)".to_string();
            self.last_rejection = "no solution".to_string();
            return;
        };
        let off = (fix.x_m - estimate.x_m).hypot(fix.y_m - estimate.y_m);
        let what = format!(
            "{}, {} satellites",
            kind_name(fix.kind),
            fix.satellites_used
        );
        let Some(decision) = decision else {
            self.gnss = format!("GNSS: {what}, {off:.1} m from the estimate");
            return;
        };
        let verdict = match decision {
            GnssDecision::Fused if self.source == LocalizationSource::Lidar => {
                "corrects the LiDAR map match".to_string()
            }
            GnssDecision::Fused => "used".to_string(),
            GnssDecision::Healthy if fix.kind == FixKind::Fixed => {
                "passes; waiting for 5 in a row to trust GNSS".to_string()
            }
            GnssDecision::Healthy if self.source == LocalizationSource::Lidar => {
                "passes, but only RTK fixed corrects LiDAR".to_string()
            }
            GnssDecision::Healthy => "passes, but only RTK fixed can take over".to_string(),
            GnssDecision::Missing => "missing".to_string(),
            GnssDecision::RejectedQuality => {
                let why = if !fix.consistent {
                    "failed the consistency check".to_string()
                } else if fix.satellites_used < 6 {
                    format!("only {} satellites (needs 6)", fix.satellites_used)
                } else if fix.hdop > 2.5 {
                    format!("HDOP {:.1} (> 2.5)", fix.hdop)
                } else if fix.satellites_excluded > 2 {
                    format!("{} satellites excluded as faulty", fix.satellites_excluded)
                } else {
                    format!("receiver reports ±{:.1} m", fix.horizontal_std_m)
                };
                self.last_rejection = why.clone();
                format!("rejected: {why}")
            }
            GnssDecision::RejectedGate => {
                self.last_rejection = format!("{off:.1} m off the estimate");
                format!("rejected: {off:.1} m off the estimate")
            }
        };
        self.gnss = format!("GNSS: {what} -> {verdict}");
    }

    /// The latest LiDAR state, in words.
    pub(crate) fn lidar(&mut self, text: String) {
        self.lidar = text;
    }

    /// Records a change of the localizer's source, with the reason.
    pub(crate) fn source(&mut self, t: f64, source: LocalizationSource) {
        if source == self.source {
            return;
        }
        let reason = match (self.source, source) {
            (LocalizationSource::Gnss, LocalizationSource::DeadReckoning) => format!(
                "GNSS -> dead reckoning: 3 solutions in a row rejected ({})",
                self.last_rejection
            ),
            (LocalizationSource::Gnss, LocalizationSource::Lidar) => {
                "GNSS -> LiDAR map: the 3D scan agrees with GNSS; it keeps charge, GNSS corrects it"
                    .to_string()
            }
            (_, LocalizationSource::Lidar) => {
                "dead reckoning -> LiDAR map: the scan found its place in the map".to_string()
            }
            (LocalizationSource::Lidar, LocalizationSource::DeadReckoning) => {
                "LiDAR map -> dead reckoning: no reliable map match".to_string()
            }
            (_, LocalizationSource::Gnss) => {
                "-> GNSS: 5 RTK fixed solutions in a row passed every check".to_string()
            }
            (_, LocalizationSource::DeadReckoning) => "-> dead reckoning".to_string(),
        };
        self.events.push((t, reason));
        self.source = source;
    }

    /// The words for a frame at `t`.
    pub(crate) fn status(&self, t: f64) -> Status {
        Status {
            gnss: self.gnss.clone(),
            lidar: self.lidar.clone(),
            event: self
                .events
                .last()
                .filter(|(at, _)| t - at <= EVENT_SHOWN_S)
                .map(|(_, text)| text.clone()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::gnss::SolutionLabel;
    use rne_slam::AbsolutePositionFix;

    fn output(kind: FixKind, sats: usize, x: f64) -> GnssOutput {
        GnssOutput {
            fix: Some(AbsolutePositionFix {
                time_s: 0.0,
                x_m: x,
                y_m: 0.0,
                horizontal_std_m: 0.02,
                satellites_used: sats,
                hdop: 1.0,
                consistent: true,
                satellites_excluded: 0,
                kind,
            }),
            label: SolutionLabel::Fixed,
            heading: None,
        }
    }

    #[test]
    fn verdicts_and_switches_read_as_sentences() {
        let mut e = Explainer::new();
        let here = Pose2d::new(0.0, 0.0, 0.0);
        e.gnss(
            &output(FixKind::Fixed, 9, 5.0),
            Some(GnssDecision::RejectedGate),
            here,
        );
        assert_eq!(
            e.status(0.0).gnss,
            "GNSS: RTK fixed, 9 satellites -> rejected: 5.0 m off the estimate"
        );
        e.source(1.0, LocalizationSource::DeadReckoning);
        assert_eq!(
            e.status(2.0).event.as_deref(),
            Some("GNSS -> dead reckoning: 3 solutions in a row rejected (5.0 m off the estimate)")
        );
        e.gnss(
            &output(FixKind::Float, 4, 0.0),
            Some(GnssDecision::RejectedQuality),
            here,
        );
        assert!(e
            .status(2.0)
            .gnss
            .ends_with("rejected: only 4 satellites (needs 6)"));
        // The event fades.
        assert!(e.status(10.0).event.is_none());
        // The factor graph has no verdicts.
        e.gnss(&output(FixKind::Fixed, 8, 0.3), None, here);
        assert_eq!(
            e.status(10.0).gnss,
            "GNSS: RTK fixed, 8 satellites, 0.3 m from the estimate"
        );
    }
}
