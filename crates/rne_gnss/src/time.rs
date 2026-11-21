//! GPS time (week and time of week).
//!
//! Simulation time is seconds since a scenario start; a [`GnssTime`] epoch
//! anchors it to the GPS calendar so broadcast ephemerides, the Klobuchar
//! model, and external solvers see consistent times. Galileo system time is
//! expressed in GPS time here, as RINEX-style receivers report it.

use serde::{Deserialize, Serialize};

/// Seconds in a GPS week.
pub const SECONDS_PER_WEEK: f64 = 604_800.0;

/// A GPS time: week number and seconds of week in `[0, 604800)`.
#[derive(Clone, Copy, Debug, PartialEq, PartialOrd, Serialize, Deserialize)]
pub struct GnssTime {
    /// GPS week number (continuous, not modulo 1024).
    pub week: u32,
    /// Seconds of week.
    pub tow_s: f64,
}

impl GnssTime {
    /// Creates a normalized time.
    pub fn new(week: u32, tow_s: f64) -> Self {
        Self { week, tow_s }.add_seconds(0.0)
    }

    /// This time shifted by `seconds` (may cross week boundaries).
    pub fn add_seconds(self, seconds: f64) -> Self {
        let total = self.tow_s + seconds;
        let weeks = (total / SECONDS_PER_WEEK).floor();
        let week = (i64::from(self.week) + weeks as i64).max(0) as u32;
        Self {
            week,
            tow_s: total - weeks * SECONDS_PER_WEEK,
        }
    }

    /// `self - other` in seconds.
    pub fn seconds_since(self, other: Self) -> f64 {
        (f64::from(self.week) - f64::from(other.week)) * SECONDS_PER_WEEK
            + (self.tow_s - other.tow_s)
    }
}

impl Default for GnssTime {
    /// GPS week 2400, Tuesday 03:00:00 (2026-01-06).
    fn default() -> Self {
        Self {
            week: 2400,
            tow_s: 2.0 * 86_400.0 + 3.0 * 3_600.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn crosses_week_boundaries_both_ways() {
        let t = GnssTime::new(2000, 604_790.0).add_seconds(20.0);
        assert_eq!(t.week, 2001);
        assert!((t.tow_s - 10.0).abs() < 1e-9);
        let back = t.add_seconds(-20.0);
        assert_eq!(back.week, 2000);
        assert!((back.seconds_since(t) + 20.0).abs() < 1e-9);
    }
}
