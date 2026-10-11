//! Evaluator-only position-motor readback for completed physics intervals.
//!
//! These samples never enter observations, commands, or the existing replay
//! vectors. Missing readback is unavailable, rather than a zero effort sample.

use super::diagnostic;
use rne_ai::UrdfSceneSim;
use rne_physics::{JointActuation, JointPositionMotorDiagnostics, PositionMotorDiagnosticInterval};
use rne_robot::Link;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::{
    collections::BTreeMap,
    error::Error,
    fs::File,
    io::{self, BufWriter, Write},
    path::Path,
};

type MotorResult<T> = Result<T, Box<dyn Error>>;
const MAX_ROW_BYTES: usize = 128 * 1024;
const SCOPE: &str = "evaluator-only Rapier ForceBased one-DoF multibody position-motor solver readback; signed applied motor impulse excludes hard limits, contacts, and direct effort sensors; no hardware validation or fall-cause claim";

/// Rejects private motor evidence, including a render-only marker relabelling.
pub(super) fn validate_render_evidence(value: &Value) -> MotorResult<()> {
    match value {
        Value::Object(fields) => {
            if fields
                .get("motor_effort_diagnostics")
                .is_some_and(|flag| flag != &Value::Bool(false))
                || fields.contains_key("position_motor_diagnostic")
            {
                return Err(io::Error::other(
                    "evaluator-only motor diagnostics cannot be rendered as public media",
                )
                .into());
            }
            for nested in fields.values() {
                validate_render_evidence(nested)?;
            }
        }
        Value::Array(values) => {
            for nested in values {
                validate_render_evidence(nested)?;
            }
        }
        _ => {}
    }
    Ok(())
}

#[derive(Debug)]
pub(super) struct FinishedMotorDiagnostics {
    pub(super) words: Vec<Vec<u64>>,
    pub(super) metadata: Value,
}

#[derive(Debug)]
pub(super) struct MotorRun {
    output: BufWriter<File>,
    filename: String,
    hash: Sha256,
    bytes: usize,
    words: Vec<Vec<u64>>,
    joints: BTreeMap<String, JointTotals>,
}

#[derive(Debug, Default)]
struct JointTotals {
    samples_available: u64,
    samples_unavailable: u64,
    completed_samples: u64,
    applied_impulse: f64,
    completed_applied_impulse: f64,
    observed_duration_s: f64,
    completed_duration_s: f64,
    eligible_slices: u64,
    active_slices: u64,
    eligible_duration_s: f64,
    active_duration_s: f64,
    peak_abs_observed_mean: Option<f64>,
    peak_abs_completed_mean: Option<f64>,
    kind: String,
    effective_ceilings: Vec<Value>,
}

fn interval_value(interval: &PositionMotorDiagnosticInterval) -> Value {
    json!({
        "physics_world_id":interval.physics_world_id,
        "nominal_step_ticks":interval.nominal_step_ticks,
        "numerical_outer_interval_s":interval.numerical_outer_interval_s,
        "ccd_partition_durations_s":interval.ccd_partition_durations_s,
        "observed_solver_interval_s":interval.observed_solver_interval_s,
        "observed_solver_slice_count":interval.observed_solver_slice_count,
        "full_step_coverage":interval.full_step_coverage,
        "observed_partition_count":interval.observed_partition_count,
        "finite_positive_bound_eligible_slice_count":interval.finite_positive_bound_eligible_slice_count,
        "finite_positive_bound_eligible_duration_s":interval.finite_positive_bound_eligible_duration_s,
        "finite_positive_bound_active_slice_count":interval.finite_positive_bound_active_slice_count,
        "finite_positive_bound_active_duration_s":interval.finite_positive_bound_active_duration_s
    })
}

fn sample_value(sample: &JointPositionMotorDiagnostics) -> Value {
    match sample {
        JointPositionMotorDiagnostics::Revolute {
            applied_motor_impulse_nm_s,
            mean_observed_motor_effort_nm,
            mean_completed_motor_effort_nm,
            effective_effort_ceiling_nm,
            interval,
        } => json!({
            "kind":"revolute","availability":"available",
            "applied_motor_impulse_nm_s":applied_motor_impulse_nm_s,
            "mean_observed_motor_effort_nm":mean_observed_motor_effort_nm,
            "mean_completed_motor_effort_nm":mean_completed_motor_effort_nm,
            "effective_effort_ceiling_nm":effective_effort_ceiling_nm,
            "interval":interval_value(interval)
        }),
        JointPositionMotorDiagnostics::Prismatic {
            applied_motor_impulse_n_s,
            mean_observed_motor_force_n,
            mean_completed_motor_force_n,
            effective_force_ceiling_n,
            interval,
        } => json!({
            "kind":"prismatic","availability":"available",
            "applied_motor_impulse_n_s":applied_motor_impulse_n_s,
            "mean_observed_motor_force_n":mean_observed_motor_force_n,
            "mean_completed_motor_force_n":mean_completed_motor_force_n,
            "effective_force_ceiling_n":effective_force_ceiling_n,
            "interval":interval_value(interval)
        }),
    }
}

fn finite_number(value: &Value, name: &str) -> MotorResult<f64> {
    value[name]
        .as_f64()
        .filter(|number| number.is_finite())
        .ok_or_else(|| io::Error::other("invalid position-motor diagnostic number").into())
}

fn positive_ratio(numerator: f64, denominator: f64) -> Option<f64> {
    (denominator > 0.0).then(|| numerator / denominator)
}

impl JointTotals {
    fn add(&mut self, sample: &Value, nominal_ticks: u64) -> MotorResult<()> {
        let kind = sample["kind"]
            .as_str()
            .ok_or_else(|| io::Error::other("missing motor diagnostic kind"))?;
        if !self.kind.is_empty() && self.kind != kind {
            return Err(
                io::Error::other("motor diagnostic kind changed during a recording").into(),
            );
        }
        self.kind = kind.to_owned();
        if sample["availability"] == "unavailable" {
            self.samples_unavailable += 1;
            return Ok(());
        }
        let (impulse_key, mean_key, completed_key, ceiling_key) = match kind {
            "revolute" => (
                "applied_motor_impulse_nm_s",
                "mean_observed_motor_effort_nm",
                "mean_completed_motor_effort_nm",
                "effective_effort_ceiling_nm",
            ),
            "prismatic" => (
                "applied_motor_impulse_n_s",
                "mean_observed_motor_force_n",
                "mean_completed_motor_force_n",
                "effective_force_ceiling_n",
            ),
            _ => return Err(io::Error::other("unsupported motor diagnostic kind").into()),
        };
        let impulse = finite_number(sample, impulse_key)?;
        let mean = finite_number(sample, mean_key)?;
        let interval = &sample["interval"];
        let outer_s = finite_number(interval, "numerical_outer_interval_s")?;
        let observed_s = finite_number(interval, "observed_solver_interval_s")?;
        if interval["nominal_step_ticks"] != nominal_ticks || outer_s <= 0.0 || observed_s <= 0.0 {
            return Err(io::Error::other(
                "position-motor interval does not match the completed step",
            )
            .into());
        }
        if mean.to_bits() != (impulse / observed_s).to_bits() {
            return Err(io::Error::other(
                "motor observed mean does not use the actual observed solver duration",
            )
            .into());
        }
        if !sample[ceiling_key].is_null() && finite_number(sample, ceiling_key)? < 0.0 {
            return Err(io::Error::other("invalid effective position-motor ceiling").into());
        }
        self.samples_available += 1;
        if !self.effective_ceilings.contains(&sample[ceiling_key]) {
            self.effective_ceilings.push(sample[ceiling_key].clone());
        }
        self.applied_impulse += impulse;
        self.observed_duration_s += observed_s;
        self.peak_abs_observed_mean =
            Some(self.peak_abs_observed_mean.unwrap_or(0.0).max(mean.abs()));
        if !sample[completed_key].is_null() {
            let completed_mean = finite_number(sample, completed_key)?;
            if interval["full_step_coverage"] != true {
                return Err(io::Error::other(
                    "partial motor interval claimed completed-step effort",
                )
                .into());
            }
            if completed_mean.to_bits() != mean.to_bits() {
                return Err(io::Error::other(
                    "qualified motor mean differs from the observed-solver mean",
                )
                .into());
            }
            self.completed_samples += 1;
            self.completed_applied_impulse += impulse;
            self.completed_duration_s += observed_s;
            self.peak_abs_completed_mean = Some(
                self.peak_abs_completed_mean
                    .unwrap_or(0.0)
                    .max(completed_mean.abs()),
            );
        }
        let eligible_slices = interval["finite_positive_bound_eligible_slice_count"]
            .as_u64()
            .ok_or_else(|| io::Error::other("invalid eligible motor slice count"))?;
        let active_slices = interval["finite_positive_bound_active_slice_count"]
            .as_u64()
            .ok_or_else(|| io::Error::other("invalid active motor slice count"))?;
        let eligible_s = finite_number(interval, "finite_positive_bound_eligible_duration_s")?;
        let active_s = finite_number(interval, "finite_positive_bound_active_duration_s")?;
        if active_slices > eligible_slices || active_s < 0.0 || eligible_s < active_s {
            return Err(io::Error::other("invalid positive-finite motor-bound exposure").into());
        }
        self.eligible_slices += eligible_slices;
        self.active_slices += active_slices;
        self.eligible_duration_s += eligible_s;
        self.active_duration_s += active_s;
        Ok(())
    }

    fn summary(&self, link: &str) -> Value {
        let mut result = json!({
            "link":link,"kind":self.kind,"samples_available":self.samples_available,
            "samples_unavailable":self.samples_unavailable,
            "completed_step_mean_available_samples":self.completed_samples,
            "observed_solver_duration_s":self.observed_duration_s,
            "qualified_completed_solver_duration_s":self.completed_duration_s,
            "finite_positive_bound_eligible_slice_count":self.eligible_slices,
            "finite_positive_bound_active_slice_count":self.active_slices,
            "finite_positive_bound_eligible_duration_s":self.eligible_duration_s,
            "finite_positive_bound_active_duration_s":self.active_duration_s,
            "finite_positive_bound_active_fraction":positive_ratio(self.active_duration_s,self.eligible_duration_s)
        });
        let (impulse_key, observed_key, completed_key, observed_peak_key, completed_peak_key) =
            if self.kind == "revolute" {
                (
                    "signed_applied_motor_impulse_sum_nm_s",
                    "mean_over_observed_solver_time_nm",
                    "mean_over_qualified_completed_time_nm",
                    "peak_abs_observed_interval_mean_nm",
                    "peak_abs_completed_interval_mean_nm",
                )
            } else {
                (
                    "signed_applied_motor_impulse_sum_n_s",
                    "mean_over_observed_solver_time_n",
                    "mean_over_qualified_completed_time_n",
                    "peak_abs_observed_interval_mean_n",
                    "peak_abs_completed_interval_mean_n",
                )
            };
        result[impulse_key] = json!((self.samples_available > 0).then_some(self.applied_impulse));
        result[observed_key] = json!(positive_ratio(
            self.applied_impulse,
            self.observed_duration_s
        ));
        result[completed_key] = json!(positive_ratio(
            self.completed_applied_impulse,
            self.completed_duration_s
        ));
        result[observed_peak_key] = json!(self.peak_abs_observed_mean);
        result[completed_peak_key] = json!(self.peak_abs_completed_mean);
        result[if self.kind == "revolute" {
            "observed_effective_effort_ceilings_nm"
        } else {
            "observed_effective_force_ceilings_n"
        }] = json!(self.effective_ceilings);
        result
    }
}

impl MotorRun {
    pub(super) fn enabled(path: &Path) -> MotorResult<Self> {
        Ok(Self {
            output: BufWriter::new(File::create(path)?),
            filename: path
                .file_name()
                .and_then(|name| name.to_str())
                .ok_or_else(|| io::Error::other("invalid motor sidecar filename"))?
                .to_owned(),
            hash: Sha256::new(),
            bytes: 0,
            words: Vec::new(),
            joints: BTreeMap::new(),
        })
    }

    pub(super) fn record(&mut self, sim: &UrdfSceneSim, consumer_ticks: u64) -> MotorResult<()> {
        let nominal_ticks = sim.fixed_delta().ticks();
        let completed_ticks = sim.sim_time().ticks();
        if nominal_ticks == 0 || consumer_ticks.checked_add(nominal_ticks) != Some(completed_ticks)
        {
            return Err(io::Error::other(
                "motor readback must follow exactly one completed command-consumer step",
            )
            .into());
        }
        if self.words.len() >= diagnostic::MAX_ROWS {
            return Err(io::Error::other("motor diagnostic row bound exceeded").into());
        }
        let mut links: Vec<_> = sim
            .world()
            .iter_entities()
            .filter_map(|entity| {
                let link = entity.get::<Link>()?;
                let kind = match entity.get::<JointActuation>()? {
                    JointActuation::RevolutePosition { .. } => "revolute",
                    JointActuation::PrismaticPosition { .. } => "prismatic",
                    _ => return None,
                };
                Some((
                    link.name.as_str(),
                    kind,
                    entity.get::<JointPositionMotorDiagnostics>(),
                ))
            })
            .collect();
        links.sort_unstable_by(|a, b| a.0.cmp(b.0));
        if links.windows(2).any(|pair| pair[0].0 == pair[1].0) {
            return Err(io::Error::other("position-motor link names must be unique").into());
        }
        let mut samples = Vec::with_capacity(links.len());
        for (link, kind, sample) in links {
            if sample.is_some_and(|sample| !sample.has_valid_values()) {
                return Err(io::Error::other("invalid completed position-motor component").into());
            }
            let mut value = sample.map_or_else(|| json!({"kind":kind,"availability":"unavailable","reason":"no qualifying completed position-motor solver sample; not zero effort"}),sample_value);
            if value["kind"] != kind {
                return Err(io::Error::other(
                    "motor readback does not match the current position-actuation mode",
                )
                .into());
            }
            self.joints
                .entry(link.to_owned())
                .or_default()
                .add(&value, nominal_ticks)?;
            value["link"] = json!(link);
            samples.push(value);
        }
        let row = json!({
            "schema_version":1,"motor_effort_diagnostics":true,
            "scope":SCOPE,"command_consumer_ticks":consumer_ticks,
            "completed_interval_start_ticks":consumer_ticks,"completed_interval_end_ticks":completed_ticks,
            "nominal_step_ticks":nominal_ticks,"samples":samples
        });
        let mut bytes = serde_json::to_vec(&row)?;
        if bytes.len() > MAX_ROW_BYTES {
            return Err(io::Error::other("motor diagnostic row byte bound exceeded").into());
        }
        bytes.push(b'\n');
        self.output.write_all(&bytes)?;
        self.hash.update(&bytes);
        self.bytes += bytes.len();
        self.words.push(diagnostic::decision_words(&row));
        Ok(())
    }

    pub(super) fn finish(mut self) -> MotorResult<FinishedMotorDiagnostics> {
        self.output.flush()?;
        let metadata = json!({
            "schema_version":1,"motor_effort_diagnostics":true,"scope":SCOPE,
            "enabled":true,"file":self.filename,"sha256":format!("{:x}",self.hash.finalize()),
            "bytes":self.bytes,"rows":self.words.len(),"maximum_rows":diagnostic::MAX_ROWS,
            "maximum_row_bytes":MAX_ROW_BYTES,"link_order":"ascending child-link name",
            "physics_world_id_scope":"local namespace within one backend instance; not a global or cross-backend identity",
            "timing":"post-step readback: command consumer t bound to completed interval [t,t+dt]; numerical solver duration and integer nominal ticks are both retained",
            "availability":"missing component is unavailable, not zero; null completed mean denotes unqualified full-step coverage; null effective ceiling is unbounded, while zero is an observed zero cap and not positive-finite-bound eligible",
            "bound_scope":"final solved positive-finite motor impulse bound occupancy over observed solver slices; distinct from requested target or target-rate clamps",
            "aggregate_scope":"signed impulse summed in tick order; both observed and qualified-completed means use actual observed solver duration, never nominal ticks or numerical outer duration; completed mean includes only full-step-qualified intervals; signed means can cancel and do not represent within-slice peaks",
            "joint_statistics":self.joints.iter().map(|(name,total)|total.summary(name)).collect::<Vec<_>>()
        });
        Ok(FinishedMotorDiagnostics {
            words: self.words,
            metadata,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn render_only_cannot_relabel_nested_or_disabled_motor_evidence() {
        assert!(
            validate_render_evidence(&json!({"motor_effort_diagnostics":false,"frames":[]}))
                .is_ok()
        );
        for evidence in [
            json!({"motor_effort_diagnostics":true}),
            json!({"motor_effort_diagnostics":false,"position_motor_diagnostic":{"enabled":false}}),
        ] {
            assert!(validate_render_evidence(&evidence).is_err());
            assert!(validate_render_evidence(
                &json!({"frames":[],"controller":{"evaluator":{"samples":[evidence]}}})
            )
            .is_err());
        }
    }

    fn observed(impulse: f64, completed: bool, cap: Option<f64>) -> Value {
        json!({"kind":"revolute","availability":"available","applied_motor_impulse_nm_s":impulse,
            "mean_observed_motor_effort_nm":impulse/0.001,"mean_completed_motor_effort_nm":completed.then_some(impulse/0.001),
            "effective_effort_ceiling_nm":cap,"interval":{"nominal_step_ticks":1_000_000,
            "numerical_outer_interval_s":0.001,"observed_solver_interval_s":0.001,"full_step_coverage":completed,
            "finite_positive_bound_eligible_slice_count":if cap.is_some_and(|value|value>0.0){2}else{0},
            "finite_positive_bound_active_slice_count":if cap.is_some_and(|value|value>0.0){1}else{0},
            "finite_positive_bound_eligible_duration_s":if cap.is_some_and(|value|value>0.0){0.001}else{0.0},
            "finite_positive_bound_active_duration_s":if cap.is_some_and(|value|value>0.0){0.0005}else{0.0}}})
    }

    #[test]
    fn missing_is_not_zero_and_partial_intervals_do_not_supply_completed_means() {
        let mut totals = JointTotals::default();
        totals
            .add(
                &json!({"kind":"revolute","availability":"unavailable"}),
                1_000_000,
            )
            .unwrap();
        let missing = totals.summary("joint");
        assert!(missing["signed_applied_motor_impulse_sum_nm_s"].is_null());
        assert!(missing["mean_over_observed_solver_time_nm"].is_null());
        assert!(missing["finite_positive_bound_active_fraction"].is_null());
        totals
            .add(&observed(-0.012, false, Some(25.0)), 1_000_000)
            .unwrap();
        let partial = totals.summary("joint");
        assert_eq!(partial["mean_over_observed_solver_time_nm"], -12.0);
        assert!(partial["mean_over_qualified_completed_time_nm"].is_null());
        assert_eq!(partial["samples_unavailable"], 1);
        assert_eq!(partial["finite_positive_bound_active_fraction"], 0.5);
    }

    #[test]
    fn zero_cap_and_unbounded_cap_have_no_positive_finite_bound_denominator() {
        for cap in [Some(0.0), None] {
            let mut totals = JointTotals::default();
            totals.add(&observed(0.0, true, cap), 1_000_000).unwrap();
            let summary = totals.summary("joint");
            assert_eq!(summary["mean_over_qualified_completed_time_nm"], 0.0);
            assert!(summary["finite_positive_bound_active_fraction"].is_null());
        }
    }

    #[test]
    fn claimed_completed_mean_on_partial_or_stale_interval_is_rejected() {
        let mut partial = observed(0.01, true, Some(25.0));
        partial["interval"]["full_step_coverage"] = json!(false);
        assert!(JointTotals::default().add(&partial, 1_000_000).is_err());
        assert!(JointTotals::default()
            .add(&observed(0.01, true, Some(25.0)), 500_000)
            .is_err());
    }

    #[test]
    fn qualified_mean_uses_actual_solver_duration_instead_of_nominal_outer_duration() {
        let observed_s = f64::from(0.001_f32);
        let impulse = 0.012;
        let mut sample = observed(impulse, true, Some(25.0));
        sample["interval"]["observed_solver_interval_s"] = json!(observed_s);
        sample["mean_observed_motor_effort_nm"] = json!(impulse / observed_s);
        sample["mean_completed_motor_effort_nm"] = json!(impulse / observed_s);
        let mut totals = JointTotals::default();
        totals.add(&sample, 1_000_000).unwrap();
        let summary = totals.summary("joint");
        assert_eq!(
            summary["mean_over_qualified_completed_time_nm"],
            impulse / observed_s
        );
        assert_ne!(summary["mean_over_qualified_completed_time_nm"], 12.0);
        assert_eq!(summary["qualified_completed_solver_duration_s"], observed_s);
    }
}
