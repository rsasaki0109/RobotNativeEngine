//! unitree_rl_gym's G1 walking policy evaluated in plain Rust.
//!
//! The network is the TorchScript `motion.pt` actor: a one-layer LSTM over the
//! 47-value observation followed by a two-layer ELU MLP. Weights come from the
//! vendored float32 blob, whose manifest also carries the deployment contract
//! (gains, default pose, scales). Nothing from the TorchScript archive runs.

use std::path::{Path, PathBuf};

use serde_json::Value;
use sha2::{Digest, Sha256};

/// Observation length expected by the policy.
pub(crate) const OBSERVATION_LEN: usize = 47;
/// Number of actuated leg joints.
pub(crate) const ACTION_LEN: usize = 12;
const HIDDEN: usize = 64;
const MLP_HIDDEN: usize = 32;

pub(crate) type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// Deployment contract published next to the weights.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Deployment {
    pub(crate) physics_dt_s: f64,
    pub(crate) control_decimation: usize,
    pub(crate) gait_period_s: f64,
    pub(crate) joint_names: Vec<String>,
    pub(crate) kp_nm_per_rad: [f64; ACTION_LEN],
    pub(crate) kd_nm_s_per_rad: [f64; ACTION_LEN],
    pub(crate) default_joint_rad: [f64; ACTION_LEN],
    pub(crate) action_scale: f64,
    pub(crate) angular_velocity_scale: f64,
    pub(crate) joint_velocity_scale: f64,
    pub(crate) command_scale: [f64; 3],
}

/// LSTM + MLP actor with its recurrent state.
#[derive(Clone, Debug)]
pub(crate) struct LstmPolicy {
    w_ih: Vec<f32>,
    w_hh: Vec<f32>,
    bias: Vec<f32>,
    w0: Vec<f32>,
    b0: Vec<f32>,
    w1: Vec<f32>,
    b1: Vec<f32>,
    hidden: [f32; HIDDEN],
    cell: [f32; HIDDEN],
    /// Deployment contract from the manifest.
    pub(crate) deployment: Deployment,
    /// SHA-256 of the weight blob, checked against the manifest.
    pub(crate) weights_sha256: String,
}

/// Default manifest path inside the repository.
pub(crate) fn default_manifest() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/policies/unitree_g1_walk/g1_walk_lstm.policy.json")
}

fn array<const N: usize>(value: &Value, key: &str) -> Result<[f64; N]> {
    let values: Vec<f64> = value[key]
        .as_array()
        .ok_or_else(|| format!("deployment.{key} is not an array"))?
        .iter()
        .map(|v| {
            v.as_f64()
                .ok_or_else(|| format!("deployment.{key} is not numeric"))
        })
        .collect::<std::result::Result<_, _>>()?;
    values
        .try_into()
        .map_err(|_| format!("deployment.{key} must have {N} values").into())
}

fn number(value: &Value, key: &str) -> Result<f64> {
    value[key]
        .as_f64()
        .ok_or_else(|| format!("deployment.{key} is not numeric").into())
}

impl Deployment {
    fn from_manifest(value: &Value) -> Result<Self> {
        let joint_names = value["joint_order"]
            .as_array()
            .ok_or("deployment.joint_order is missing")?
            .iter()
            .map(|name| name.as_str().map(str::to_owned).ok_or("joint name"))
            .collect::<std::result::Result<Vec<_>, _>>()?;
        if joint_names.len() != ACTION_LEN {
            return Err("deployment.joint_order must list 12 joints".into());
        }
        let decimation = value["control_decimation"]
            .as_u64()
            .ok_or("deployment.control_decimation is missing")?;
        Ok(Self {
            physics_dt_s: number(value, "physics_dt_s")?,
            control_decimation: usize::try_from(decimation)?,
            gait_period_s: number(value, "gait_period_s")?,
            joint_names,
            kp_nm_per_rad: array(value, "kp_nm_per_rad")?,
            kd_nm_s_per_rad: array(value, "kd_nm_s_per_rad")?,
            default_joint_rad: array(value, "default_joint_rad")?,
            action_scale: number(value, "action_scale")?,
            angular_velocity_scale: number(value, "angular_velocity_scale")?,
            joint_velocity_scale: number(value, "joint_velocity_scale")?,
            command_scale: array(value, "command_scale")?,
        })
    }

    /// Policy period in seconds.
    pub(crate) fn control_dt_s(&self) -> f64 {
        self.physics_dt_s * self.control_decimation as f64
    }
}

fn sigmoid(x: f32) -> f32 {
    1.0 / (1.0 + (-x).exp())
}

/// `out = matrix(rows x cols) * input + out`, accumulated in f32 like numpy.
fn mat_vec_add(matrix: &[f32], cols: usize, input: &[f32], out: &mut [f32]) {
    for (row, value) in out.iter_mut().enumerate() {
        let weights = &matrix[row * cols..(row + 1) * cols];
        *value += weights.iter().zip(input).map(|(w, x)| w * x).sum::<f32>();
    }
}

impl LstmPolicy {
    /// Loads the manifest and weight blob, verifying the blob digest and shapes.
    pub(crate) fn load(manifest_path: &Path) -> Result<Self> {
        let manifest: Value = serde_json::from_str(&std::fs::read_to_string(manifest_path)?)?;
        let weights = &manifest["weights"];
        let file = weights["file"].as_str().ok_or("weights.file is missing")?;
        let blob = std::fs::read(manifest_path.with_file_name(file))?;
        let digest = format!("{:x}", Sha256::digest(&blob));
        if weights["sha256"].as_str() != Some(digest.as_str()) {
            return Err("policy weight blob does not match its manifest digest".into());
        }
        if blob.len() % 4 != 0 {
            return Err("policy weight blob is not float32".into());
        }
        let values: Vec<f32> = blob
            .chunks_exact(4)
            .map(|bytes| f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]))
            .collect();
        let tensor = |name: &str, shape: &[usize]| -> Result<Vec<f32>> {
            let entry = weights["tensors"]
                .as_array()
                .ok_or("weights.tensors is missing")?
                .iter()
                .find(|entry| entry["name"] == name)
                .ok_or_else(|| format!("tensor {name} is missing"))?;
            let declared: Vec<usize> = entry["shape"]
                .as_array()
                .ok_or("tensor shape")?
                .iter()
                .filter_map(|dim| dim.as_u64().and_then(|dim| usize::try_from(dim).ok()))
                .collect();
            if declared != shape {
                return Err(
                    format!("tensor {name} has shape {declared:?}, expected {shape:?}").into(),
                );
            }
            let start = usize::try_from(entry["offset_f32"].as_u64().ok_or("tensor offset")?)?;
            let count: usize = shape.iter().product();
            values
                .get(start..start + count)
                .map(<[f32]>::to_vec)
                .ok_or_else(|| format!("tensor {name} exceeds the blob").into())
        };
        let gates = 4 * HIDDEN;
        let bias_ih = tensor("memory.bias_ih_l0", &[gates])?;
        let bias_hh = tensor("memory.bias_hh_l0", &[gates])?;
        Ok(Self {
            w_ih: tensor("memory.weight_ih_l0", &[gates, OBSERVATION_LEN])?,
            w_hh: tensor("memory.weight_hh_l0", &[gates, HIDDEN])?,
            bias: bias_ih.iter().zip(&bias_hh).map(|(a, b)| a + b).collect(),
            w0: tensor("actor.0.weight", &[MLP_HIDDEN, HIDDEN])?,
            b0: tensor("actor.0.bias", &[MLP_HIDDEN])?,
            w1: tensor("actor.2.weight", &[ACTION_LEN, MLP_HIDDEN])?,
            b1: tensor("actor.2.bias", &[ACTION_LEN])?,
            hidden: [0.0; HIDDEN],
            cell: [0.0; HIDDEN],
            deployment: Deployment::from_manifest(&manifest["deployment"])?,
            weights_sha256: digest,
        })
    }

    /// Advances the recurrent state and returns the 12 joint actions.
    pub(crate) fn step(&mut self, observation: &[f32; OBSERVATION_LEN]) -> [f32; ACTION_LEN] {
        let mut gates = self.bias.clone();
        mat_vec_add(&self.w_ih, OBSERVATION_LEN, observation, &mut gates);
        mat_vec_add(&self.w_hh, HIDDEN, &self.hidden, &mut gates);
        for unit in 0..HIDDEN {
            let input = sigmoid(gates[unit]);
            let forget = sigmoid(gates[HIDDEN + unit]);
            let candidate = gates[2 * HIDDEN + unit].tanh();
            let output = sigmoid(gates[3 * HIDDEN + unit]);
            self.cell[unit] = forget * self.cell[unit] + input * candidate;
            self.hidden[unit] = output * self.cell[unit].tanh();
        }
        let mut features = self.b0.clone();
        mat_vec_add(&self.w0, HIDDEN, &self.hidden, &mut features);
        for value in &mut features {
            if *value <= 0.0 {
                *value = value.exp_m1();
            }
        }
        let mut action = [0.0; ACTION_LEN];
        action.copy_from_slice(&self.b1);
        mat_vec_add(&self.w1, MLP_HIDDEN, &features, &mut action);
        action
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rust_policy_matches_the_mujoco_reference_outputs() {
        let fixture: Value =
            serde_json::from_str(include_str!("fixtures/mujoco_policy_io.json")).expect("fixture");
        let mut policy = LstmPolicy::load(&default_manifest()).expect("policy");
        let steps = fixture["steps"].as_array().expect("steps");
        assert_eq!(steps.len(), 50);
        let mut worst = 0.0_f32;
        for step in steps {
            let obs: Vec<f32> = step["obs"]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap() as f32)
                .collect();
            let action = policy.step(&obs.try_into().unwrap());
            for (value, expected) in action.iter().zip(step["action"].as_array().unwrap()) {
                worst = worst.max((value - expected.as_f64().unwrap() as f32).abs());
            }
        }
        assert!(worst < 1.0e-4, "largest action difference {worst}");
    }

    #[test]
    fn deployment_contract_matches_unitree_rl_gym() {
        let policy = LstmPolicy::load(&default_manifest()).expect("policy");
        let deployment = &policy.deployment;
        assert_eq!(deployment.control_decimation, 10);
        assert!((deployment.control_dt_s() - 0.02).abs() < 1e-12);
        assert_eq!(deployment.joint_names[3], "left_knee_joint");
        assert_eq!(deployment.kp_nm_per_rad[3], 150.0);
        assert_eq!(deployment.default_joint_rad[3], 0.3);
    }
}
