//! Recurrent actor policies exported from legged-robot RL training.
//!
//! [`LstmPolicy`] evaluates the actor used by unitree_rl_gym's walking
//! policies: a one-layer LSTM (PyTorch gate order input, forget, cell, output;
//! zero initial state) followed by `Linear -> ELU(1) -> Linear`. Weights are a
//! float32 little-endian blob described by a JSON manifest, so a host without a
//! filesystem can embed both and nothing from a training framework is executed.
//! The manifest also carries the deployment contract the policy was trained
//! against (control period, PD gains, default pose and observation scales).
//!
//! Arithmetic is float32 with row-major accumulation, matching the reference
//! evaluation to about 1e-6 per action.

use std::path::{Path, PathBuf};

use serde_json::Value;

/// Failure to load or evaluate an [`LstmPolicy`].
#[derive(Debug, thiserror::Error)]
pub enum LstmPolicyError {
    /// The manifest is not valid JSON or misses a required field.
    #[error("invalid policy manifest: {0}")]
    Manifest(String),
    /// A tensor is missing, mis-shaped or outside the weight blob.
    #[error("invalid policy weights: {0}")]
    Weights(String),
    /// Reading the manifest or blob from disk failed.
    #[error("policy file {path}: {message}")]
    Io {
        /// File that could not be read.
        path: String,
        /// Operating-system error message.
        message: String,
    },
    /// An observation or action buffer has the wrong length.
    #[error("expected {expected} values, got {actual}")]
    Length {
        /// Length the policy requires.
        expected: usize,
        /// Length that was supplied.
        actual: usize,
    },
}

/// Deployment contract published with the weights.
///
/// Joint vectors follow [`Self::joint_names`] order.
#[derive(Clone, Debug, PartialEq)]
pub struct PolicyDeployment {
    /// Physics step the PD loop runs at, in seconds.
    pub physics_dt_s: f64,
    /// Physics steps per policy step.
    pub control_decimation: usize,
    /// Gait phase period fed to the policy as `sin`/`cos`, in seconds.
    pub gait_period_s: f64,
    /// Actuated joints in action order (URDF joint names).
    pub joint_names: Vec<String>,
    /// PD stiffness per joint in N·m/rad.
    pub kp_nm_per_rad: Vec<f64>,
    /// PD damping per joint in N·m·s/rad.
    pub kd_nm_s_per_rad: Vec<f64>,
    /// Default joint angles the actions are offsets from, in radians.
    pub default_joint_rad: Vec<f64>,
    /// Joint target = action × this + default angle (radians per action unit).
    pub action_scale: f64,
    /// Scale applied to the base angular velocity observation.
    pub angular_velocity_scale: f64,
    /// Scale applied to the joint velocity observation.
    pub joint_velocity_scale: f64,
    /// Scales applied to the (forward, lateral, yaw-rate) command.
    pub command_scale: [f64; 3],
}

impl PolicyDeployment {
    /// Policy period in seconds.
    pub fn control_dt_s(&self) -> f64 {
        self.physics_dt_s * self.control_decimation as f64
    }
}

/// LSTM + MLP actor with its recurrent state.
#[derive(Clone, Debug)]
pub struct LstmPolicy {
    input_len: usize,
    hidden_len: usize,
    feature_len: usize,
    output_len: usize,
    w_ih: Vec<f32>,
    w_hh: Vec<f32>,
    bias: Vec<f32>,
    w0: Vec<f32>,
    b0: Vec<f32>,
    w1: Vec<f32>,
    b1: Vec<f32>,
    hidden: Vec<f32>,
    cell: Vec<f32>,
    gates: Vec<f32>,
    features: Vec<f32>,
    deployment: PolicyDeployment,
}

/// Manifest of the vendored unitree_rl_gym G1 walking policy.
pub fn unitree_g1_walk_policy_manifest() -> PathBuf {
    crate::asset_path::bundled_asset_path("policies/unitree_g1_walk/g1_walk_lstm.policy.json")
}

fn manifest_error(message: impl Into<String>) -> LstmPolicyError {
    LstmPolicyError::Manifest(message.into())
}

fn number(value: &Value, key: &str) -> Result<f64, LstmPolicyError> {
    value[key]
        .as_f64()
        .filter(|number| number.is_finite())
        .ok_or_else(|| manifest_error(format!("deployment.{key} is not a finite number")))
}

fn numbers(value: &Value, key: &str, len: usize) -> Result<Vec<f64>, LstmPolicyError> {
    let values = value[key]
        .as_array()
        .ok_or_else(|| manifest_error(format!("deployment.{key} is not an array")))?
        .iter()
        .map(|item| item.as_f64().filter(|number| number.is_finite()))
        .collect::<Option<Vec<_>>>()
        .ok_or_else(|| manifest_error(format!("deployment.{key} has a non-finite value")))?;
    if values.len() != len {
        return Err(manifest_error(format!(
            "deployment.{key} has {} values, expected {len}",
            values.len()
        )));
    }
    Ok(values)
}

impl PolicyDeployment {
    fn from_manifest(value: &Value, joints: usize) -> Result<Self, LstmPolicyError> {
        let joint_names = value["joint_order"]
            .as_array()
            .ok_or_else(|| manifest_error("deployment.joint_order is missing"))?
            .iter()
            .map(|name| name.as_str().map(str::to_owned))
            .collect::<Option<Vec<_>>>()
            .ok_or_else(|| manifest_error("deployment.joint_order must hold strings"))?;
        if joint_names.len() != joints {
            return Err(manifest_error(format!(
                "deployment.joint_order lists {} joints, the actor outputs {joints}",
                joint_names.len()
            )));
        }
        let control_decimation = value["control_decimation"]
            .as_u64()
            .and_then(|decimation| usize::try_from(decimation).ok())
            .filter(|decimation| *decimation > 0)
            .ok_or_else(|| manifest_error("deployment.control_decimation must be positive"))?;
        let command_scale = numbers(value, "command_scale", 3)?;
        let deployment = Self {
            physics_dt_s: number(value, "physics_dt_s")?,
            control_decimation,
            gait_period_s: number(value, "gait_period_s")?,
            joint_names,
            kp_nm_per_rad: numbers(value, "kp_nm_per_rad", joints)?,
            kd_nm_s_per_rad: numbers(value, "kd_nm_s_per_rad", joints)?,
            default_joint_rad: numbers(value, "default_joint_rad", joints)?,
            action_scale: number(value, "action_scale")?,
            angular_velocity_scale: number(value, "angular_velocity_scale")?,
            joint_velocity_scale: number(value, "joint_velocity_scale")?,
            command_scale: [command_scale[0], command_scale[1], command_scale[2]],
        };
        if deployment.physics_dt_s <= 0.0 || deployment.gait_period_s <= 0.0 {
            return Err(manifest_error(
                "time steps and gait period must be positive",
            ));
        }
        if deployment
            .kp_nm_per_rad
            .iter()
            .chain(&deployment.kd_nm_s_per_rad)
            .any(|gain| *gain < 0.0)
        {
            return Err(manifest_error("PD gains must be non-negative"));
        }
        Ok(deployment)
    }
}

/// `out += matrix(rows × cols) · input`, accumulated per row in f32.
fn mat_vec_add(matrix: &[f32], cols: usize, input: &[f32], out: &mut [f32]) {
    for (row, value) in out.iter_mut().enumerate() {
        let weights = &matrix[row * cols..(row + 1) * cols];
        *value += weights
            .iter()
            .zip(input)
            .map(|(weight, x)| weight * x)
            .sum::<f32>();
    }
}

fn sigmoid(x: f32) -> f32 {
    1.0 / (1.0 + (-x).exp())
}

impl LstmPolicy {
    /// Builds the policy from manifest JSON text and its float32 weight blob.
    ///
    /// Tensor shapes are read from the manifest and must form one LSTM layer
    /// followed by two linear layers; the blob must hold every tensor.
    pub fn from_manifest(manifest_json: &str, weights: &[u8]) -> Result<Self, LstmPolicyError> {
        let manifest: Value = serde_json::from_str(manifest_json)
            .map_err(|error| manifest_error(error.to_string()))?;
        let tensors = manifest["weights"]["tensors"]
            .as_array()
            .ok_or_else(|| manifest_error("weights.tensors is missing"))?;
        if weights.len() % 4 != 0 {
            return Err(LstmPolicyError::Weights(
                "blob length is not a multiple of 4 bytes".into(),
            ));
        }
        let tensor = |name: &str| -> Result<(Vec<usize>, Vec<f32>), LstmPolicyError> {
            let entry = tensors
                .iter()
                .find(|entry| entry["name"] == name)
                .ok_or_else(|| LstmPolicyError::Weights(format!("tensor {name} is missing")))?;
            let shape = entry["shape"]
                .as_array()
                .and_then(|dims| {
                    dims.iter()
                        .map(|dim| dim.as_u64().and_then(|dim| usize::try_from(dim).ok()))
                        .collect::<Option<Vec<_>>>()
                })
                .filter(|shape| !shape.is_empty() && shape.iter().all(|dim| *dim > 0))
                .ok_or_else(|| LstmPolicyError::Weights(format!("tensor {name} has no shape")))?;
            let start = entry["offset_f32"]
                .as_u64()
                .and_then(|offset| usize::try_from(offset).ok())
                .ok_or_else(|| LstmPolicyError::Weights(format!("tensor {name} has no offset")))?;
            let count: usize = shape.iter().product();
            let bytes = start
                .checked_mul(4)
                .and_then(|begin| Some(begin..begin.checked_add(count.checked_mul(4)?)?))
                .and_then(|range| weights.get(range))
                .ok_or_else(|| {
                    LstmPolicyError::Weights(format!("tensor {name} exceeds the blob"))
                })?;
            let values = bytes
                .chunks_exact(4)
                .map(|chunk| f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]))
                .collect::<Vec<_>>();
            if values.iter().any(|value| !value.is_finite()) {
                return Err(LstmPolicyError::Weights(format!(
                    "tensor {name} has a non-finite value"
                )));
            }
            Ok((shape, values))
        };
        let (ih_shape, w_ih) = tensor("memory.weight_ih_l0")?;
        let (hh_shape, w_hh) = tensor("memory.weight_hh_l0")?;
        let (bih_shape, bias_ih) = tensor("memory.bias_ih_l0")?;
        let (bhh_shape, bias_hh) = tensor("memory.bias_hh_l0")?;
        let (w0_shape, w0) = tensor("actor.0.weight")?;
        let (b0_shape, b0) = tensor("actor.0.bias")?;
        let (w1_shape, w1) = tensor("actor.2.weight")?;
        let (b1_shape, b1) = tensor("actor.2.bias")?;
        let shape_error = |message: &str| LstmPolicyError::Weights(message.to_owned());
        let [gates, input_len] = ih_shape[..] else {
            return Err(shape_error("memory.weight_ih_l0 must be 2-D"));
        };
        if gates % 4 != 0 {
            return Err(shape_error("LSTM gate rows must be a multiple of 4"));
        }
        let hidden_len = gates / 4;
        let [feature_len, w0_cols] = w0_shape[..] else {
            return Err(shape_error("actor.0.weight must be 2-D"));
        };
        let [output_len, w1_cols] = w1_shape[..] else {
            return Err(shape_error("actor.2.weight must be 2-D"));
        };
        if hh_shape != [gates, hidden_len]
            || bih_shape != [gates]
            || bhh_shape != [gates]
            || w0_cols != hidden_len
            || b0_shape != [feature_len]
            || w1_cols != feature_len
            || b1_shape != [output_len]
        {
            return Err(shape_error("tensors do not chain LSTM -> Linear -> Linear"));
        }
        let deployment = PolicyDeployment::from_manifest(&manifest["deployment"], output_len)?;
        Ok(Self {
            input_len,
            hidden_len,
            feature_len,
            output_len,
            w_ih,
            w_hh,
            bias: bias_ih.iter().zip(&bias_hh).map(|(a, b)| a + b).collect(),
            w0,
            b0,
            w1,
            b1,
            hidden: vec![0.0; hidden_len],
            cell: vec![0.0; hidden_len],
            gates: vec![0.0; gates],
            features: vec![0.0; feature_len],
            deployment,
        })
    }

    /// Reads a manifest and the weight blob it names from disk.
    pub fn load(manifest_path: &Path) -> Result<Self, LstmPolicyError> {
        let read_error = |path: &Path, error: std::io::Error| LstmPolicyError::Io {
            path: path.display().to_string(),
            message: error.to_string(),
        };
        let text = std::fs::read_to_string(manifest_path)
            .map_err(|error| read_error(manifest_path, error))?;
        let manifest: Value =
            serde_json::from_str(&text).map_err(|error| manifest_error(error.to_string()))?;
        let file = manifest["weights"]["file"]
            .as_str()
            .filter(|file| !file.contains(['/', '\\']))
            .ok_or_else(|| manifest_error("weights.file must be a plain file name"))?;
        let blob_path = manifest_path.with_file_name(file);
        let blob = std::fs::read(&blob_path).map_err(|error| read_error(&blob_path, error))?;
        Self::from_manifest(&text, &blob)
    }

    /// Number of observation values per step.
    pub fn observation_len(&self) -> usize {
        self.input_len
    }

    /// Number of action values per step.
    pub fn action_len(&self) -> usize {
        self.output_len
    }

    /// Deployment contract from the manifest.
    pub fn deployment(&self) -> &PolicyDeployment {
        &self.deployment
    }

    /// Clears the recurrent state, as at the start of an episode.
    pub fn reset(&mut self) {
        self.hidden.fill(0.0);
        self.cell.fill(0.0);
    }

    /// Advances the recurrent state on one observation and writes the actions.
    pub fn step(&mut self, observation: &[f32], action: &mut [f32]) -> Result<(), LstmPolicyError> {
        if observation.len() != self.input_len {
            return Err(LstmPolicyError::Length {
                expected: self.input_len,
                actual: observation.len(),
            });
        }
        if action.len() != self.output_len {
            return Err(LstmPolicyError::Length {
                expected: self.output_len,
                actual: action.len(),
            });
        }
        let hidden = self.hidden_len;
        self.gates.copy_from_slice(&self.bias);
        mat_vec_add(&self.w_ih, self.input_len, observation, &mut self.gates);
        mat_vec_add(&self.w_hh, hidden, &self.hidden, &mut self.gates);
        for unit in 0..hidden {
            let input = sigmoid(self.gates[unit]);
            let forget = sigmoid(self.gates[hidden + unit]);
            let candidate = self.gates[2 * hidden + unit].tanh();
            let output = sigmoid(self.gates[3 * hidden + unit]);
            self.cell[unit] = forget * self.cell[unit] + input * candidate;
            self.hidden[unit] = output * self.cell[unit].tanh();
        }
        self.features.copy_from_slice(&self.b0);
        mat_vec_add(&self.w0, hidden, &self.hidden, &mut self.features);
        for value in &mut self.features {
            if *value <= 0.0 {
                *value = value.exp_m1();
            }
        }
        action.copy_from_slice(&self.b1);
        mat_vec_add(&self.w1, self.feature_len, &self.features, action);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn g1() -> LstmPolicy {
        LstmPolicy::load(&unitree_g1_walk_policy_manifest()).expect("vendored G1 policy")
    }

    #[test]
    fn vendored_g1_policy_has_the_unitree_rl_gym_contract() {
        let policy = g1();
        assert_eq!(policy.observation_len(), 47);
        assert_eq!(policy.action_len(), 12);
        let deployment = policy.deployment();
        assert!((deployment.control_dt_s() - 0.02).abs() < 1e-12);
        assert_eq!(deployment.joint_names[3], "left_knee_joint");
        assert_eq!(deployment.kp_nm_per_rad[3], 150.0);
    }

    #[test]
    fn recurrent_state_carries_over_and_resets() {
        let mut policy = g1();
        let observation = [0.1_f32; 47];
        let mut first = [0.0_f32; 12];
        let mut second = [0.0_f32; 12];
        policy.step(&observation, &mut first).unwrap();
        policy.step(&observation, &mut second).unwrap();
        assert_ne!(
            first, second,
            "the LSTM state must change the second answer"
        );
        policy.reset();
        let mut again = [0.0_f32; 12];
        policy.step(&observation, &mut again).unwrap();
        assert_eq!(first.map(f32::to_bits), again.map(f32::to_bits));
    }

    #[test]
    fn rejects_wrong_lengths_and_truncated_weights() {
        let mut policy = g1();
        let mut action = [0.0_f32; 12];
        assert!(matches!(
            policy.step(&[0.0; 46], &mut action),
            Err(LstmPolicyError::Length {
                expected: 47,
                actual: 46
            })
        ));
        let path = unitree_g1_walk_policy_manifest();
        let manifest = std::fs::read_to_string(&path).unwrap();
        let blob = std::fs::read(path.with_file_name("g1_walk_lstm.f32")).unwrap();
        assert!(matches!(
            LstmPolicy::from_manifest(&manifest, &blob[..blob.len() - 4]),
            Err(LstmPolicyError::Weights(_))
        ));
    }
}
