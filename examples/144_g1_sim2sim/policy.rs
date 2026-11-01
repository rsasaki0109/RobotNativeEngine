//! Loads the vendored policy through `rne_ai` and checks its weight digest.

use std::path::{Path, PathBuf};

use rne_ai::LstmPolicy;
use serde_json::Value;
use sha2::{Digest, Sha256};

/// Number of actuated leg joints.
pub(crate) const ACTION_LEN: usize = 12;

pub(crate) type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// Manifest of the vendored unitree_rl_gym G1 policy.
pub(crate) fn default_manifest() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/policies/unitree_g1_walk/g1_walk_lstm.policy.json")
}

/// SHA-256 of the weight blob, after checking it against the manifest.
pub(crate) fn weights_sha256() -> Result<String> {
    let manifest_path = default_manifest();
    let manifest: Value = serde_json::from_str(&std::fs::read_to_string(&manifest_path)?)?;
    let file = manifest["weights"]["file"]
        .as_str()
        .ok_or("weights.file is missing")?;
    let digest = format!(
        "{:x}",
        Sha256::digest(std::fs::read(manifest_path.with_file_name(file))?)
    );
    if manifest["weights"]["sha256"].as_str() != Some(digest.as_str()) {
        return Err("policy weight blob does not match its manifest digest".into());
    }
    Ok(digest)
}

/// Loads the vendored policy after verifying the blob digest.
pub(crate) fn load() -> Result<LstmPolicy> {
    weights_sha256()?;
    Ok(LstmPolicy::load(&default_manifest())?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rust_policy_matches_the_mujoco_reference_outputs() {
        let fixture: Value =
            serde_json::from_str(include_str!("fixtures/mujoco_policy_io.json")).expect("fixture");
        let mut policy = load().expect("policy");
        let steps = fixture["steps"].as_array().expect("steps");
        assert_eq!(steps.len(), 50);
        let mut worst = 0.0_f32;
        let mut action = [0.0_f32; ACTION_LEN];
        for step in steps {
            let obs: Vec<f32> = step["obs"]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap() as f32)
                .collect();
            policy.step(&obs, &mut action).unwrap();
            for (value, expected) in action.iter().zip(step["action"].as_array().unwrap()) {
                worst = worst.max((value - expected.as_f64().unwrap() as f32).abs());
            }
        }
        assert!(worst < 1.0e-4, "largest action difference {worst}");
    }

    #[test]
    fn deployment_contract_matches_unitree_rl_gym() {
        let policy = load().expect("policy");
        let deployment = policy.deployment();
        assert_eq!(deployment.control_decimation, 10);
        assert!((deployment.control_dt_s() - 0.02).abs() < 1e-12);
        assert_eq!(deployment.joint_names[3], "left_knee_joint");
        assert_eq!(deployment.kp_nm_per_rad[3], 150.0);
        assert_eq!(deployment.default_joint_rad[3], 0.3);
    }
}
