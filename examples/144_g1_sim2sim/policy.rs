//! Loads the vendored policy through `rne_ai` and checks its weight digest.

use std::path::{Path, PathBuf};

use rne_ai::LstmPolicy;
use serde_json::Value;
use sha2::{Digest, Sha256};

pub(crate) type Result<T> = std::result::Result<T, Box<dyn std::error::Error>>;

/// A unitree_rl_gym robot with a vendored model, policy and MuJoCo reference.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Robot {
    G1,
    H1,
}

impl Robot {
    pub(crate) fn parse(name: &str) -> Result<Self> {
        match name {
            "g1" => Ok(Self::G1),
            "h1" => Ok(Self::H1),
            other => Err(format!("unknown robot {other}; expected g1 or h1").into()),
        }
    }

    pub(crate) fn name(self) -> &'static str {
        match self {
            Self::G1 => "g1",
            Self::H1 => "h1",
        }
    }

    fn assets() -> &'static Path {
        Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/../../assets"))
    }

    /// Manifest of the vendored policy.
    pub(crate) fn manifest(self) -> PathBuf {
        Self::assets().join(match self {
            Self::G1 => "policies/unitree_g1_walk/g1_walk_lstm.policy.json",
            Self::H1 => "policies/unitree_h1_walk/h1_walk_lstm.policy.json",
        })
    }

    /// RNE scene the policy walks in. With `self_collision`, the links also
    /// collide with each other except a link and its parent, as in the MJCF.
    pub(crate) fn scene(self, self_collision: bool) -> PathBuf {
        Self::assets().join(match (self, self_collision) {
            (Self::G1, false) => "scenes/unitree_g1_12dof_walk.rne.scene.toml",
            (Self::G1, true) => "scenes/unitree_g1_12dof_walk_self_collision.rne.scene.toml",
            (Self::H1, false) => "scenes/unitree_h1_walk.rne.scene.toml",
            (Self::H1, true) => "scenes/unitree_h1_walk_self_collision.rne.scene.toml",
        })
    }

    /// MuJoCo walking reference recorded by `mujoco_reference.py`.
    pub(crate) fn reference(self) -> &'static str {
        match self {
            Self::G1 => include_str!("fixtures/mujoco_reference.json"),
            Self::H1 => include_str!("fixtures/h1_mujoco_reference.json"),
        }
    }

    /// MuJoCo's first 50 policy observations and actions.
    #[cfg(test)]
    pub(crate) fn policy_io(self) -> &'static str {
        match self {
            Self::G1 => include_str!("fixtures/mujoco_policy_io.json"),
            Self::H1 => include_str!("fixtures/h1_mujoco_policy_io.json"),
        }
    }
}

/// SHA-256 of the weight blob, after checking it against the manifest.
pub(crate) fn weights_sha256(robot: Robot) -> Result<String> {
    let manifest_path = robot.manifest();
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
pub(crate) fn load(robot: Robot) -> Result<LstmPolicy> {
    weights_sha256(robot)?;
    Ok(LstmPolicy::load(&robot.manifest())?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rust_policies_match_the_mujoco_reference_outputs() {
        for robot in [Robot::G1, Robot::H1] {
            let fixture: Value = serde_json::from_str(robot.policy_io()).expect("fixture");
            let mut policy = load(robot).expect("policy");
            let steps = fixture["steps"].as_array().expect("steps");
            assert_eq!(steps.len(), 50);
            let mut worst = 0.0_f32;
            let mut action = vec![0.0_f32; policy.action_len()];
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
            assert!(
                worst < 1.0e-4,
                "{robot:?}: largest action difference {worst}"
            );
        }
    }

    #[test]
    fn deployment_contract_matches_unitree_rl_gym() {
        let policy = load(Robot::G1).expect("policy");
        let deployment = policy.deployment();
        assert_eq!(deployment.control_decimation, 10);
        assert!((deployment.control_dt_s() - 0.02).abs() < 1e-12);
        assert_eq!(deployment.joint_names[3], "left_knee_joint");
        assert_eq!(deployment.kp_nm_per_rad[3], 150.0);
        assert_eq!(deployment.default_joint_rad[3], 0.3);
    }
}
