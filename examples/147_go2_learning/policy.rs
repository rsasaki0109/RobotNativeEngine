//! The walking policy: a one-hidden-layer MLP from observation to action.

use serde_json::{json, Value};

use crate::go2::PROPRIOCEPTION;
use crate::Result;

/// Joint targets per action.
pub(crate) const ACTIONS: usize = 12;

/// `a = tanh(W₂ tanh(W₁ x + b₁) + b₂)`, parameters flat in the order W₁ (row
/// per hidden unit), b₁, W₂ (row per action), b₂.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct Mlp {
    /// Observation width: the proprioception, and the height scan if the
    /// task has one.
    pub(crate) inputs: usize,
    pub(crate) hidden: usize,
    pub(crate) params: Vec<f64>,
}

impl Mlp {
    pub(crate) fn parameter_count(inputs: usize, hidden: usize) -> usize {
        hidden * inputs + hidden + ACTIONS * hidden + ACTIONS
    }

    /// W₁ from `normal`'s first deviates over √inputs; b₁, W₂ and b₂ zero, so
    /// the policy starts by holding the default pose.
    pub(crate) fn initial(inputs: usize, hidden: usize, mut normal: impl FnMut() -> f64) -> Self {
        let mut params = vec![0.0; Self::parameter_count(inputs, hidden)];
        let scale = (inputs as f64).sqrt();
        for weight in &mut params[..hidden * inputs] {
            *weight = normal() / scale;
        }
        Self {
            inputs,
            hidden,
            params,
        }
    }

    pub(crate) fn with_params(&self, params: Vec<f64>) -> Self {
        Self {
            inputs: self.inputs,
            hidden: self.hidden,
            params,
        }
    }

    /// The action for `observation`, which has [`Self::inputs`] values.
    pub(crate) fn act(&self, observation: &[f64]) -> [f64; ACTIONS] {
        assert_eq!(observation.len(), self.inputs, "observation width");
        let (inputs, hidden) = (self.inputs, self.hidden);
        let (w1, rest) = self.params.split_at(hidden * inputs);
        let (b1, rest) = rest.split_at(hidden);
        let (w2, b2) = rest.split_at(ACTIONS * hidden);
        let h: Vec<f64> = (0..hidden)
            .map(|unit| {
                let row = &w1[unit * inputs..(unit + 1) * inputs];
                (dot(row, observation) + b1[unit]).tanh()
            })
            .collect();
        let mut action = [0.0; ACTIONS];
        for (index, value) in action.iter_mut().enumerate() {
            *value = (dot(&w2[index * hidden..(index + 1) * hidden], &h) + b2[index]).tanh();
        }
        action
    }

    pub(crate) fn to_json(&self) -> Value {
        json!({"model": "mlp", "inputs": self.inputs, "hidden": self.hidden,
               "outputs": ACTIONS, "params": self.params})
    }

    pub(crate) fn from_json(value: &Value) -> Result<Self> {
        let hidden = value["hidden"]
            .as_u64()
            .ok_or("policy has no hidden width")? as usize;
        let inputs = value["inputs"]
            .as_u64()
            .map_or(PROPRIOCEPTION, |n| n as usize);
        let params: Vec<f64> = value["params"]
            .as_array()
            .ok_or("policy has no params")?
            .iter()
            .map(|p| p.as_f64().ok_or("a parameter is not a number"))
            .collect::<std::result::Result<_, _>>()?;
        if params.len() != Self::parameter_count(inputs, hidden) {
            return Err(format!(
                "{} parameters for a {inputs}-input, {hidden}-unit MLP",
                params.len()
            )
            .into());
        }
        Ok(Self {
            inputs,
            hidden,
            params,
        })
    }
}

fn dot(a: &[f64], b: &[f64]) -> f64 {
    a.iter().zip(b).map(|(x, y)| x * y).sum()
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde_json::Value;

    #[test]
    fn the_mlp_matches_mujoco_go2_learning_py() {
        let fixture: Value =
            serde_json::from_str(include_str!("fixtures/mlp_check.json")).expect("fixture parses");
        let hidden = fixture["hidden"].as_u64().unwrap() as usize;
        let params = (0..Mlp::parameter_count(PROPRIOCEPTION, hidden))
            .map(|i| 0.3 * (0.37 * i as f64).sin())
            .collect();
        let mlp = Mlp {
            inputs: PROPRIOCEPTION,
            hidden,
            params,
        };
        let mut observation = [0.0; PROPRIOCEPTION];
        for (slot, value) in observation
            .iter_mut()
            .zip(fixture["observation"].as_array().unwrap())
        {
            *slot = value.as_f64().unwrap();
        }
        let action = mlp.act(&observation);
        for (value, expected) in action.iter().zip(fixture["action"].as_array().unwrap()) {
            assert!((value - expected.as_f64().unwrap()).abs() < 1e-12);
        }
        assert_eq!(Mlp::from_json(&mlp.to_json()).unwrap(), mlp);
    }

    #[test]
    fn a_new_policy_holds_the_default_pose() {
        let mut deviate = 0.0;
        let mlp = Mlp::initial(PROPRIOCEPTION, 32, || {
            deviate += 0.1;
            deviate
        });
        assert_eq!(mlp.params.len(), 32 * 44 + 32 + 12 * 32 + 12);
        assert_eq!(mlp.act(&[0.5; PROPRIOCEPTION]), [0.0; ACTIONS]);
        // With the height scan the policy reads 15 more inputs.
        let scan = crate::go2::Task {
            height_scan: true,
            random_starts: false,
        }
        .observations();
        let wide = Mlp::initial(scan, 32, || 0.1);
        assert_eq!(scan, 59);
        assert_eq!(wide.act(&[0.5; 59]), [0.0; ACTIONS]);
        assert_eq!(Mlp::from_json(&wide.to_json()).unwrap(), wide);
    }
}
