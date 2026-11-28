//! The boundary to learned components: a minimal inference interface.
//!
//! Learned algorithms enter the foresight crates only through
//! [`InferenceModel`]: a fixed-size `f32` vector in, a fixed-size `f32`
//! vector out. An ONNX, TorchScript or accelerator runtime implements the
//! trait in an adapter; [`MlpModel`] is a dependency-free dense network that
//! loads exported weights from JSON, and [`FnModel`] wraps a closure for tests
//! and stand-ins.

use serde::{Deserialize, Serialize};

use crate::ForesightError;

/// Identity and shape of a model.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ModelSpec {
    /// Stable identifier, recorded in [`crate::Provenance::Learned`].
    pub model_id: String,
    /// Input vector length.
    pub input_dim: usize,
    /// Output vector length.
    pub output_dim: usize,
}

/// A learned function with a fixed input and output size.
pub trait InferenceModel {
    /// Identity and shape.
    fn spec(&self) -> &ModelSpec;

    /// Runs the model.
    ///
    /// # Errors
    ///
    /// Implementations return [`ForesightError::DimensionMismatch`] for a
    /// wrong-sized input and [`ForesightError::NonFiniteOutput`] for NaN or
    /// infinite outputs (see [`check_output`]).
    fn infer(&self, input: &[f32]) -> Result<Vec<f32>, ForesightError>;
}

/// Checks a model output's length and finiteness against its spec.
///
/// # Errors
///
/// Returns [`ForesightError::DimensionMismatch`] or
/// [`ForesightError::NonFiniteOutput`].
pub fn check_output(spec: &ModelSpec, output: &[f32]) -> Result<(), ForesightError> {
    if output.len() != spec.output_dim {
        return Err(ForesightError::DimensionMismatch {
            model_id: spec.model_id.clone(),
            expected: spec.output_dim,
            actual: output.len(),
        });
    }
    if output.iter().any(|v| !v.is_finite()) {
        return Err(ForesightError::NonFiniteOutput(spec.model_id.clone()));
    }
    Ok(())
}

fn check_input(spec: &ModelSpec, input: &[f32]) -> Result<(), ForesightError> {
    if input.len() != spec.input_dim {
        return Err(ForesightError::DimensionMismatch {
            model_id: spec.model_id.clone(),
            expected: spec.input_dim,
            actual: input.len(),
        });
    }
    if input.iter().any(|v| !v.is_finite()) {
        return Err(ForesightError::InvalidInput(format!(
            "model `{}` input",
            spec.model_id
        )));
    }
    Ok(())
}

/// Element-wise activation of a [`DenseLayer`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Activation {
    /// No activation.
    Identity,
    /// `max(0, x)`.
    Relu,
    /// `tanh(x)`.
    Tanh,
}

/// One fully connected layer: `y = act(W x + b)`, `W` row-major
/// (`outputs x inputs`).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DenseLayer {
    /// Input size.
    pub inputs: usize,
    /// Output size.
    pub outputs: usize,
    /// Row-major weights, `outputs * inputs` values.
    pub weights: Vec<f32>,
    /// Biases, `outputs` values.
    pub bias: Vec<f32>,
    /// Activation applied after the affine map.
    pub activation: Activation,
}

/// A dense feed-forward network loaded from exported weights.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct MlpModel {
    /// Identity and shape; must match the first and last layers.
    pub spec: ModelSpec,
    /// Layers, applied in order.
    pub layers: Vec<DenseLayer>,
}

impl MlpModel {
    /// Builds and validates a network.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidConfig`] when layer sizes do not chain
    /// or do not match the spec, or a weight is non-finite.
    pub fn new(
        model_id: impl Into<String>,
        layers: Vec<DenseLayer>,
    ) -> Result<Self, ForesightError> {
        let (Some(first), Some(last)) = (layers.first(), layers.last()) else {
            return Err(ForesightError::InvalidConfig(
                "MLP needs at least one layer".into(),
            ));
        };
        let model = Self {
            spec: ModelSpec {
                model_id: model_id.into(),
                input_dim: first.inputs,
                output_dim: last.outputs,
            },
            layers,
        };
        model.validate()?;
        Ok(model)
    }

    /// Parses and validates a network from JSON (the serde form of this type).
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::Parse`] or the [`MlpModel::validate`] error.
    pub fn from_json(json: &str) -> Result<Self, ForesightError> {
        let model: Self =
            serde_json::from_str(json).map_err(|e| ForesightError::Parse(e.to_string()))?;
        model.validate()?;
        Ok(model)
    }

    /// Checks that layer shapes chain and match the spec.
    ///
    /// # Errors
    ///
    /// Returns [`ForesightError::InvalidConfig`].
    pub fn validate(&self) -> Result<(), ForesightError> {
        let bad =
            |m: &str| ForesightError::InvalidConfig(format!("MLP `{}`: {m}", self.spec.model_id));
        let mut width = self.spec.input_dim;
        for layer in &self.layers {
            if layer.inputs != width
                || layer.weights.len() != layer.inputs * layer.outputs
                || layer.bias.len() != layer.outputs
            {
                return Err(bad("layer shapes do not chain"));
            }
            if layer
                .weights
                .iter()
                .chain(&layer.bias)
                .any(|v| !v.is_finite())
            {
                return Err(bad("non-finite weight"));
            }
            width = layer.outputs;
        }
        if self.layers.is_empty() || width != self.spec.output_dim {
            return Err(bad("output size does not match spec"));
        }
        Ok(())
    }
}

impl InferenceModel for MlpModel {
    fn spec(&self) -> &ModelSpec {
        &self.spec
    }

    fn infer(&self, input: &[f32]) -> Result<Vec<f32>, ForesightError> {
        check_input(&self.spec, input)?;
        let mut x = input.to_vec();
        for layer in &self.layers {
            let mut y = layer.bias.clone();
            for (o, out) in y.iter_mut().enumerate() {
                let row = &layer.weights[o * layer.inputs..(o + 1) * layer.inputs];
                *out += row.iter().zip(&x).map(|(w, v)| w * v).sum::<f32>();
                *out = match layer.activation {
                    Activation::Identity => *out,
                    Activation::Relu => out.max(0.0),
                    Activation::Tanh => out.tanh(),
                };
            }
            x = y;
        }
        check_output(&self.spec, &x)?;
        Ok(x)
    }
}

/// A model backed by a closure: for tests, analytic stand-ins and wrapping
/// runtimes that are not worth a dedicated type.
pub struct FnModel<F> {
    spec: ModelSpec,
    f: F,
}

impl<F> FnModel<F>
where
    F: Fn(&[f32]) -> Vec<f32>,
{
    /// Wraps `f` with the given identity and shape.
    pub fn new(model_id: impl Into<String>, input_dim: usize, output_dim: usize, f: F) -> Self {
        Self {
            spec: ModelSpec {
                model_id: model_id.into(),
                input_dim,
                output_dim,
            },
            f,
        }
    }
}

impl<F> std::fmt::Debug for FnModel<F> {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FnModel")
            .field("spec", &self.spec)
            .finish_non_exhaustive()
    }
}

impl<F> InferenceModel for FnModel<F>
where
    F: Fn(&[f32]) -> Vec<f32>,
{
    fn spec(&self) -> &ModelSpec {
        &self.spec
    }

    fn infer(&self, input: &[f32]) -> Result<Vec<f32>, ForesightError> {
        check_input(&self.spec, input)?;
        let out = (self.f)(input);
        check_output(&self.spec, &out)?;
        Ok(out)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tiny() -> MlpModel {
        MlpModel::new(
            "tiny",
            vec![
                DenseLayer {
                    inputs: 2,
                    outputs: 2,
                    weights: vec![1.0, 0.0, 0.0, -1.0],
                    bias: vec![0.0, 0.0],
                    activation: Activation::Relu,
                },
                DenseLayer {
                    inputs: 2,
                    outputs: 1,
                    weights: vec![2.0, 3.0],
                    bias: vec![0.5],
                    activation: Activation::Identity,
                },
            ],
        )
        .unwrap()
    }

    #[test]
    fn mlp_forward_pass() {
        let m = tiny();
        assert_eq!(m.infer(&[1.0, -2.0]).unwrap(), vec![2.0 + 6.0 + 0.5]);
        assert_eq!(m.infer(&[-1.0, 2.0]).unwrap(), vec![0.5]);
    }

    #[test]
    fn mlp_json_roundtrip() {
        let m = tiny();
        let json = serde_json::to_string(&m).unwrap();
        assert_eq!(MlpModel::from_json(&json).unwrap(), m);
    }

    #[test]
    fn wrong_input_size_is_rejected() {
        assert!(matches!(
            tiny().infer(&[1.0]),
            Err(ForesightError::DimensionMismatch {
                expected: 2,
                actual: 1,
                ..
            })
        ));
    }

    #[test]
    fn non_finite_closure_output_is_rejected() {
        let m = FnModel::new("nan", 1, 1, |_| vec![f32::NAN]);
        assert_eq!(
            m.infer(&[0.0]),
            Err(ForesightError::NonFiniteOutput("nan".into()))
        );
    }

    #[test]
    fn broken_shapes_are_rejected() {
        let mut m = tiny();
        m.layers[1].weights.pop();
        assert!(m.validate().is_err());
    }
}
