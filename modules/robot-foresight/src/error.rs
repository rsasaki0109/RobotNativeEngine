//! Errors shared by the foresight crates.

/// Errors raised by forecasts, models and pipelines.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum ForesightError {
    /// A configuration value was non-finite or out of range.
    #[error("invalid configuration: {0}")]
    InvalidConfig(String),
    /// An input value was non-finite or malformed.
    #[error("invalid input: {0}")]
    InvalidInput(String),
    /// A model received or produced a vector of the wrong size.
    #[error("model `{model_id}` dimension mismatch: expected {expected}, got {actual}")]
    DimensionMismatch {
        /// Model identifier from its [`crate::ModelSpec`].
        model_id: String,
        /// Expected length.
        expected: usize,
        /// Actual length.
        actual: usize,
    },
    /// A model produced a non-finite value.
    #[error("model `{0}` produced a non-finite output")]
    NonFiniteOutput(String),
    /// A model could not be parsed.
    #[error("model parse error: {0}")]
    Parse(String),
    /// A search or proposer found no solution.
    #[error("no solution: {0}")]
    NoSolution(String),
}
